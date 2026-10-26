# 地图与代价地图（Map & Costmap）

> 最后核对：本次提交（2026-10-23）
> 读者：要改避障/代价地图的人、要调参的人
> 代码：`nav2_costmap_adapter.*`（静态层）、`rolling_costmap.*`（局部滚动层）、`nav_server.cpp`

---

## 1. 为什么是两层

对齐 Nav2 的 global/local 概念，但只用两个类实现：

| 代价地图 | 类 | 数据源 | 给谁用 | 范围 |
|---|---|---|---|---|
| **全局** | `Nav2CostmapAdapter` + `OverlayCostmap` | `/map` 静态图 + 局部窗口内的激光障碍叠加 | `GlobalPlannerBase` 插件 | 整张地图（615×1334 @0.05） |
| **局部** | `RollingCostmap` | 静态图（可选）+ `/nav_scan` 激光 | `LocalPlannerBase` 插件 | 8m×8m 滚动窗口，机器人居中 |

```
                /map ─────────────▶ Nav2CostmapAdapter ──▶ 全局规划器（A* 等）
                                          │
                                          │（静态障碍，可选叠加）
                                          ▼
/nav_scan ──▶ 障碍点(map系, TTL) ──▶ RollingCostmap ──▶ 局部规划器（DWA2D 等）
                                          │
                                          ├─▶ /costmap（合并窗口，2Hz）
                                          └─▶ /obstacle_layer（仅激光障碍，2Hz）
```

**为什么全局不滚动**：全局规划器要看到整张地图才能做长路径规划；如果把窗口也给它，A* 只能看 8m，
2m 以外的目标直接"无解"。所以全局保持静态整图，滚动窗口只给局部规划器。

**但全局需要"看得见"窗口里的动态障碍**，否则 A* 每次都plan一条"穿过障碍"的直线。做法是
`OverlayCostmap(base=静态, overlay=滚动窗口)`：范围/分辨率/坐标系跟随静态层（可见整图），
查询时**落在窗口内就用窗口代价**，窗口外回退静态层。这样 A* 重规划时天然会绕过箱子。

---

## 2. RollingCostmap 工作方式

```
① 扫描回调（LaserScan）
   - 用 TF(map ← 激光帧) 把每束命中点投到 map 系，跳过无效/超量程/贴脸点
   - 存进 deque<{x, y, steady_clock 时间}>，上限 20000 点

② update(robot_pose)（每控制周期，20Hz）
   - 过期：删除超过 obstacle_ttl（默认 3s）的点
   - 重建条件：有新扫描（dirty）或机器人移动 > 0.25m
   - 重建：窗口居中机器人（8m×8m，0.05m/cell → 160×160）

③ 重建时
   - 静态障碍：逐格查 /map 栅格（O(1)/cell，受 include_static 控制）
   - 激光障碍：障碍点落格
   - **raytrace 清除**：用"最新一帧"的每条光束把沿途格子置空闲（命中格再补回）；
     超量程/无效的光束按 max_range 截断后同样参与清除（只清不标）
   - 膨胀：对障碍格按 inflation_radius 一次性预计算（指数衰减，与静态层同公式）
   - 结果存 merged_cost_（静态+激光）与 obstacle_cost_（仅激光）

④ 查询（规划器每格调用）
   - getCost/isOccupied：O(1) 查表（不是每次扫邻域）
   - isInBounds：窗口内为真；窗口外返回 UNKNOWN（unknown_as_occupied=false）
```

**关键设计**：膨胀在重建时预计算，查询 O(1)。旧版 `Nav2CostmapAdapter::getCost` 是"每次查询扫
7×7 邻域"，A*/DWA 每格都调 → 慢；滚动层把这份开销摊到 10Hz 的重建上。

---

## 2.1 点云障碍层（3D 激光，2.5D 投影）

3D 激光车（`gz_3d_lidar.launch.py`）输出 `sensor_msgs/PointCloud2` 到 `/points`（Velodyne 16 线，
水平 360°、垂直 ±15°、10Hz）。`RollingCostmap` 用同一套窗口/膨胀逻辑，把点云**压到 2D**：

```
/points ──TF(velodyne→map)──▶ 逐点变换
   ├ 降采样：每 cloud_stride 个点取一个（默认 8，约 3.6k 点/帧）
   ├ 距离过滤：0.3m < 水平距离 ≤ cloud_max_range（默认 8m）
   ├ 高度带过滤：cloud_min_z ≤ z ≤ cloud_max_z（默认 0.10–1.50m，滤掉地面与高处结构）
   └ 投影到 2D 栅格 → 与 2D 扫描共用障碍点队列（TTL 过期）
```

**为什么是 2.5D 投影而不是真 3D**：轮式底盘只关心"能不能从这一列通过"，把点云按高度带压成
2D 障碍图就够用，代价是**会丢掉"从下方可钻过去"这类信息**（比如桌底）。真正的 3D 规划
（OctoMap / 高程图 / 可通行性）属于 Phase 3，接口已经在 `CostmapInterface` 里留好
（`getGroundHeight` / `getTraversability` / `getDistanceToObstacle`）。

**2D 扫描与点云二选一**：配置了 `cloud_topic` 就只订阅点云，不再订阅 `scan_topic`
（3D 车上没有 `nav_scan`，避免刷 "cannot transform laser" 告警）。

### 实测（3D 激光车）

| 项 | 结果 |
|---|---|
| `/points` 频率 | ~4.4Hz（gz 桥接后） |
| 窗口占用格 | **4927**（2D 扫描为 1079；3D 覆盖更全，含墙体沿高度方向的投影） |
| 3D + PurePursuit 2m | `success=true, 10.85s, 1.90m` |
| 3D + DWA，箱子挡在 2m、目标 4m | `success=true, 23.17s, 4.07m`（绕行） |
| 启动方式 | `ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py` |

## 3. 参数（`local_costmap.*`）

| 参数 | 默认 | 单位 | 说明 / 调参建议 |
|---|---|---|---|
| `enable` | true | — | 关掉后局部规划器退化为直接用静态图（旧行为） |
| `scan_topic` | `nav_scan` | — | 2D 激光话题；3D 车需要先转成 LaserScan |
| `window_size` | 8.0 | m | 窗口边长。太小：看不见远处障碍；太大：重建变慢 |
| `resolution` | 0.05 | m/cell | 窗口分辨率，建议与静态图一致 |
| `obstacle_ttl` | 3.0 | s | 障碍点存活时间。太短：障碍闪烁；太长：残影 |
| `max_range` | 6.0 | m | 只用这个范围内的激光，超出视为"无障碍" |
| `inflation_radius` | 0.35 | m | 膨胀半径，应 ≥ `robot_radius` |
| `robot_radius` | 0.25 | m | 内切半径；膨胀小于它时判 LETHAL |
| `cost_scaling_factor` | 3.0 | — | 衰减系数，越大衰减越快（代价越"尖"） |
| `include_static` | true | — | 是否叠加静态图障碍（激光已能看到大部分墙时影响很小） |
| `unknown_as_occupied` | false | — | 窗口外是否当障碍。局部规划建议 false（否则边界会"撞鬼"） |
| `cloud_topic` | "" | — | 点云话题；填 `points` 启用 3D 点云障碍层（与 scan 二选一） |
| `cloud_stride` | 8 | — | 每 N 个点取一个（降采样） |
| `cloud_min_z` / `cloud_max_z` | 0.10 / 1.50 | m | 高度带（map 系），滤地面与高处结构 |
| `cloud_max_range` | 8.0 | m | 点云水平距离上限 |

---

## 4. 验证证据（实测，2026-10-23）

| 项 | 结果 |
|---|---|
| 窗口尺寸 | `/costmap` = **160×160 @0.05m**（8m 窗口） |
| 滚动 | 机器人 (0,0) → 原点 (-4.00,-4.00)；走到 (1.91,0.01) → 原点 (-2.09,-3.99) |
| 障碍检测 | 静止时窗口内 occupied **1079** 格；机器人移动后 **1672** 格 |
| 静态叠加是否生效 | `include_static:=false` 时 1076 格（静态贡献 ~3 格，激光是主来源） |
| 动态障碍 | 在 Gazebo 里 spawn 一个 0.4×0.4 的箱子到 (3.0, 0)：`/obstacle_layer` 在 (3.0,0) 半径 0.5m 内出现 **217–256** 个占用格 |
| raytrace 清除 | 箱子在 2.0s 被 `gz service /world/world_demo/remove` 删除后，占用格在 **~0.9s 内从 219 降到 51**（剩余 51 格是世界里真实存在、但静态图未标注的几何，不是残影） |
| 绕障回归 | 0.4m 箱子挡在 2.0m 处、目标 4.0m：`success=true, code=0, 21.7s, 4.07m`（直行仅 4.0m） |
| 验证脚本 | `python3 test/check_costmap_layers.py`（打印尺寸/原点/占用格数） |

---

## 5. 已知限制

1. **激光视野只有 240°**（`angle_min/max = ±2.09`）：机器人**背后 120° 是盲区**，障碍层不会标记；
   倒车/后退类恢复行为前必须额外处理（见 `known_issues.md` KI-014）。
2. **raytrace 清除只作用于"最新一帧"**：光束扫过的地方会立刻清空，但**没有光束覆盖的格子**
   仍靠 TTL 过期（窗口跟机器人走，不跟传感器朝向走）。实测箱子被移走后 ~0.9s 内清掉
   （其中 ~0.5s 是 `/obstacle_layer` 2Hz 可视化的发布间隔，内部代价地图更新更快）。
3. **只有一层障碍 + 膨胀**：没有 voxel 层、没有多图层插件、没有多机器人命名空间。
4. **静态层与激光层不做一致性融合**：同一堵墙可能出现 1–2 格错位（地图与仿真世界的偏差）。
5. **窗口重建有阈值**（移动 0.25m 或新扫描）：机器人极慢移动时，窗口边缘的障碍可能短暂滞后。
6. **局部规划器依赖它**：`PathTrackerBase`（PurePursuit/Stanley）不接收代价地图，
   换回 tracker 时障碍层只影响可视化，不影响控制（DWA2D 已修复"只转不走"，见 KI-013）。
7. **没有"不可达"判定**：窗口被完全堵死时，A* 仍可能规划出穿过障碍的路径（静态图里没有），
   DWA 会卡住并走恢复流程，最终以 `error_code=3` 结束。

---

## 6. 调试

```bash
# 看窗口是否跟着机器人
python3 test/check_costmap_layers.py

# RViz：加两个 Map 显示
#   /costmap        —— 合并窗口（静态 + 激光 + 膨胀）
#   /obstacle_layer —— 只有激光障碍（确认"障碍层真的在工作"）
ros2 topic hz /costmap          # 期望 ~2Hz
```

`/costmap` 是 **8m 窗口**，不是全图。想看整图用 `/map`（静态）或 RViz 的 Map 显示 `/map`。

---

## 7. 与 Nav2 的差距（下一步）

| Nav2 | 本实现 |
|---|---|
| `local_costmap` + `obstacle_layer` + `inflation_layer` + `voxel_layer` | `RollingCostmap`（obstacle + inflation 精简版） |
| raytrace 清除（光线穿过的格子置空闲） | ✅ 已实现（仅最新一帧；无光束覆盖的格子靠 TTL） |
| 多图层插件化（layer 可插拔） | 单类硬编码 |
| 滚动窗口 + 全局/局部双图 | ✅ 已对齐（双图） |
| 多机器人命名空间 | 无 |

真正要补的顺序：**真 3D 表示（OctoMap/高程图）→ 图层插件化 → 多机器人命名空间**。
（`voxel` 的实用子集——点云 2.5D 投影——已经实现，见 §2.1。）

---

## 8. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：双代价地图 + 障碍层 + 滚动窗口 + 实测数据 |
| 2026-10-23 | 全局层改为 `OverlayCostmap`（静态整图 + 窗口障碍），A* 可绕行动态障碍；DWA2D 修复 + 绕障实测 |
| 2026-10-23 | 新增 raytrace 清除（含超量程光束），障碍消失后 ~0.9s 内清除；补充实测数据 |
| 2026-10-23 | 新增点云障碍层（3D 激光 2.5D 投影）+ 3D bringup；实测 3D 车可绕障 |
