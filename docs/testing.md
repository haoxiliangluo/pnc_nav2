# 测试与回归（Testing）

> 最后核对：commit `7eb2a59` + 本轮（2026-10-23）
> 一条命令：`./scripts/run_all_tests.sh`（加 `--sim` 跑仿真冒烟）

---

## 1. 三层测试

| 层 | 内容 | 命令 | 速度 |
|---|---|---|---|
| **L1 单元** | 纯函数/状态机（gtest，不依赖仿真） | `colcon test --packages-select <pkg>` | 秒级 |
| **L2 节点冒烟** | action 生命周期（成功/取消/不可达），**断言 error_code** | `python3 test/action_smoke_test.py <case>` | 数十秒（需仿真） |
| **L3 仿真回归** | 起 `bringup profile:=sim2d` 跑 L2 三用例 | `./scripts/run_all_tests.sh --sim` | 2–3 分钟 |

> 原则：**L1 抓算法与策略，L2 抓接口契约，L3 抓集成**。L3 数量要少（慢且受启动时序影响）。

---

## 2. 当前基线

### L1（30 例，全绿）
| 包 | 用例数 | 覆盖 |
|---|---|---|
| `pnc_nav_map` | 16 | 坡度/台阶/粗糙度、可通行性→代价、高程网格、补洞、噪声回归 |
| `pnc_nav_task` | 7 | 顺序推进、循环、重试、暂停/恢复/取消 |
| `pnc_nav_locomotion` | 5 | 零速中性、幅值随速度、关节运动、急停、频率 |
| `pnc_nav_planners` | 2 | PP2D 正向前进、路径在身后必须原地转（KI-020 回归） |

### L2/L3（冒烟）
| 用例 | 期望 | 实测基线 |
|---|---|---|
| `feedback` | `success=true, error_code=0` | 可达 2m：~10–16s / 1.9m |
| `cancel` | `error_code=4` | 2s 后取消，车停 |
| `unreachable` | `error_code=3` | 3 次恢复后失败，~3–13s |

冒烟脚本**会校验** `error_code`，不匹配则非 0 退出（避免"跑完就算过"）。

---

## 2.5 实验（论文/报告用数据）

```bash
python3 scripts/experiment_runner.py experiments/planner_comparison.yaml   # 跑实验
PYTHONNOUSERSITE=1 python3 scripts/plot_results.py experiments/results/<目录>  # 出图
```

- 配置在 `experiments/*.yaml`（profile / goal / runs / cases），结果落
  `experiments/results/<时间戳>_<实验名>/`（runs.json + summary.csv + plot_*.png）
- 已有实验：`planner_comparison` / `planner_extended`（4 种规划器）/ `costmap_ablation`（静态图 vs 障碍层）/
  `cross_platform`（2D 差速 vs 轮足 Go2-W）

### 实验 1：规划器/跟踪器对比（目标 (2,0)，平地空走廊）

| 用例 | 耗时 (s) | 里程 (m) |
|---|---|---|
| A* + PurePursuit2D | 13.52 / 13.51 | 1.91 |
| A* + DWA2D | 9.47 / 9.50 | 1.86 |
| **RRT\*** + PurePursuit2D | 22.35 / 32.44 | 1.79 / 2.00 |
| 直线规划（示例插件）+ PurePursuit2D | **8.53 / 8.46** | 1.85 |

> RRT* 随机性强（两次 22.3s / 32.4s），空走廊下直线规划最快；DWA 比 PP 更稳。

### 实验 2：代价地图消融（目标 (2,0)，路径上 (1.0,0) 放 0.4m 箱子）

| 用例 | 耗时 (s) | 里程 (m) |
|---|---|---|
| `static_only`（不接激光） | 13.5 / 13.5 / 17.2 | 1.91 |
| `with_obstacle_layer`（激光 + 滚动窗口） | 28.1 / 28.4 / 29.4 | **2.20–2.24** |

> 障碍层让 A* 看到窗口内的箱子并绕行（里程 1.91 → 2.2m）。
> ⚠️ 注意：gz 运行期 spawn 的 `static` 模型**对激光可见，但物理碰撞不生效**（与 KI-016 同类问题），
> 所以消融指标用"是否绕行 / 里程"，不要用"是否碰撞"。实验脚本已加 `setup verified` 校验模型确实存在。

### 实验 3：跨平台对比（目标 (1,0)）

| 平台 | 耗时 (s) | 里程 (m) |
|---|---|---|
| 2D 差速车（`sim2d`） | 10.18 / 10.16 | 0.91 |
| 轮足 Go2-W（`sim_go2w`） | **3.82 / 3.26** | 1.00 / 0.99 |

> 同一套导航框架、同一个目标，换平台只换"运动层插件 + profile"；轮足速度闭环后反而更快。

### 早期基线（planner_comparison，各 3 次）

| 用例 | 耗时 (s) | 里程 (m) |
|---|---|---|
| AStar2D + PurePursuit2D | 9.09 / 13.50 / 13.56 | 1.90 / 1.91 / 1.91 |
| AStar2D + DWA2D | 9.44 / 9.45 / 9.49 | 1.86 / 1.86 / 1.86 |

> DWA 更稳定（耗时方差小）；PP 有一次更快（9.09s）但波动大。
> 注意：出图要用 `PYTHONNOUSERSITE=1`（`~/.local` 的 numpy 2.x 与系统 matplotlib 不兼容）。

## 2.6 3D 定位（LIO-SAM）验证

启动：`gz_sim_3d_lidar_bringup.launch.py localization:=lio_sam`（细节见 [guides/lio_sam_setup.md](guides/lio_sam_setup.md)）

| 检查项 | 方法 | 结果（2026-11-04） |
|---|---|---|
| TF 链路 | `ros2 run tf2_ros tf2_echo map base_link` | `map→base_link` 正常（z≈-0.29，雷达外参） |
| 定位精度 | 直行 0.75m，比对 gz `/odom` 真值 | LIO-SAM 0.76m → **误差 0.01m** |
| 导航闭环 | `test/action_smoke_test.py feedback 1.5 0.0` | `success=True error_code=0 4.69s / 0.65m` |
| 无环境变量运行 | 不设 `LD_LIBRARY_PATH`（只 source ROS+install） | 通过（RPATH 已固化 `/opt/gtsam_sys/lib`） |

> `/lio_sam/mapping/odometry` 是 **BEST_EFFORT** QoS，订阅端需匹配，否则收不到数据。

## 2.7 LIO-SAM 长距离漂移（3 圈矩形闭环）

```bash
# 1) 起仿真（专用世界 lio_room.sdf：12x8m 房间 + 4 立柱 + 2 箱子）
ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py \
  use_gui:=false use_rviz:=false localization:=lio_sam world:=lio_room.sdf
# 2) 桥出 gz 世界真值（注意：不能用 /odom 当真值！）
ros2 run ros_gz_bridge parameter_bridge \
  "/world/lio_room/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
# 3) 跑矩形闭环
python3 test/lio_drift_test.py --laps 3 --forward 4.0 --side 3.0 \
  --gt-topic /world/lio_room/dynamic_pose/info
```

| 配置 | 圈末误差 (m) | ATE RMSE | 最大误差 | 最终航向误差 |
|---|---|---|---|---|
| 关回环 | 0.005 / 0.009 / 0.009 | **0.039 m** | 0.128 m | 0.02° |
| 开回环 | 0.005 / 0.006 / 0.006 | **0.038 m** | 0.110 m | 0.03° |

> 路径实测 30.7 m（真值），3 圈回到起点。漂移量级 4 cm → 在这个场景里回环基本无事可做（两者差异在噪声内）。

### ⚠️ 两个测量陷阱（都踩过，别重复）

1. **不能用 gz 的 `/odom` 当定位真值**：那是轮速里程计，原地转 90° 时会严重打滑，
   3 圈下来"里程"报 41.65 m（实际 30.75 m）。用它当参照物会算出 LIO-SAM "漂移 5.6 m"，
   实际 LIO-SAM 是准的（4 cm）。真值要用 `/world/<world>/dynamic_pose/info`（gz 世界位姿）。
2. **ROS domain 别和别的仿真混**：和 MoveXBot 仿真同 domain 时会看到重复节点，
   甚至两个 LIO-SAM 实例同时往同一话题发位姿。测试固定用 `ROS_DOMAIN_ID=77`。

## 3. 怎么加用例

- **算法/策略** → 加 gtest（参考 `pnc_nav_map/test/` 与 `pnc_nav_task/test/`）
- **接口契约**（action/服务/话题）→ 加 `action_smoke_test.py` 的 case 或新脚本，必须断言结果
- **仿真场景** → 优先写进 world SDF（运行期 spawn 的模型对 3D 激光不可见，见 KI-016）

## 4. 失败定位

| 症状 | 先看 |
|---|---|
| 构建失败 | `/tmp/pnc_build.log` |
| gtest 失败 | `build/**/test_results/**/*.gtest.xml`、`/tmp/pnc_test.log` |
| 仿真冒烟失败 | `/tmp/pnc_sim_bringup.log`、`/tmp/pnc_test.log` |
| 第一个用例报 `TF_FAILURE` | 启动时序：脚本已等待 `map→base_link` 就绪；仍失败查 `ros2 run tf2_ros tf2_echo map base_link` |

## 5. 已知不稳定项

| 项 | 说明 | 缓解 |
|---|---|---|
| 仿真启动时序 | action server 比 TF 先就绪 | 脚本等待 TF 后再跑用例 |
| 残留进程 | 上一个 launch 的 gz 没退干净会导致 TF 两棵树 | 脚本启动前会清 `pnc_nav2/install` 进程 |
| 运行期 spawn 模型 | 3D 激光看不到（KI-016） | 用 world SDF 里的静态障碍 |
| 测试用 domain/partition | 默认 `ROS_DOMAIN_ID=43`、`GZ_PARTITION=pnc_nav_test` | 环境变量可覆盖 |

## 6. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：三层测试、30 例基线、一键脚本与失败定位 |
