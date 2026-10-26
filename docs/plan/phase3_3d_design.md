# Phase 3 设计草案：3D 可通行性与规划

> 最后核对：commit `dcf7568`（2026-10-23）
> 状态：**设计草案（未实现）**，用于指导接下来的编码；实现后把"状态"改成"已实现"并补实测数据
> 读者：要动手写 Phase 3 的人 / 评审方案的人
> 相关：[design/map_and_costmap.md](../design/map_and_costmap.md)、[design/implementation_status.md](../design/implementation_status.md)、[notes/decisions.md](../notes/decisions.md)

---

## 1. 目标与范围

**目标**：让"3D 数据"真正影响规划决策——从"点云压平面"升级到"高程 + 可通行性"，并让全局规划器按地形代价选路。

**本期做**：
- 2.5D 高程图（elevation map）构建与维护
- 可通行性（坡度 / 台阶 / 粗糙度）计算与发布
- 用可通行性代价替换"二值占用"作为规划代价
- `AStar3D` 在 2.5D 图上真正跑通（现在只有 2D 验证）

**本期不做**（明确划掉，避免范围膨胀）：
- 真 3D 体素/ESDF 规划（OctoMap、Voxblox）
- 四足步态（`LocomotionControllerBase` 仍只有基类）
- 多楼层/任务层
- 3D 定位（Fast-LIO2/LIO-SAM 接入单独排期）

---

## 2. 现状盘点

| 已有 | 位置 | 说明 |
|---|---|---|
| 3D 激光数据 | `pnc_nav_sim`（Velodyne 16 线）→ `/points` | 已接入局部代价地图（2.5D 投影，见 `map_and_costmap.md` §2.1） |
| 3D 接口 | `CostmapInterface` | `getGroundHeight` / `getTraversability` / `getDistanceToObstacle` 已定义，**2D 实现未实现/未使用** |
| 3D 规划器 | `pnc_nav_planners/AStar3D` | 有 `use_3d` 与 26 邻域，但**从未在 3D 地图上验证**（`use_3d: false`） |
| 消息定义 | `pnc_nav_interfaces/msg/TraversabilityMap.msg` | 已含 `elevation` / `slope` / `roughness` / `traversability` 数组，**零引用** |
| 运动层边界 | `controller_base.hpp` | `LocomotionControllerBase` 只有基类，未接入 NavServer |

| 缺 | 影响 |
|---|---|
| 高程图构建 | 现在点云只被压成 2D 障碍，斜坡/台阶信息全部丢失 |
| 可通行性 | 无法区分"平地 0.2m 障碍"和"0.3m 高的台阶"这类地形差异 |
| 3D 代价接入 | 规划器仍是二值/膨胀代价，不是地形代价 |
| 定位 | 仍靠固定 `map→odom`（仿真）或外部 AMCL |

---

## 3. 方案选型：2.5D 高程图（推荐） vs 真 3D OctoMap

| 维度 | 2.5D 高程图 + 可通行性（**推荐**） | 真 3D 体素 / OctoMap |
|---|---|---|
| 适用对象 | 地面/轮足机器人（本项目定位） | 无人机、复杂多层结构 |
| 内存/算力 | 8m×8m@0.05 → 25.6k 格，可实时重建 | 同区域 3D 体素数量级更大 |
| 可解释性 | 坡度/台阶/粗糙度都能画出来、讲清楚 | 需要切片才能可视化 |
| 与现有代码契合 | `CostmapInterface` 的 `getGroundHeight/getTraversability` 天然对应 | 需要新的 3D 碰撞模型（机器人 3D 包围体） |
| 主要代价 | 桌底/桥洞这类"下方可穿过"空间丢失 | 内存/搜索成本高、调试难 |

**决策（ADR 追加）**：先用 2.5D 高程图把"地形影响规划"这条链打通；OctoMap 作为
`CostmapInterface` 的另一种实现保留，等有"多层/桥洞"需求再切。这也是 ADR-006
（接口抽象）真正的兑现点。

---

## 4. 目标架构与数据流

```
/points ──TF──▶ ElevationBuilder ──▶ ElevationMap（每格：z_min/z_max/置信度）
                                          │
                        ┌─────────────────┼─────────────────┐
                        ▼                 ▼                 ▼
                  Traversability      CostmapInterface   /traversability_map
                  (slope/step/rough)  (getCost 等)        (RViz/评估)
                        │                 │
                        └────────┬────────┘
                                 ▼
                     AStar3D / DWA（地形代价）
                                 ▼
                             cmd_vel
```

**关键点**：`ElevationMap` 是新的 `CostmapInterface` 实现；规划器**不改代码**，只是换了一个
costmap（这正是 ADR-006 想要的效果，Phase 3 是它的验收）。

---

## 5. 模块划分（建议新增 `pnc_nav_map` 包）

```
src/pnc_nav_map/
├── include/pnc_nav_map/
│   ├── elevation_map.hpp        # 2.5D 高程网格（z_min/z_max/更新计数）
│   ├── elevation_builder.hpp    # /points → ElevationMap（TF、降采样、高度带）
│   ├── traversability.hpp       # slope / step / roughness → [0,1]
│   └── elevation_costmap.hpp    # CostmapInterface 实现（给规划器）
├── src/...
├── config/elevation_params.yaml
└── test/                        # 纯函数单测：坡度估计、台阶检测、代价映射
```

与现有层的关系：

| 层 | 现在 | Phase 3 |
|---|---|---|
| 全局代价地图 | `Nav2CostmapAdapter`（静态 2D） | `OverlayCostmap(静态, ElevationCostmap)` |
| 局部代价地图 | `RollingCostmap`（2D 点云投影） | 保留（实时避障） + 地形代价可叠加 |
| 全局规划 | `AStar2D`（静态图） | `AStar3D`（2.5D，`use_3d: true`） |

---

## 6. 接口契约（ElevationCostmap）

`ElevationCostmap` 实现 `CostmapInterface`：

| 方法 | 语义（Phase 3 约定） |
|---|---|
| `getGroundHeight(x, y)` | 该格高程（米，map 系）；无数据返回 NaN |
| `getTraversability(x, y, z)` | `[0,1]`：`min(slope_score, step_score, roughness_score)`；无数据返回 `-1` |
| `getCost(x, y, z)` | 由可通行性映射到 `[0,254]`：`cost = (1 - trav) * 253`，不可通行直接 `LETHAL` |
| `isOccupied(x, y, z)` | `trav < 0.3` 或 `|z - ground| > max_step` |
| `getDistanceToObstacle(x, y)` | 2D 距离变换（对不可通行格做 BFS/距离场），用于贴近惩罚 |
| `isInBounds` / `getBounds` / `getResolution` | 与窗口/整图一致（沿用滚动窗口策略） |

**可通行性定义（第一版，可解释优先）**：

```
slope     = atan(|∇z|)                    # 高程梯度
step      = max(|z_max - z_center|)       # 邻域高差（台阶）
roughness = std(z in cell)                # 格内点云 z 的方差/极差

score(x)  = clamp(1 - slope/slope_max, 0, 1) *
            clamp(1 - step/step_max, 0, 1) *
            clamp(1 - roughness/rough_max, 0, 1)
```

参数（第一版默认）：`slope_max = 20°`、`step_max = 0.10m`、`rough_max = 0.05m`。

---

## 7. 分阶段落地与验收

### 3.1 高程图 + 2.5D 可通行性（1–2 天）🟡 进行中
- ✅ `ElevationMap`（z 均值/min/max/count）+ `traversability` 纯函数 + 15 个 gtest 用例
- ✅ `ElevationBuilder`：`/points` → 高程图（TF、stride 降采样、高度带、滚动窗口、`fillUnknown` 补洞）
- ✅ `ElevationCostmap`：实现 `CostmapInterface`（getCost / isOccupied / getTraversability / getGroundHeight）
- ✅ 节点 `elevation_map_node`：发布 `/elevation_map`（OccupancyGrid）+ `/traversability_map`（TraversabilityMap）
- ⬜ 斜坡/台阶 world 与 RViz 调参（下一步）
- **实测（3D 激光车，8m 窗口 @0.1m）**：平地可通行性 0.90–0.98；箱子所在格 0.25（被判为难以通行）；
  补洞后 6400/6400 格全部有数据；机器人移动后窗口原点从 (-4.00,-4.00) 滚到 (-2.90,-4.04)
- **仍待验收**：15° 斜坡 / 10cm 台阶场景、`getGroundHeight` 误差 < 0.05m

### 3.2 地形代价接入规划（1 天）
- `ElevationCostmap` 实现 `CostmapInterface`，`AStar3D` 用 `use_3d: true` 规划
- **验收**：给定"平地直行 vs 绕开陡坡"两条路，规划选择缓坡路线；路径点 z 跟随高程

### 3.3 局部地形代价（1 天）
- `RollingCostmap` 叠加地形代价（不可通行区域直接 LETHAL）
- **验收**：DWA 在斜坡边缘不再"贴边"，台阶前会绕行或停下

### 3.4 定位接口预留（0.5 天，不实现算法）
- 文档写明 `map→odom` 由 AMCL / LIO-SAM / Cartographer 提供，NavServer 只查 TF
- **验收**：切到 `amcl_localization.launch.py` 后 3D 链路仍能跑（定位质量另说）

---

## 8. 验证场景与指标

**仿真场景（新增 world）**：
| 场景 | 目的 |
|---|---|
| `ramp.sdf`：15°/25° 斜坡 | 坡度代价是否生效、路径是否选缓坡 |
| `step.sdf`：8cm / 15cm 台阶 | 台阶阈值是否区分"能过 / 不能过" |
| `rough.sdf`：碎石/瓦砾地面 | 粗糙度代价 |
| 现有 `playground.sdf`（平地 + 货架） | 回归，不能比 Phase 2 差 |

**指标**（写进 `docs/phase3_验证清单.md`）：
- 高程误差：对已知斜坡/台阶的 `getGroundHeight` 误差（目标 < 0.05m）
- 可通行性分类：人工标注 50 格，准确率（目标 ≥ 85%）
- 规划：成功率、路径坡度分布（max/mean）、耗时
- 控制：到达成功率、最小障碍距离、恢复次数

---

## 9. 风险与开放问题

| 风险 | 说明 | 缓解 |
|---|---|---|
| 点云稀疏 | 16 线在 8m 处线间隔 ~0.4m，高程格 0.05m 会大量空洞 | 格内累积 + 邻域插值；必要时降分辨率到 0.1 |
| 地面估计 | 坡度/台阶依赖准确地面，噪声大会误判 | 先用局部平面拟合（RANSAC）做地面分割 |
| 桌底/桥洞 | 2.5D 表示天生丢失 | 文档标注；需要时切 OctoMap（接口不变） |
| 定位漂移 | 3D 代价对位姿敏感 | 先仿真验证；真机阶段接 LIO-SAM |
| 性能 | 高程图重建 + A* 3D 搜索 | 窗口化（复用滚动窗口思路）、增量更新 |

---

## 10. 第一步任务清单（可直接开工）

1. ✅ `pnc_nav_map` 包骨架（CMakeLists/package.xml），`ElevationMap` 数据结构 + 单元测试
2. `/points` → `ElevationMap` 的构建器（TF、降采样、高度带、累积更新）
3. `Traversability` 纯函数 + 单测（给定合成高程，验证 slope/step/roughness）
4. `ElevationCostmap` 实现 `CostmapInterface` + 发布 `TraversabilityMap`
5. 新 world（ramp/step）与 launch，RViz 观察→调参
6. 接 `AStar3D`（`use_3d: true`），补 `docs/phase3_验证清单.md`

---

## 11. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版设计草案（commit `dcf7568`） |
| 2026-10-23 | 3.1 第一步完成：`pnc_nav_map` 包 + `ElevationMap` + `traversability` + 10 个 gtest 全绿 |
| 2026-10-23 | 3.1 主体完成：ElevationBuilder（含 fillUnknown 补洞）+ ElevationCostmap + 节点双话题；15 个 gtest 全绿 |
