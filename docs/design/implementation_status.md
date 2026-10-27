# 实现现状矩阵（设计 vs 代码）

> 最后核对：commit `216e21d`（2026-10-23）
> 用途：治"文档写得比代码超前"的病。`ARCHITECTURE.md` 是**目标架构**，本文是**当前实现**。
> 规则：本文与代码不一致时，以本文 + `design/` 为准，并同步修 `ARCHITECTURE.md`。

---

## 1. 包级对照

| 包（`ARCHITECTURE.md`） | 设计内容 | 现状 | 证据/说明 |
|---|---|---|---|
| `pnc_nav_core` | 插件基类 + NavServer | ✅ 已实现 | `global_planner_base` / `local_planner_base` / `path_tracker_base` / `controller_base` / `costmap_interface` / `nav_server`；**额外**有 `nav2_costmap_adapter` |
| `pnc_nav_planners` | A*/RRT*/DWA/PP/Stanley/MPC/DRL/PCT | 🟡 部分 | 有 `AStar2D`、`AStar3D`、`RRTStar3D`、`DWA2D`、`DWA3D`、`PurePursuit2D/3D`、`Stanley3D`；**没有** MPC/TEB/DRL/PCT |
| `pnc_nav_interfaces` | 3 msg + 2 srv + 1 action | 🟡 部分 | msg/srv 全部生成，但 **`TraversabilityMap`、`PlannerStatus`、`SetMap`、`SwitchPlanner` 零引用**；只有 action 在链路里 |
| `pnc_nav_sim` | Gazebo 世界 + 差速车 + bridge | ✅ 2D/3D 激光车 | `diff_drive.urdf.xacro`、`diff_drive_3d.urdf.xacro`、`bridge.yaml`/`bridge_3d.yaml`；**没有** Go2 模型 |
| `pnc_nav_bringup` | launch + 参数 + 地图 + RViz | ✅ | `gz_sim_2d_bringup`、`amcl_localization`、`cartographer_slam`；`sim_3d_bringup` **不可用**（依赖外部 `go2_config`/`lio_sam`） |
| `pnc_nav_utils` | 可视化 / 评估 / bag 工具 | 🟡 部分 | 有 `evaluation/`（nav_evaluator、planner_tester、goal_recorder、costmap_editor、nav_quality_monitor）；无 `visualization/`、`bag_tools/` |
| `pnc_nav_map` | OctoMap / map_converter / traversability | ❌ 不存在 | 地图只靠 `nav2_map_server` 发 `/map` + 自建 adapter |
| `pnc_nav_localization` | Fast-LIO2 / pose_estimator | ❌ 不存在 | 用外部 AMCL/Cartographer launch，靠 TF 对接 |
| `pnc_nav_control` | locomotion 接口 / RL / MPC 步态 | ❌ 不存在 | 只有 `controller_base.hpp`（`LocomotionControllerBase`），无实现、未接入 NavServer |
| （未在架构图中）`pnc_nav_rviz` | — | ✅ 存在 | 自绘 RViz 面板（移动控制 + 初始位姿 + Cartographer write_state） |
| （未在架构图中）`pnc_nav_examples` | — | ✅ 新增 | 教学示例插件 `StraightLinePlanner`（commit `ed5280c`） |
| （未在架构图中）`pnc_nav_map` | — | 🟡 新增 | 2.5D 高程图 + 可通行性 + 高程节点（Phase 3.1） |
| （未在架构图中）`pnc_nav_diagnostics` | — | ✅ 新增 | 健康监控（`/diagnostics`：map/odom/sensor/tf/action） |
| （未在架构图中）`pnc_nav_task` | — | ✅ 新增 | 任务层骨架（多点顺序/重试/暂停取消 + `/mission_status`） |
| （未在架构图中）`pnc_nav_locomotion` | — | ✅ 新增 | 运动层：FakeTrot + **IK walk/trot 步态**；`cmd_vel → /joint_commands → gz 关节`（开环） |
| （新增仿真资产）`quadruped.urdf.xacro` | — | 🟡 极简四足 | 4 腿 × 3 关节 + JointPositionController（对照用） |
| （新增仿真资产）`go2_description`（vendored） | — | ✅ 开源模型 | Unitree 官方 Go2 模型（BSD），配 gz 关节位置控制器 |

**`third_party/`**：实际只有 `cartographer`、`cartographer_ros`、`turtlebot3*`、`simdog_go2`（submodule，指向作者本地路径，克隆后为空）。
架构图里写的 `octomap/`、`grid_map/`、`unitree_ros2/` **不存在**。

---

## 2. 接口对照

### 2.1 Topic（架构图 §5.1 vs 实际）

| 架构图写的 | 实际 | 备注 |
|---|---|---|
| `/goal_pose`（"接收目标"，唯一入口） | `/goal_pose` 仍在，但只是**兼容入口** | 主入口换成 action |
| `/cmd_vel` | ✅ | 20Hz，退出时补零速 |
| `/global_plan`、`/local_plan` | ✅ | local_plan 由插件提供 |
| `/odom` | ✅ | 只用 `twist` |
| `/map` | ✅ | 来自 `nav2_map_server` |

### 2.2 Service（架构图 §5.2）

| 架构图写的 | 实际 |
|---|---|
| `SetMap`、`SwitchPlanner` | **定义了但没实现**（零引用）。切换插件目前用参数或 action 的 `planner_id` |

### 2.3 Action（架构图 §5.3）

| 架构图写的 | 实际 |
|---|---|
| `NavigateTo3DPose` | ✅ 已实现（commit `4c0cad3`）：feedback / result / cancel / 抢占 / `error_code` |

---

## 3. 路线图对照

| 阶段（`ARCHITECTURE.md` / `NAV2_LEARNING_...md`） | 计划内容 | 实际 |
|---|---|---|
| Phase 1：2D 仿真验证 | 工作空间、插件框架、Gazebo、A*、DWA、PP、评估 | ✅ 完成（`docs/phase1_验证清单.md`），但**架构图里的勾选框从未更新（全 `[ ]`）** |
| Phase 2：3D 定位与建图 | Fast-LIO2/LIO-SAM、OctoMap、高程图、ESDF、可通行性 | 🟡 只做了 2D 的 AMCL + Cartographer；3D 部分未开始 |
| Phase 3：3D 规划与控制 | 3D 全局/局部、可通行性代价 | ❌ 代码里有 3D 插件但没在 3D 地图上验证（`use_3d: false`，`getGroundHeight/getTraversability` 零调用） |
| Phase 4：机器狗仿真 | Go2 + 步态 | ❌ 只有 `LocomotionControllerBase` 基类；`sim_3d_bringup` 不可用 |
| Phase 5：进阶与真机 | 任务层、性能、部署 | ❌ 未开始（`ARCHITECTURE` 的 Mission Layer 只有图） |
| （实际完成但没写进路线图） | — | Action 接口 + 最小恢复闭环、3D 激光仿真车、RViz 面板、教学示例插件 |

---

## 4. 代码规模（当前）

| 包 | 文件数 | 行数 |
|---|---|---|
| `pnc_nav_core` | 10 | 1705 |
| `pnc_nav_planners` | 16 | 2985 |
| `pnc_nav_examples` | 2 | 144 |
| `pnc_nav_rviz` | 2 | 236 |
| 其余（sim/bringup/interfaces/utils） | 多为 launch/yaml/脚本/消息定义 | 不计 |

---

## 5. 同名但语义不同（容易误读的地方）

| 名字 | 设计意图 | 当前实际 |
|---|---|---|
| `AStar3D` / `RRTStar3D` / `DWA3D` / `PurePursuit3D` / `Stanley3D` | 3D 空间规划/控制 | 只有 2D 逻辑跑通；3D 只在 `CostmapInterface` 上保留了 `z` 参数，实际 `use_3d: false` |
| `TraversabilityMap.msg` | 可通行性地图消息 | 定义存在、零引用 |
| `LocalPlanner` vs `PathTracker` | 前者避障、后者纯跟踪 | 二选一，DWA2D 是唯一"带避障"的实现，且有已知缺陷 |
| `sim_3d_bringup.launch.py` | 3D 完整导航（Go2 + LIO-SAM） | 依赖外部包，**当前跑不起来** |

---

## 6. 建议的更新顺序

1. 修 `ARCHITECTURE.md`：文首加"本文为目标架构，现状见 `docs/design/implementation_status.md`"；
   删掉/标注不存在包；勾选 Phase 1；把 action 接口补进 §5.3
2. 把本文 §1 的"❌"项拆成 `plan/phase3_3d_design.md` 的输入（不是马上实现）
3. 每完成一个阶段，更新本文 + 对应 `phaseN_验证清单.md`（文档与代码同 PR）

---

## 7. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：包/接口/路线图三维度对照（commit `216e21d`） |
