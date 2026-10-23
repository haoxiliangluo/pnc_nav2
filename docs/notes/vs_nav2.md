# PNC Nav2 vs 官方 Nav2：逐项对照与取舍

> 最后核对：commit `216e21d`（2026-10-23）
> 用途：面试/评审时"为什么不用 Nav2""你的框架差在哪"的标准答案；也是后续补齐功能的优先级依据
> 对照版本：官方 `navigation2`（Jazzy 分支，`/home/movexbot/navigation2`）

先给结论：**这不是"重写 Nav2"，而是"用 5k 行把导航服务器的最小闭环做透"**。
我们复用 Nav2 的地图服务（`nav2_map_server` + `nav2_lifecycle_manager`），其余自研，目的是把
"规划-控制-恢复-插件化"这条主线讲清楚，并为 3D/四足留出接口。

---

## 1. 结构对照

| 维度 | 官方 Nav2 | PNC Nav2 | 差异带来的后果 |
|---|---|---|---|
| 节点划分 | `planner_server` + `controller_server` + `bt_navigator` + `behavior_server` + `smoother_server` + `velocity_smoother` + `collision_monitor` | **单个 `NavServer`** | 调试简单、代码可通读；但没有独立重规划、无行为编排、扩展性差 |
| 任务编排 | 行为树（XML，可运行时替换） | 6 状态机（IDLE/PLANNING/FOLLOWING/RECOVERING/SUCCEEDED/FAILED） | 可读性高；做不了"先退再转再重规划"的复合恢复 |
| 生命周期 | 全部 `rclcpp_lifecycle` + `lifecycle_manager` + bond | 普通 `rclcpp::Node`（只有 `map_server` 用 lifecycle） | 起停不受控、崩溃无 bond 检测；好处是没有生命周期样板代码 |
| 线程模型 | 每节点可配 executor，回调组隔离 | **单线程 `rclcpp::spin`** | 代码无锁、无竞态；任一插件阻塞会卡整个导航 |
| 目标接口 | `NavigateToPose`（`behavior_tree` 字段 + feedback 含 `navigation_time`/`estimated_time_remaining`/`number_of_recoveries(int16)`） | `NavigateTo3DPose`（`planner_id` + `max_planning_time`；feedback 含 `current_state`、`number_of_recoveries(uint16)`) | 少了"选择行为树"的能力，多了"运行时选规划器"的能力 |
| 错误码 | `error_code` + `error_msg`（uint16，命名常量） | `error_code` + `message`（uint16，8 个枚举） | 方向一致，语义更少（够用即可） |
| 插件体系 | `nav2_core`: GlobalPlanner / Controller / Behavior / Smoother / ProgressChecker / GoalChecker 等 | 4 个基类：GlobalPlanner / LocalPlanner / PathTracker / Locomotion | 少了 GoalChecker/Smoother/Behavior 三类扩展点 |
| 代价地图 | `nav2_costmap_2d` 分层（static / obstacle / voxel / inflation / range）+ **滚动窗口** + 多命名空间 | 单层 `Nav2CostmapAdapter`（订阅 `/map` + 固定膨胀） | **不能感知动态障碍**，这是目前最大的能力缺口 |
| 全局规划 | NavFn、Smac 2D/3D（Hybrid-A*、State Lattice）、Theta* | AStar2D（含 B 样条平滑）、AStar3D、RRTStar3D、示例直线 | 算法够学，缺"运动学约束"（车体转弯半径） |
| 局部控制 | DWB（DWA 官方版，多 critic）、RPP、MPPI、Graceful | DWA2D（有已知缺陷）、PP2D/3D、Stanley3D | 有 DWA 骨架但没 critics 体系；跟踪器更"教学" |
| 恢复行为 | `behavior_server` 插件：Spin / BackUp / DriveOnHeading / Wait，由 BT 组合 | 原地旋转 + 重规划（固定序列） | 最小可用；没有后退、没有按失败原因选策略 |
| 安全/平滑 | `velocity_smoother`、`collision_monitor`、`speed_limit` 话题 | 无 | 真机部署前必须补（急停、限速、碰撞监控） |
| 定位/建图 | `nav2_amcl` + SLAM Toolbox / Cartographer 深度集成 | 复用外部 `amcl_localization`/`cartographer_slam` launch，靠 TF 对接 | 能用，但"定位健康度"没有反馈给 NavServer |
| 多机器人 | 命名空间 + tf 前缀 + 多 costmap | 不支持（`ROS_DOMAIN_ID`/`GZ_PARTITION` 做隔离） | 单机项目够用 |
| 3D / 四足 | 2.5D 为主（Smac 3D 规划、costmap 3D 支持有限）；四足需外接 | 设计目标就是 3D + 四足（`CostmapInterface(x,y,z)`、`LocomotionControllerBase`） | 接口留好了，实现只到 2D |
| 代码量 | 数十万行 | 约 5k 行（core 1.7k + planners 3.0k + examples 0.14k + rviz 0.24k） | 可通读、可讲清；能力自然少 |

---

## 2. 每个取舍的"为什么 / 代价 / 何时补回"

| 决策 | 为什么这么选 | 代价 | 什么时候必须补回 |
|---|---|---|---|
| 单 `NavServer` | 早期要"能讲清一条闭环"，拆成两个 server 会把调试成本翻倍 | 单线程、无独立重规划、无生命周期 | 需要边跑边重规划、或多机器人、或需要节点级故障恢复时 |
| 无行为树 | 状态机 6 个状态就能覆盖 2D 闭环；BT 样板代码多 | 恢复行为无法组合、无法按场景配置 | 出现 ≥3 种恢复策略并需要按失败原因选择时 |
| 单线程 | 无锁、无竞态，教学友好 | 插件慢 = 全局卡 | 单次 `createPlan()` 逼近 100ms，或需要并行定位时 |
| 单层 costmap | 复用 `/map` + 膨胀，一天能跑通 | 无动态障碍、无滚动窗口 | **下一步就要补**（Phase 2 的核心） |
| 不用 `nav2_costmap_2d` | 想自己实现"代价抽象"，为 3D 留口 | 分层/膨胀/滚动窗口全要自己写 | 若要上真机或复杂场景，直接用 `nav2_costmap_2d` 更省 |
| `CostmapInterface::getCost(x,y,z)` | 让 3D（OctoMap/高程图/ESDF）可以无痛替换 | 2D 实现忽略 `z`；每点虚函数调用 | 大图性能瓶颈时（加批量查询/缓存/直接暴露 grid） |
| `LocomotionControllerBase` 先留空 | 划定"导航层输出 `cmd_vel`"的边界，避免和步态耦合 | 目前没有任何实现 | 接 Go2/CHAMP/RL 策略时 |
| 复用 `nav2_map_server` | 地图格式、生命周期、bond 都是现成的 | 引入 `nav2_lifecycle_manager` 依赖 | 不需要改，复用它是对的 |
| 错误码 uint16 | 与 Nav2 一致的惯例，可编程判断 + 日志可读 | 接口升级要同步文档 | 不需要改 |
| 目标入口双轨（action + `/goal_pose`） | 保留旧脚本/RViz 习惯 | 两条路径都要维护 | 外部工具全部迁移后可删兼容入口 |

---

## 3. 能力矩阵（当前）

| 能力 | Nav2 | PNC Nav2 | 说明 |
|---|---|---|---|
| 目标点导航（2D） | ✅ | ✅ | 已实测：成功/取消/不可达/兼容入口 |
| 到达判定 | GoalChecker 插件 | 插件内部各自实现 | 语义不统一，见 `known_issues.md` |
| 动态避障 | ✅（obstacle/voxel 层） | ❌ | 下一步 P1 |
| 重规划 | ✅（BT 触发） | ❌ | 仅规划失败/卡住时重试 |
| 复合恢复 | ✅ | ❌ | 最小恢复 |
| 速度平滑/限速 | ✅ | ❌ | 真机前必须 |
| 多楼层/任务调度 | 不含（需要上层） | ❌ | Phase 5 话题 |
| 3D 规划 | 部分 | 接口有、实现未验证 | Phase 3 |
| 四足步态 | ❌ | 接口有、实现为 0 | Phase 4 |

---

## 4. 面试问答速查（可直接背）

**Q1：为什么不用现成的 Nav2？**
> 目标不是"再造一个 Nav2"，而是把导航服务器的最小闭环吃透：插件契约、状态机、错误码、恢复与看门狗。
> 我复用了它的 `map_server`/`lifecycle_manager` 这类"基础设施"，自研了规划/控制/调度，这样每一行都能讲清为什么。

**Q2：你的框架和 Nav2 差在哪？**
> 差在四块：任务编排（BT）、代价地图分层与滚动窗口、恢复行为库、安全层（速度平滑/碰撞监控）。
> 这四块我都在文档里标了"何时补回"的触发条件——不是不知道，而是按优先级排期。

**Q3：为什么把规划和控制在同一个 NavServer 里？**
> 早期为了可控和可调试：一次会话的所有状态在一个地方，日志能串起来。
> 代价是单线程 + 无生命周期；等需要"边跑边重规划"或多机时，会按 Nav2 的方式拆成 planner/controller 两个 server。

**Q4：插件化带来了什么？**
> 两个东西：一是**算法对比**（同一张图、同一套评估脚本，A* vs RRT*、PP vs DWA）；二是**运行时热插拔**（action goal 里带 `planner_id` 就能换规划器）。
> 代价是错误延后到运行时、以及参数必须遵守 `<实例名>.*` 的命名约定——我在接口文档里把它写成硬约定。

**Q5：怎么扩到 3D 和四足？**
> 两条边界已经划好：地图侧是 `CostmapInterface::getCost(x,y,z)` + `getGroundHeight/getTraversability`（2D 实现只是忽略 `z`）；运动侧是 `LocomotionControllerBase`（`cmd_vel → 关节指令`，导航层不感知腿）。
> 所以 3D 化是换地图实现 + 换规划器，四足化是加一个 locomotion 插件，不需要改 NavServer。

---

## 5. 数据点（面试可引用）

| 项 | 数值 |
|---|---|
| 代码规模 | core 1705 行 / planners 2985 行 / examples 144 行 / rviz 236 行 |
| 依赖 | ROS 2 Jazzy + Gazebo Harmonic（`ros_gz`）+ `nav2_map_server`/`nav2_lifecycle_manager` + pluginlib |
| 插件 | 5 个已注册（AStar2D/AStar3D/DWA2D/PP2D/PP3D）+ 教学示例 1 个 |
| 实测 | 2m 目标：`success=true, code=0, ~16–20s, ~1.9m`；取消 `code=4`；不可达 `code=3`（3 次恢复后失败，12.7s） |
| 接口 | 1 个 action（`NavigateTo3DPose`）、4 个 msg、2 个 srv（2 个 srv 尚未使用） |

---

## 6. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版（commit `216e21d`） |
