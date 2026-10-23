# 决策记录（ADR）

> 最后核对：commit `216e21d`（2026-10-23）
> 用法：面试/评审被问"为什么这么设计"时按编号引用；改设计时新增一条，不覆盖旧条目。
> 模板：背景 → 决策 → 备选与不选的理由 → 后果 → 重审触发条件

---

## ADR-001 插件化：用 pluginlib 暴露规划/控制算法

**状态**：已采纳（2026-10）

**背景**：目标是"能对比、能替换"——同一张地图上跑不同规划器/控制器，且不改框架代码。

**决策**：所有算法实现 `pnc_nav_core` 里的 4 个纯虚基类，通过 `pluginlib` 动态加载；插件在
`plugins.xml` 注册，参数挂在 `<实例名>.*` 命名空间。

**备选**：① 编译期策略（模板/if-else）——最简单，但换算法要重编译；② 自定义工厂 + `dlopen`——重复造轮子。

**后果**：
- 正：算法对比零成本（改 yaml 或 action 里的 `planner_id`）；插件可独立编译
- 负：类型擦除导致错误延后到运行时；参数命名要守约定（否则插件互相串味）；接口变更要改所有插件

**重审触发**：插件总数 < 2 且无对比需求时，可退回编译期直连。

---

## ADR-002 单个 `NavServer` 承载规划 + 控制调度

**状态**：已采纳（2026-10），**已知是阶段性选择**

**背景**：早期只有一条闭环，需要"一个登录点看全所有状态"。

**决策**：把全局规划、局部规划/跟踪、状态机、恢复、发布全部放在 `NavServer` 一个节点里，
用 6 个状态（IDLE/PLANNING/FOLLOWING/RECOVERING/SUCCEEDED/FAILED）串起来。

**备选**：照搬 Nav2 拆 `planner_server` + `controller_server` + `bt_navigator`——学习曲线陡、早期收益低。

**后果**：
- 正：日志串行可读、状态机一眼看懂、调试成本低
- 负：无法"边跑边重规划"（无独立控制循环）、无生命周期、无法单独重启某一层

**重审触发**：需要周期性重规划、或多机器人、或要接入任务层（Phase 5）时，按 Nav2 的边界拆分。

---

## ADR-003 单线程执行器

**状态**：已采纳（2026-10）

**背景**：`NavServer` 持有可变会话状态（目标、路径、计数、看门狗），多线程要处处加锁。

**决策**：`rclcpp::spin` 单线程，所有 action/timer/subscription 回调串行；代码里不写锁。

**备选**：`MultiThreadedExecutor` + 回调组隔离（Nav2 的做法）——吞吐高但竞态风险大。

**后果**：
- 正：零竞态、代码短、行为可预测
- 负：任一插件慢（`createPlan` 100ms+）会拖慢整个控制循环

**重审触发**：单次规划耗时超过控制周期的 50%，或需要并行跑定位/感知时。

---

## ADR-004 目标入口：action 为主 + `/goal_pose` 兼容

**状态**：已采纳（2026-10，commit `4c0cad3`）

**背景**：早期只有 `/goal_pose` 话题（RViz "2D Goal Pose" 直接发），没有 feedback/result/cancel。

**决策**：新增 `navigate_to_3d_pose` action 作为主入口；`/goal_pose` 保留，内部转成"不带 planner_id 的 goal"。

**备选**：直接删掉 `/goal_pose`——会破坏已有 RViz 配置和脚本。

**后果**：
- 正：有 feedback/result/取消/抢占；旧工具仍可用
- 负：两条路径都要维护；`/goal_pose` 没有 feedback（只能看日志）

**重审触发**：外部工具全部迁移到 action 后，删除兼容入口。

---

## ADR-005 错误码：`uint16 error_code` 放在 Result

**状态**：已采纳（2026-10）

**背景**：需要机器可判定的失败原因（脚本/任务层用），又要人能看懂。

**决策**：`Result` 里放 `uint16 error_code` + `string message`；枚举 0–7（成功/无路径/规划超时/重试耗尽/取消/TF失败/无效目标/无插件）。

**备选**：① 只用字符串 message——脚本解析脆弱；② 独立 `PlannerStatus` 话题——异步、易丢。

**后果**：
- 正：与 Nav2 的 `error_code` 惯例一致；日志一行可 grep（`error_code=3`）
- 负：新增失败类型要改 action 定义（需重新编译所有依赖包）并同步文档

**重审触发**：错误类型超过 16 个或需要携带结构化上下文（如失败点坐标）时，改成嵌套 msg。

---

## ADR-006 `CostmapInterface` 用 `(x, y, z)` 查询点

**状态**：已采纳（2026-10）

**背景**：项目定位是 3D 导航，但眼下只有 2D 栅格；不想让规划器绑死在某一种地图表示上。

**决策**：抽象出 `getCost(x,y,z)` / `isOccupied` / `isInBounds` / `getDistanceToObstacle` /
`getTraversability` / `getGroundHeight`；2D 实现忽略 `z`，3D 实现（OctoMap/高程图/ESDF）将来替换。

**备选**：直接暴露 `OccupancyGrid` 指针（最快，但把规划器焊死在 2D）。

**后果**：
- 正：算法只依赖接口，2D→3D 是"换实现"不是"重写算法"
- 负：每点一次虚函数调用（大图慢）；2D 实现里 `z` 是谎言（靠文档说明）

**重审触发**：规划耗时出现明显热点时，加批量查询（`getCostBatch`）或缓存层。

---

## ADR-007 自建 `Nav2CostmapAdapter`，不用 `nav2_costmap_2d`

**状态**：已采纳（2026-10），**代价最大的一条**

**背景**：想自己实现"代价地图抽象"，同时复用地 `map_server` 的 `/map`。

**决策**：自建 adapter：订阅 `/map`，按 `occupied_threshold` 二值化，再按 `robot_radius`/`inflation_radius`/`cost_scaling_factor` 生成代价。

**备选**：直接用 `nav2_costmap_2d`（分层、滚动窗口、插件化 layer 一应俱全）。

**后果**：
- 正：实现 ~200 行可通读；CONTROL 在自己手里
- 负：**没有 obstacle/voxel 层、没有滚动窗口、没有动态障碍**——这是当前最大能力缺口
- 负：膨胀逻辑是简化版（无 distance transform 优化）

**重审触发**：需要动态避障时（下一步 P1）。届时有两条路：① 给 adapter 加 obstacle layer；② 直接切到 `nav2_costmap_2d` 并用它的 `Costmap2DROS` 适配 `CostmapInterface`（更省，但要接受它的生命周期）。

---

## ADR-008 单目标抢占模型（不做目标队列）

**状态**：已采纳（2026-10）

**背景**：一次只服务一个导航目标，任务层（多点导航）还没有。

**决策**：新 goal 直接以 `CANCELLED` 抢占旧 goal（旧 goal 返回 `error_code=4`，message 标注 `preempted`）。

**备选**：目标队列（FIFO）——会把"任务编排"的复杂度提前塞进导航层。

**后果**：
- 正：语义简单、时序清楚
- 负：任务层必须自己做排队和重试

**重审触发**：出现"批量点位导航"需求时，在任务层排队而不是改 NavServer。

---

## ADR-009 超时用 `steady_clock`，消息时间戳用节点时钟

**状态**：已采纳（2026-10）

**背景**：仿真里 `use_sim_time=true`，时钟可能暂停、跳变甚至重置（重启 Gazebo）。

**决策**：所有超时判断（规划超时、恢复时长、卡住检测、TF 超时）用 `std::chrono::steady_clock`；
只有消息 `header.stamp` 用 `node->now()`。

**备选**：统一用 ROS 时间——仿真重置时会出现"负超时"或"永不到期"。

**后果**：正：超时行为与仿真时钟解耦；负：日志里的时间和消息时间戳不是同一基准（已在此说明）。

**重审触发**：真机长期运行需要与系统时间对齐时，另加 wall time 日志字段。

---

## ADR-010 教学示例插件独立成包（`pnc_nav_examples`）

**状态**：已采纳（2026-10，commit `ed5280c`）

**背景**：教程需要一个"最小可运行插件"，但不想污染核心插件集合。

**决策**：新建 `pnc_nav_examples` 包，只放示例；README/教程引用它，不作为默认插件。

**备选**：塞进 `pnc_nav_planners`——会增加核心包的噪声，且示例的"故意很笨"会误导读者。

**后果**：正：示例可删可改、不影响生产；负：多一个包要构建（教程里单独说明）。

---

## ADR-011 恢复行为先做"旋转 + 重规划"

**状态**：已采纳（2026-10），**待扩展**

**背景**：原来 `RECOVERING` 只是打日志然后转 `FAILED`，`max_planning_retries` 形同虚设。

**决策**：进入恢复 → 原地旋转 `recovery_spin_duration`（1s）→ 重新规划；重试 `max_planning_retries` 次后以 `error_code=3` 结束。

**备选**：直接照搬 Nav2 的 Spin/BackUp/DriveOnHeading + 行为树编排——依赖任务层，太重。

**后果**：
- 正：有出口、可观测（`number_of_recoveries`）、可调参
- 负：只有旋转一种动作；不清代价地图（动态层加入后必须补）；旋转不检查碰撞（真机需加安全判断）

**重审触发**：加 obstacle layer 后，恢复序列改为"清局部图 → 后退 → 旋转 → 重规划"，并按失败原因分策略。

---

## ADR-012 复用 `nav2_map_server` + `nav2_lifecycle_manager`

**状态**：已采纳（2026-10）

**背景**：地图加载（`yaml + png`）和生命周期管理是纯基础设施，自己写没有学习价值。

**决策**：`pnc_nav_bringup` 的 launch 里直接起 Nav2 的 `map_server` 和 `lifecycle_manager`；`NavServer` 只订阅 `/map`。

**备选**：自己写 map loader——重复劳动，还容易在 `negate/occupied_thresh` 上踩坑。

**后果**：正：格式兼容、`bond` 检测现成、少写代码；负：多两个外部依赖（已在 README 依赖清单里写明）。

---

## 索引

| 编号 | 主题 | 状态 |
|---|---|---|
| ADR-001 | 插件化（pluginlib） | 已采纳 |
| ADR-002 | 单 NavServer | 已采纳（阶段性） |
| ADR-003 | 单线程执行器 | 已采纳 |
| ADR-004 | action 主入口 + 兼容 | 已采纳 |
| ADR-005 | error_code 设计 | 已采纳 |
| ADR-006 | CostmapInterface 3D 抽象 | 已采纳 |
| ADR-007 | 自建 costmap adapter | 已采纳（待补动态层） |
| ADR-008 | 单目标抢占 | 已采纳 |
| ADR-009 | steady_clock 计时 | 已采纳 |
| ADR-010 | 示例插件独立包 | 已采纳 |
| ADR-011 | 最小恢复行为 | 已采纳（待扩展） |
| ADR-012 | 复用 nav2_map_server | 已采纳 |

## 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版 12 条（commit `216e21d`） |
