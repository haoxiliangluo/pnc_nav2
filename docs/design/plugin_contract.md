# 插件契约（Plugin Contract）

> 最后核对：commit `ed5280c`（2026-10-23）
> 读者：想写插件的人 / 想评审接口设计的人
> 代码：`src/pnc_nav_core/include/pnc_nav_core/*_base.hpp`、示例插件 `src/pnc_nav_examples/`

---

## 1. 四个基类

| 基类 | 职责 | 是否需要 costmap | NavServer 是否在用 |
|---|---|---|---|
| `GlobalPlannerBase` | 起点 → 目标，输出全局路径 | ✅ | ✅ `global_planner_plugin` |
| `PathTrackerBase` | 跟踪给定路径，输出 `cmd_vel`（**不避障**） | ❌ | ✅ `path_tracker_plugin` |
| `LocalPlannerBase` | 结合代价地图输出 `cmd_vel`（**带避障**） | ✅（拿到的是**局部滚动窗口**） | ✅ `local_planner_plugin`（与 tracker 二选一） |
| `LocomotionControllerBase` | `cmd_vel` → 关节指令（四足步态） | ❌ | ❌ 未接入（为 3D/Go2 预留） |

> `local_planner` 与 `path_tracker` 互斥：两者都配置时**优先 local_planner**（`loadPlugins()` 提前 return）。

---

## 2. 通用契约（所有插件都适用）

### 2.1 生命周期调用顺序

```
createSharedInstance()          // pluginlib 构造（构造函数里不要碰 ROS）
  → configure(node, name, costmap?)   // 只做参数读取/初始化，不发布话题
  → activate()                        // 可选钩子（默认空实现）
  → [createPlan / setPath / computeVelocityCommand 反复调用]
  → deactivate()                      // 可选钩子
  → cleanup()                         // 释放 node_ / costmap_ / 内部状态
```

由 `NavServer::loadPlugins()` / `switchGlobalPlanner()` / `switchLocalPlanner()` / `switchPathTracker()` 驱动。

### 2.2 线程模型

- NavServer 使用**单线程执行器**（`rclcpp::spin`），插件调用与控制循环串行
- 插件**不得**自己起线程、不得阻塞（单次 `createPlan()` 应在 100ms 量级）
- 插件**不得**在自己的 `configure` 里创建定时器去改内部状态（会破坏"单线程"假设）

> 如果将来改多线程执行器，这节必须重写，并重新审会话状态。

### 2.3 参数命名空间（硬约定）

`configure()` 的第二个参数是**实例名**，NavServer 固定传：

| 基类 | 传入的 name | 参数前缀 |
|---|---|---|
| `GlobalPlannerBase` | `global_planner` | `global_planner.*` |
| `LocalPlannerBase` | `local_planner` | `local_planner.*` |
| `PathTrackerBase` | `path_tracker` | `path_tracker.*` |

插件必须把自己的参数全部声明在 `<name>.*` 之下：

```cpp
node_->declare_parameter(name_ + ".step", 0.1);
step_ = node_->get_parameter(name_ + ".step").as_double();
```

反例：`pnc_nav_planners` 里的 `PurePursuit2D` 有个 `declare_parameter("debug", false)` 没有加前缀——
这类"漏网参数"会让同一算法以不同实例名加载时互相干扰，新插件不要模仿。

### 2.4 坐标系与单位

| 项 | 约定 |
|---|---|
| 长度 | 米；角度 | 弧度 |
| 位姿坐标系 | 入参 `PoseStamped.header.frame_id` 由 NavServer 填 `global_frame`（默认 `map`） |
| 输出路径 | `path.header.frame_id` 必须与入参一致，`header.stamp` 用 `node_->now()` |
| 朝向 | 四元数；需要 yaw 用 `tf2::getYaw()`（`#include <tf2/utils.h>`） |
| 速度 | `Twist`：`linear.x` 前进、`angular.z` 逆时针为正 |

### 2.5 失败与边界约定

| 场景 | 约定 |
|---|---|
| 规划无解 | `createPlan()` 返回**空路径**（`poses.empty()`），NavServer 进入 `RECOVERING` |
| 拒绝路径 | `setPath()` 返回 `false` → 会话以 `error_code=1` 失败 |
| 无法输出速度 | 返回全零 `TwistStamped`（= 停车），不要返回 NaN |
| 地图外/未知区域 | 通过 `CostmapInterface::isInBounds()` / `isOccupied()` 判断，不要自己猜网格越界行为 |
| 插件内部异常 | 不要 `throw` 穿过 NavServer（会打死进程）；日志 + 返回失败值 |

### 2.6 内存与所有权

- `node_` / `costmap_` 是 `shared_ptr`，插件**不拥有**它们，`cleanup()` 里 reset 即可
- ⚠️ **不要在 reset 之后继续使用**：

```cpp
// 反例（真实踩过，热插拔必崩，exit code -11）
void AStar2D::cleanup() {
  node_.reset();
  RCLCPP_INFO(node_->get_logger(), "cleaned up");   // ← 空指针解引用
}

// 正确
void AStar2D::cleanup() {
  if (node_) { RCLCPP_INFO(node_->get_logger(), "cleaned up"); }
  node_.reset();
  costmap_.reset();
}
```

### 2.7 日志约定

- `configure` / `cleanup` / 状态变化：`RCLCPP_INFO`
- 每周期都会走到的路径：`RCLCPP_DEBUG` 或 `RCLCPP_INFO_THROTTLE(..., 2000, ...)`
- 反例：`DWA2D` 每个控制周期打两行 INFO（"与目标距离差未达到" / "DWA2D: cmd=..."），20Hz 下会淹掉日志

---

## 3. 各基类详细契约

### 3.1 `GlobalPlannerBase`

```cpp
virtual void configure(node, name, costmap) = 0;
virtual void activate() {}
virtual void deactivate() {}
virtual void cleanup() = 0;
virtual nav_msgs::msg::Path createPlan(start, goal) = 0;
virtual std::string getName() const = 0;
```

**createPlan 语义**

- 入参已带 `frame_id`（默认 `map`），出参路径的 `frame_id` 必须与之一致
- 路径点顺序：`poses[0]` 应接近起点，`poses.back()` 接近目标
- 返回空路径 = 规划失败（会触发恢复行为）
- 不要求等距采样，但**相邻点间距不要过大**（跟踪器按最近点查表）

**已实现的参考实现**：`AStar2D`（含 B 样条平滑）、`AStar3D`、`RRTStar3D`

### 3.2 `PathTrackerBase`

```cpp
virtual void configure(node, name) = 0;      // 注意：没有 costmap
virtual bool setPath(path) = 0;
virtual TwistStamped computeVelocityCommand(current_pose, current_vel) = 0;
virtual bool isPathCompleted(current_pose, dist_tolerance) = 0;
virtual size_t getCurrentWaypointIndex() const = 0;
virtual double getCrossTrackError() const { return 0.0; }
virtual double getHeadingError() const { return 0.0; }
virtual std::string getName() const = 0;
```

- `setPath()` 需重置内部索引（新路径从头上开始跟踪）
- `isPathCompleted()` 决定**是否判定到达**——这里的选择会影响 feedback 里 `distance_remaining` 的一致性（见 `../design/nav_server.md` §8）
- 已实现：`PurePursuit2D`、`PurePursuit3D`、`Stanley3D`

### 3.3 `LocalPlannerBase`

```cpp
virtual void configure(node, name, costmap) = 0;
virtual bool setPath(path) = 0;
virtual TwistStamped computeVelocityCommand(current_pose, current_vel) = 0;
virtual bool isGoalReached(current_pose, goal, dist_tol, angle_tol) = 0;
virtual nav_msgs::msg::Path getLocalPlan() const { return {}; }   // 可视化用
virtual std::string getName() const = 0;
```

- 与 tracker 的区别：**必须**使用 `costmap_` 做碰撞/代价判断
- `getLocalPlan()` 返回的轨迹会被 NavServer 发到 `/local_plan`
- 已实现：`DWA2D`（可用但有已知问题）、`DWA3D`（未验证）

### 3.4 `LocomotionControllerBase`（未接入）

```cpp
virtual JointCommands computeJointCommands(cmd_vel, robot_state) = 0;
virtual double getControlFrequency() const = 0;     // 步态通常 200–1000Hz
virtual JointCommands emergencyStop(robot_state) = 0;
```

设计意图：`导航层 → cmd_vel → 运动层` 的边界。接入前 NavServer 不感知它。

---

## 4. 新增插件 Checklist（7 步）

1. **头文件**：`class Xxx : public pnc_nav_core::GlobalPlannerBase`，实现 4 个纯虚函数
2. **实现**：参数全部用 `name_ + ".*"`；失败返回空路径/`false`/零速，不 throw
3. **导出**：`PLUGINLIB_EXPORT_CLASS(pnc_nav_examples::Xxx, pnc_nav_core::GlobalPlannerBase)`
4. **plugins.xml**：`<class type="包名::类名" base_class_type="pnc_nav_core::GlobalPlannerBase">`
5. **CMakeLists**：
   ```cmake
   find_package(pnc_nav_core REQUIRED)
   add_library(xxx_planner SHARED src/xxx.cpp)
   ament_target_dependencies(xxx_planner rclcpp pluginlib nav_msgs geometry_msgs pnc_nav_core)
   pluginlib_export_plugin_description_file(pnc_nav_core plugins.xml)   # 注册到基类包
   ```
6. **package.xml**：依赖 `pnc_nav_core`（以及用到的消息包）
7. **参数**：在 `nav_params.yaml` 里加 `<实例名>.*`，**不要**改 NavServer 代码

验证：

```bash
colcon build --symlink-install --packages-select <你的包> pnc_nav_core
# 1) 参数方式（改 nav_params.yaml 的 xxx_plugin）
# 2) 运行时热插拔（action goal 里带 planner_id）
ros2 action send_goal /navigate_to_3d_pose pnc_nav_interfaces/action/NavigateTo3DPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0}, orientation: {w: 1.0}}},
    planner_id: '<你的包>::<你的类>', max_planning_time: 0.0}"
```

**预期日志**：

```
Switched global planner to: <你的包>::<你的类>
[<你的类>] configured: ...
[<你的类>] ... path: 1.50 m, 16 poses
```

---

## 5. 热插拔是怎么生效的

```
action goal.planner_id 非空
  → NavServer::startGoal()
  → switchGlobalPlanner(planner_id)
      ├ 与当前类名相同 → 直接返回（不重载）
      └ 不同 → 旧插件 deactivate() + cleanup() → pluginlib 构造新插件 + configure() + activate()
```

- 只支持**全局规划器**热插拔（`planner_id` 字段），局部规划器/跟踪器目前只能靠参数切换
- 切换失败（类名写错/库没装）→ 该 goal 以 `error_code=6` 失败，**旧插件已被清理**，需要重新发目标或重启

---

## 6. 已知坑（都真实发生过）

| 现象 | 原因 | 修复 |
|---|---|---|
| 带 `planner_id` 发目标，nav_server 直接退出（`exit code -11`） | 插件 `cleanup()` 先 `node_.reset()` 再打日志 | 日志在前，reset 在后；已修 `AStar2D` |
| 改了 yaml 参数没生效 | 插件把参数读在 `configure()` 里，只读一次 | 改参数后重启节点（无动态参数回调） |
| 两个不同插件互相干扰 | 参数没加 `<name>.` 前缀 | 补前缀（`PurePursuit2D` 的 `debug` 是遗留问题） |
| 日志刷屏 / 帧率下降 | 插件在每周期打 INFO | 改 DEBUG 或 THROTTLE |

---

## 7. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：通用契约、四个基类、checklist、热插拔机制、坑位（commit `ed5280c`） |
