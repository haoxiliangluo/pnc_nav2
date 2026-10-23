# 教程 02：写你的第一个规划器插件

> 最后核对：commit `ed5280c`（2026-10-23）
> 前置：已按 [01_quickstart.md](01_quickstart.md) 跑通仿真和 action
> 预计耗时：30–40 分钟
> 成品代码：`src/pnc_nav_examples/`（可以直接对照/修改）

---

## 1. 我们要写什么

一个**故意很笨**的全局规划器 `StraightLinePlanner`：不做任何搜索，直接把起点和终点连成直线、等距采样成路径。

选它当第一个插件，是因为它把"插件最少要实现什么"暴露得最干净——没有搜索算法分散注意力：

- 继承 `pnc_nav_core::GlobalPlannerBase`
- 实现 `configure` / `createPlan` / `cleanup` / `getName`
- 用 `PLUGINLIB_EXPORT_CLASS` 导出 + `plugins.xml` 注册
- 参数挂在 `global_planner.*` 下

写完后你会看到：**同一份 NavServer 代码不动，只是发目标时指定 `planner_id`，规划器就换成了你写的这个**。

---

## 2. 五个文件

```
src/pnc_nav_examples/
├── package.xml                                  # 依赖声明
├── CMakeLists.txt                               # 编译 + 插件注册
├── plugins.xml                                  # 声明"我实现了哪个基类"
├── include/pnc_nav_examples/straight_line_planner.hpp
└── src/straight_line_planner.cpp
```

前三个是"接线"，后两个是代码。

---

## 3. 头文件：只比基类多两个成员

```cpp
#include "pnc_nav_core/global_planner_base.hpp"

namespace pnc_nav_examples
{
class StraightLinePlanner : public pnc_nav_core::GlobalPlannerBase
{
public:
  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  void cleanup() override;
  std::string getName() const override {return "StraightLinePlanner";}

private:
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<pnc_nav_core::CostmapInterface> costmap_;
  double step_{0.1};         // 采样间距
  double max_length_{50.0};  // 超过就返回空路径（演示失败）
};
}
```

完整文件：[src/pnc_nav_examples/include/pnc_nav_examples/straight_line_planner.hpp](../../src/pnc_nav_examples/include/pnc_nav_examples/straight_line_planner.hpp)

---

## 4. 实现：三个要点

### 4.1 参数必须带实例名前缀

```cpp
void StraightLinePlanner::configure(node, name, costmap)
{
  node_ = node;
  name_ = name;          // NavServer 传进来的是 "global_planner"
  costmap_ = costmap;

  node_->declare_parameter(name_ + ".step", 0.1);
  node_->declare_parameter(name_ + ".max_length", 50.0);
  step_ = node_->get_parameter(name_ + ".step").as_double();
  max_length_ = node_->get_parameter(name_ + ".max_length").as_double();
}
```

于是 yaml 里就是：

```yaml
global_planner:
  step: 0.1
  max_length: 50.0
```

### 4.2 失败 = 返回空路径

```cpp
if (length > max_length_) {
  RCLCPP_WARN(node_->get_logger(), "goal too far, returning empty path");
  return path;                 // poses 为空 → NavServer 进入 RECOVERING
}
```

NavServer 不认识"异常"，只认三个约定：**空路径 / `setPath` 返回 false / 零速指令**。

### 4.3 输出路径的 header 要与入参一致

```cpp
path.header.stamp = node_->now();
path.header.frame_id = start.header.frame_id.empty() ? "map" : start.header.frame_id;
```

---

## 5. 注册：三处缺一不可

**① C++ 里导出**（文件末尾）：

```cpp
PLUGINLIB_EXPORT_CLASS(pnc_nav_examples::StraightLinePlanner,
                       pnc_nav_core::GlobalPlannerBase)
```

**② `plugins.xml` 声明**：

```xml
<library path="straight_line_planner">
  <class type="pnc_nav_examples::StraightLinePlanner"
         base_class_type="pnc_nav_core::GlobalPlannerBase">
    <description>示例插件：直线路径</description>
  </class>
</library>
```

**③ CMake 注册到基类包**（最容易漏）：

```cmake
add_library(straight_line_planner SHARED src/straight_line_planner.cpp)
ament_target_dependencies(straight_line_planner rclcpp pluginlib nav_msgs geometry_msgs pnc_nav_core)
pluginlib_export_plugin_description_file(pnc_nav_core plugins.xml)
```

`pluginlib_export_plugin_description_file` 的第一个参数是**基类所在的包**（`pnc_nav_core`），
不是自己的包——写错的话插件永远不会被发现，而且不会报错，只会"类找不到"。

---

## 6. 构建并热插拔

```bash
cd ~/pnc_nav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select pnc_nav_core pnc_nav_examples
source install/setup.bash          # 让新包进入环境
```

起仿真（同教程 01）：

```bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export ROS_DOMAIN_ID=43 GZ_PARTITION=pnc_nav
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py use_gui:=false use_rviz:=false
```

**发目标时用 `planner_id` 指定你的插件**：

```bash
ros2 action send_goal /navigate_to_3d_pose \
  pnc_nav_interfaces/action/NavigateTo3DPose \
  "{goal: {header: {frame_id: map},
           pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
    planner_id: 'pnc_nav_examples::StraightLinePlanner',
    max_planning_time: 0.0}" --feedback
```

**预期日志**（实测）：

```
Accepted goal request: (1.50, 0.00, 0.00) planner_id='pnc_nav_examples::StraightLinePlanner'
Switched global planner to: pnc_nav_examples::StraightLinePlanner
[StraightLinePlanner] configured: step=0.100 m max_length=50.0 m
Start goal: (1.50, 0.00) frame=map planner=pnc_nav_examples::StraightLinePlanner
[StraightLinePlanner] straight-line path: 1.50 m, 16 poses
State transition: planning -> following
State transition: following -> succeeded
```

**预期结果**（实测）：

```
success = true
error_code = 0
total_time = 9.37 s
distance_traveled = 1.43 m
message = goal reached
```

注意：这一次路径**不是 A\* 算的**，是你写的直线；NavServer 一行都没改。

---

## 7. 三个练习（加深理解）

**练习 1：让插件"失败"**

把 `global_planner.max_length` 改成 `0.5`，再发一个 1.5m 的目标。预期：

```
[StraightLinePlanner] goal too far (1.50 m > 0.5 m), returning empty path
Entering recovery (planning failed), attempt 1/3
...
Goal finished: success=false error_code=3(MAX_RETRIES) ...
```

**练习 2：让机器人"撞墙"**

把目标设在障碍后面（例如地图上有货架的地方），直线会穿过障碍。
观察：`PathTracker` 只跟踪不避障 → 车会顶上去；这就是 `LocalPlannerBase`（DWA）存在的理由。

**练习 3：加一个参数**

给插件加 `global_planner.keep_final_heading`（bool）：为 true 时最后一个路径点的朝向保持起点朝向。
体会"插件参数只改 yaml、不动 NavServer"这条设计约束。

---

## 8. 检查点

- [ ] `ros2 action send_goal ... planner_id: 'pnc_nav_examples::StraightLinePlanner'` 能返回 `success=true`
- [ ] 日志出现 `Switched global planner to: pnc_nav_examples::StraightLinePlanner`
- [ ] `/global_plan` 里是一条直线（RViz 打开 Path 显示可见）
- [ ] 把 `max_length` 改小后能看到 `RECOVERING → MAX_RETRIES`
- [ ] 换回默认：不带 `planner_id` 只是发目标，插件仍是 `AStar2D`

---

## 9. 踩坑提醒

| 现象 | 原因 |
|---|---|
| `Failed to switch global planner: ...` | `plugins.xml` 的 `type` 写错，或 CMake 里 `pluginlib_export_plugin_description_file` 第一个参数写成了自己的包 |
| 插件加载后进程直接退出（`exit code -11`） | `cleanup()` 里先 `node_.reset()` 再打日志（我们真的踩过，见 `../design/plugin_contract.md` §2.6） |
| yaml 里改了参数没生效 | 参数只在 `configure()` 读一次，改完要重启节点 |
| 插件之间参数串味 | 参数没加 `<name>.` 前缀 |

下一课：`tutorials/03_how_a_cycle_runs.md` —— 一次控制循环里到底发生了什么（🚧 编写中）。
