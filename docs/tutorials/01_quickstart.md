# 教程 01：30 分钟跑起来

> 最后核对：commit `4c0cad3`（2026-10-23），Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
> 目标：从零把仿真跑起来，用 action 发第一个导航目标，并确认"成功 / 取消 / 不可达"三种结果
> 预计耗时：30–40 分钟（含首次编译）

完成本教程后你会得到：Gazebo 里的差速小车在 `maps/111` 地图上按 `AStar2D + PurePursuit2D` 走到目标点，并能在终端看到连续的 feedback 与 `error_code`。

---

## 0. 前置检查

```bash
ls /opt/ros/jazzy/setup.bash          # 必须有
grep -q "24.04" /etc/os-release && echo "Ubuntu 24.04 ok"
```

缺少依赖时先装（ros-base 默认不带这些）：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-ros2launch ros-jazzy-ros2action ros-jazzy-ros2topic ros-jazzy-ros2node
```

> 为什么单列 `ros2launch/ros2action`：`ros-base` 安装里没有 `ros2 launch` 和 `ros2 action` 子命令，
> 症状分别是 `invalid choice: 'launch'` 和 `invalid choice: 'action'`。

---

## 1. 构建

```bash
cd ~/pnc_nav2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --parallel-workers 4 \
  --packages-select \
  pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_sim pnc_nav_bringup
```

**预期**：`Summary: 5 packages finished`。

> 为什么不 `colcon build` 全量：`third_party/cartographer` 需要 `abseil/protobuf/ceres`，
> 默认环境没有；`pnc_nav_rviz`（RViz 控制面板）依赖 `cartographer_ros_msgs`。
> 本教程不需要它们。确实要编 RViz 面板时见 `guides/runbook.md`。
> 做教程 02（写插件）时需要再构建示例包：`--packages-select pnc_nav_core pnc_nav_examples`。

---

## 2. 启动仿真 + 地图 + 导航服务器

```bash
cd ~/pnc_nav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH     # 让 gz 命令可用

# 关键：与机器上其它 ROS 栈隔离（本项目约定 43 / pnc_nav）
export ROS_DOMAIN_ID=43
export GZ_PARTITION=pnc_nav

# 有显示器：开 GUI；无显示器：都关掉
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py use_gui:=true use_rviz:=true
# 无显示器/服务器上：
# ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py use_gui:=false use_rviz:=false
```

> 想省事可以直接 `./start_nav.sh`：它已经设置好 `ROS_DOMAIN_ID=43`、`GZ_PARTITION=pnc_nav` 和
> gz 的 `PATH`。**但它会在前台常驻**，进阶操作建议手工起 launch。

---

## 3. 检查点 1：系统是否正常

另开一个终端（同样 `source install/setup.bash` 且 `export ROS_DOMAIN_ID=43`）：

```bash
ros2 node list
```
**预期**：至少包含 `/nav_server`、`/map_server`、`/lifecycle_manager_map`、`/ros_gz_bridge`、`/robot_state_publisher`。

```bash
ros2 action list | grep navigate
```
**预期**：`/navigate_to_3d_pose`

```bash
ros2 topic hz /odom        # 预期约 50 Hz
ros2 topic hz /clock       # 预期约 50–100 Hz
ros2 run tf2_ros tf2_echo map base_link
```
**预期**：`tf2_echo` 能打出 `Translation` / `Rotation`（机器人初始在 `(0, 0)`）。
若一直提示 `frame does not exist`，看 §6 的"TF 不连通"。

```bash
ros2 topic echo /map --once | head -8
```
**预期**：`resolution: 0.05`、`width: 615`、`height: 1334`（来自 `maps/111`）。

---

## 4. 检查点 2：发第一个目标

```bash
ros2 action send_goal /navigate_to_3d_pose \
  pnc_nav_interfaces/action/NavigateTo3DPose \
  "{goal: {header: {frame_id: map},
           pose: {position: {x: 2.0, y: 0.0, z: 0.0},
                  orientation: {w: 1.0}}},
    planner_id: '', max_planning_time: 0.0}" \
  --feedback
```

**预期输出**（节选）：

```
Result:
  success        = True
  error_code     = 0
  total_time     = ~20 s
  distance_traveled = ~1.9 m
  message        = goal reached

Feedback（每 ~50ms 一条）:
  current_state: following, distance_remaining: 1.32, number_of_recoveries: 0
```

同时 `nav_server` 日志里能看到状态迁移：

```
State transition: idle -> planning
State transition: planning -> following
Global path found with N waypoints
Goal finished: success=true error_code=0(OK) time=20.5s distance=1.93m msg=goal reached
State transition: following -> succeeded
```

> 看不到路径？在 RViz 里加 `Path` 显示，话题选 `/global_plan`；代价地图是 `/costmap`。

---

## 5. 检查点 3：取消 与 不可达

取消（发目标 2 秒后取消，期望 `error_code=4`）：

```bash
python3 test/action_smoke_test.py cancel
```

不可达（目标在地图外，期望 `error_code=3`，恢复 3 次后干净失败）：

```bash
python3 test/action_smoke_test.py unreachable
```

可达目标（不带交互，直接看结果）：

```bash
python3 test/action_smoke_test.py feedback
```

**预期**：

| 脚本参数 | error_code | message |
|---|---|---|
| `feedback` | 0 | `goal reached` |
| `cancel` | 4 | `goal cancelled by client` |
| `unreachable` | 3 | `recovery attempts exhausted after 4 tries (last: planning failed)` |

---

## 6. 常见问题（都是真踩过的）

| 现象 | 原因 | 修复 |
|---|---|---|
| `ros2: error: invalid choice: 'launch'` | ros-base 没装 `ros2launch` | `sudo apt install ros-jazzy-ros2launch` |
| `gz: command not found` | ROS 的 Gazebo 命令在 vendor 目录 | `export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH` |
| TF 提示 `two or more unconnected trees`、机器人位姿在两个位置跳变 | **同时跑了两个仿真实例**（上一个 launch 的 gz 没退干净） | `ps -ef \| grep pnc_nav2` 找全并 `kill`，确认只有一个 gz 再启动 |
| 起来了但 `/map` 没有数据 | `map_server` 没被 lifecycle 激活 | 看 `/lifecycle_manager_map` 日志；确认 `maps/111/111.yaml` 路径存在 |
| action 一直不返回 / `action list` 里没有 | `ROS_DOMAIN_ID` 不一致，或 `nav_server` 没起来 | 所有终端统一 `ROS_DOMAIN_ID=43`；`ros2 node list` 确认 `/nav_server` |
| 目标很近但一直失败、机器人原地转圈 | 目标落在障碍/膨胀层里 | 换一个空地上的目标点；或调小 `map_adapter.inflation_radius` |
| 换成 DWA2D 后停在目标前 ~0.25m | 已知问题（到达判定与膨胀层冲突） | 用 baseline `PurePursuit2D`；见 `notes/known_issues.md` |

---

## 7. 你刚刚跑起来的东西

| 组件 | 说明 |
|---|---|
| `pnc_nav_sim` | Gazebo Harmonic world + 差速车 URDF + `ros_gz_bridge`（`/odom` `/tf` `/nav_scan` `/clock`） |
| `nav2_map_server` + `nav2_lifecycle_manager` | 加载 `maps/111` 并发布 `/map` |
| `static_transform_publisher` | 阶段 1 用固定 `map→odom`（还没有跑定位） |
| `pnc_nav_core/nav_server` | 状态机 + 插件调度 + action 服务 |
| `pnc_nav_planners` | `AStar2D` 全局规划 + `PurePursuit2D` 跟踪 |

下一课：`tutorials/02_first_plugin.md` —— 自己写一个规划器插件并热插拔（🚧 编写中）。
在此之前建议先读 [../design/nav_server.md](../design/nav_server.md)，理解状态机与错误码。

---

## 8. 清理

```bash
# 前台 Ctrl-C，或
pkill -f "ros2 launch pnc_nav_bringup"
pkill -f "pnc_nav2/install"
```
