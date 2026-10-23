# 坐标系与话题（Frames & Topics）

> 最后核对：commit `216e21d`（2026-10-23）
> 用途：排错第一入口（"这个数据从哪来/到哪去"）、写插件时确认坐标系与单位

---

## 1. TF 树

```
map ──(静态/AMCL)──▶ odom ──(gz DiffDrive, 50Hz)──▶ base_link ──(固定关节)──▶ laser / velodyne
 │                        │                              │
 │                        │                              └── left_wheel / right_wheel / support_wheel
 │                        └── /odom (nav_msgs/Odometry)
 └── /map (nav_msgs/OccupancyGrid, map_server)
```

| 变换 | 谁发布 | 频率 | 说明 |
|---|---|---|---|
| `map → odom` | 阶段 1：`static_transform_publisher`（launch 里，恒等变换）<br>定位模式：AMCL / Cartographer | 静态 / 按定位更新 | **阶段 1 没有定位**，所以是假的恒等变换 |
| `odom → base_link` | Gazebo `gz::sim::systems::DiffDrive`（经 `ros_gz_bridge` 的 `/tf`） | 50Hz | 里程计积分，会漂 |
| `base_link → laser` | `robot_state_publisher`（固定关节，`/tf_static`） | 静态 | 2D 车（`diff_drive.urdf.xacro`） |
| `base_link → velodyne` | `robot_state_publisher` | 静态 | 3D 激光车（`diff_drive_3d.urdf.xacro`） |

**NavServer 只查一个变换**：`lookupTransform(global_frame, robot_frame, TimePointZero)`，
默认 `map → base_link`（参数 `global_frame` / `robot_frame`）。

> 真机/多机时把 `odom` 换成 `base_footprint` 之类都行，只要改这两个参数。

---

## 2. 话题清单

### 2.1 传感器 / 仿真（由 Gazebo + bridge 提供）

| 话题 | 类型 | 方向 | 频率 | 发布者 | 说明 |
|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ→ROS | ~50–100Hz | `ros_gz_bridge` | 仿真时钟（`use_sim_time=true` 时必须） |
| `/odom` | `nav_msgs/msg/Odometry` | GZ→ROS | 50Hz | DiffDrive + bridge | 位姿与速度（NavServer 只用 `twist`） |
| `/tf` | `tf2_msgs/msg/TFMessage` | GZ→ROS | 50Hz | DiffDrive + bridge | `odom → base_link` |
| `/joint_states` | `sensor_msgs/msg/JointState` | GZ→ROS | ~50Hz | bridge | 供 `robot_state_publisher` |
| `/nav_scan` | `sensor_msgs/msg/LaserScan` | GZ→ROS | ~10Hz | 2D 车 `gpu_lidar` | 局部代价地图的障碍来源（**240° FOV**，见 KI-014） |
| `/points` | `sensor_msgs/msg/PointCloud2` | GZ→ROS | — | 3D 车 `gpu_lidar`（`bridge_3d.yaml`） | 3D 激光点云 |

### 2.2 地图 / 导航（本项目核心）

| 话题 | 类型 | 方向 | 频率 | 发布者 | 订阅者 |
|---|---|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | → | 一次 + 变化 | `nav2_map_server` | `Nav2CostmapAdapter` |
| `/costmap` | `nav_msgs/msg/OccupancyGrid` | → | 2Hz（每 10 个控制周期） | `nav_server` | RViz（**8m 滚动窗口**：静态+激光+膨胀） |
| `/obstacle_layer` | `nav_msgs/msg/OccupancyGrid` | → | 2Hz | `nav_server` | RViz（仅激光障碍，确认障碍层工作） |
| `/global_plan` | `nav_msgs/msg/Path` | → | 每次规划成功 | `nav_server` | RViz / 评估脚本 |
| `/local_plan` | `nav_msgs/msg/Path` | → | 跟随中 20Hz | `nav_server` | RViz |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS→GZ | 跟随中 20Hz；退出时一帧零速 | `nav_server` | DiffDrive（经 bridge） |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | ← | 事件 | RViz / 脚本 | `nav_server`（兼容入口） |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | ← | 事件 | RViz 面板 / AMCL 工具 | AMCL |

### 2.3 Action

| Action | 类型 | 说明 |
|---|---|---|
| `/navigate_to_3d_pose` | `pnc_nav_interfaces/action/NavigateTo3DPose` | **主入口**：goal（位姿 + planner_id + max_planning_time）、feedback、result（error_code） |

### 2.4 QoS 与深度（代码现状）

| 发布/订阅 | 队列深度 |
|---|---|
| `/global_plan`、`/local_plan`、`/cmd_vel` | 10 |
| `/costmap` | 1 |
| `/goal_pose` 订阅 | 10 |
| `/odom` 订阅 | 20 |

均为默认可靠性策略（未做 `transient_local`）。**含义**：后启动的观察者看不到历史消息；
调试时先起订阅者再起节点，或直接看 RViz 的实时显示。

---

## 3. 坐标与单位约定

| 项 | 约定 |
|---|---|
| 长度 / 角度 | 米 / 弧度 |
| 平面 | `x` 前、`y` 左、`z` 上（ROS REP-103） |
| 朝向 | 四元数；算 yaw 用 `tf2::getYaw()` |
| 世界坐标系 | `map`（`global_frame` 参数） |
| 本体坐标系 | `base_link`（`robot_frame` 参数） |
| 速度 | `Twist`：`linear.x` 前进、`angular.z` 逆时针 |
| 地图栅格 | 值 0–100 表示占用概率；`-1` = 未知（`nav_msgs/OccupancyGrid`） |

---

## 4. 数据流（一次导航）

```
外部(脚本/RViz) ──goal──▶ navigate_to_3d_pose
                                │
map_server ──/map──▶ Nav2CostmapAdapter ──costmap_──┐
                                │                    │
Gazebo ──/tf──▶ TF(map→base_link) ──pose──▶ NavServer│
                                                     ▼
                                     GlobalPlanner 插件 ──/global_plan──▶
                                                     │
                            LocalPlanner 或 PathTracker 插件
                                                     │
                                                /cmd_vel
                                                     ▼
                                     Gazebo DiffDrive（动起来）
                                                     │
                                                /odom + /tf
```

---

## 5. 排错清单（TF/话题相关）

| 现象 | 检查 | 结论 |
|---|---|---|
| `tf2_echo map base_link` 报 `frame does not exist` | `ros2 topic hz /tf`、`ros2 node list` | 仿真没起 / bridge 没起 / `ROS_DOMAIN_ID` 不一致 |
| 报 `two or more unconnected trees` | `ros2 topic info /tf_static` 的 publisher 数量 | 有两个 `map→odom` 发布者（双实例或新旧 launch 混跑） |
| 位姿在两个位置之间跳变 | `ps -ef \| grep pnc_nav2` | **有两个仿真实例**（最常见，见 `01_quickstart.md` §6） |
| `/odom` 有数据但 `map→base_link` 失败 | `ros2 run tf2_ros tf2_monitor map base_link` | `map→odom` 缺失（static publisher 没起 / AMCL 未激活） |
| `/map` 没数据 | `ros2 lifecycle get /map_server` | `map_server` 未激活（`lifecycle_manager` 日志） |
| `/cmd_vel` 有输出但车不动 | `ros2 topic info /cmd_vel`、`gz topic -l` | bridge 方向错 / `GZ_PARTITION` 不一致 |

---

## 6. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版（commit `216e21d`） |
| 2026-10-23 | `/costmap` 改为 8m 滚动窗口；新增 `/obstacle_layer`；`/nav_scan` 已接入障碍层 |
