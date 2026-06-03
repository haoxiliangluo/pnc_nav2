# pnc_nav_sim

PNC Nav2 的**经典 Gazebo 仿真层**。本包只负责提供仿真环境与传感器数据，
机器人动力学/步态控制不在范围内（由运动层负责）。

> 注：本包原名 `pnc_nav_simulation`，已重命名为 `pnc_nav_sim`，并明确职责为
> "仿真环境 + 传感器"，不再承担机器人模型的权威来源。

## 为什么不直接用 third_party 的仿真包？

`third_party/ROS2-Gazebo-GO2` 使用的是**新版 Gazebo Sim**（`gz sim` +
`ros_gz_bridge` + `gz_ros2_control`），其世界文件与传感器插件和本仓库其余部分
所用的**经典 Gazebo 11**（`libgazebo_ros_*`）不兼容，且自带一整套会与
`pnc_nav_core` 冲突的 Nav2。因此本包不 include 其启动文件，而是：

- **复用** GO2 的视觉网格（`.dae` 与仿真器无关），让机器人外观逼真；
- **替换**其运动/控制栈为经典 Gazebo 的 `planar_move` 插件（Phase 1 把步态抽象掉）。

## 机器人平台

| 平台 | 类型 | 用途 | 模型来源 |
|------|------|------|----------|
| diff_drive | 差速小车 | Phase 1 2D 快速验证 | 本包自有 |
| unitree_go2 | 宇树 Go2 四足 | 3D 导航验证 | 复用 go2_description 网格 + planar_move |

> WPR 平台已移除：`third_party/wpr_simulation2` 是完整的经典 Gazebo 仿真，
> 直接用它即可，本包不再重复维护 WPR 模型。

## 传感器（通用 xacro 宏，可挂载到任意机器人）

- **Livox Mid360**：`urdf/sensors/livox_mid360.urdf.xacro`，发布 `PointCloud2`
- **IMU**：`urdf/sensors/imu.urdf.xacro`，发布 `Imu`

## 仿真世界

| 世界 | 说明 |
|------|------|
| `simple_maze.world` | 简单 2D 迷宫，Phase 1 验证 |
| `multi_floor.world` | 多层楼 + 斜坡 + 台阶，3D 导航验证 |

## 第三方依赖

`go2_classic` 模型复用 `go2_description`（来自 `third_party/ROS2-Gazebo-GO2`）
的视觉网格。该包会随工作空间一同被 colcon 构建，xacro 通过
`$(find go2_description)` 解析其网格路径。

## 启动

```bash
# 差速小车 + 简单迷宫（2D）
ros2 launch pnc_nav_sim diff_drive_sim.launch.py world:=simple_maze

# Go2 + 多层环境（3D）
ros2 launch pnc_nav_sim go2_sim.launch.py world:=multi_floor
```

启动后本包只拉起 Gazebo + 机器人 + 传感器。导航栈请单独启动，例如：

```bash
ros2 launch pnc_nav_bringup sim_3d_bringup.launch.py
```
