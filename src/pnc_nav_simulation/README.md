# PNC Nav2 Simulation

本包提供仿真环境和机器人模型，支持三种平台：

## 机器人平台

| 平台 | 类型 | 用途 | 传感器 |
|------|------|------|--------|
| diff_drive | 差速小车 | Phase 1 2D验证 | Mid360 LiDAR + IMU |
| wpr | 启智机器人(轮式) | 2D/2.5D导航验证 | Mid360 LiDAR + IMU |
| unitree_go2 | 宇树Go2四足 | 3D导航 + 步态控制 | Mid360 LiDAR + IMU |

## 传感器

- **Livox Mid360**: 非重复扫描LiDAR，360°FOV，发布 PointCloud2
- **IMU**: 6轴IMU，发布 Imu 消息

## 仿真世界

| 世界 | 说明 |
|------|------|
| simple_maze.world | 简单2D迷宫，Phase 1验证 |
| office.world | 室内办公环境 |
| multi_floor.world | 多层楼梯/斜坡环境 |
| outdoor_terrain.world | 室外不平坦地形 |

## 第三方依赖

需要克隆到 `third_party/` 目录：

```bash
# Unitree Go2 描述文件
cd third_party
git clone https://github.com/unitreerobotics/unitree_ros2.git

# WPR 仿真
git clone https://github.com/6-robot/wpr_simulation2.git

# Livox Gazebo 仿真插件 (ROS 2)
git clone https://github.com/LihanChen2004/livox_laser_simulation_ros2.git
```

## 启动

```bash
# 差速小车 + 简单迷宫
ros2 launch pnc_nav_simulation diff_drive_sim.launch.py world:=simple_maze

# WPR + 办公环境
ros2 launch pnc_nav_simulation wpr_sim.launch.py world:=office

# Go2 + 多层环境
ros2 launch pnc_nav_simulation go2_sim.launch.py world:=multi_floor
```
