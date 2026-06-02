# PNC Nav2 — 三维导航规划与控制系统

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

面向四足机器人的三维导航系统，核心聚焦**规划与控制 (Planning & Control)**，支持插件化算法热插拔。

## 特性

- **插件化架构**: 全局规划、局部规划、路径跟踪、步态控制均可通过 pluginlib 热插拔
- **多算法对比**: 内置 A\*、RRT\*、DWA、TEB、MPC、DRL 等多种算法实现
- **三维导航**: 基于 OctoMap 的三维路径规划，支持多层结构环境
- **仿真到真机**: Gazebo 仿真验证 → 真实机器狗部署的完整链路
- **渐进式开发**: 2D 小车验证 → 3D 机器狗仿真 → 真机部署

## 系统架构

```
任务层 (Goal Manager / BT)
        │
导航层 (Global Planner → Local Planner → Path Tracker)  ← 插件化
        │
运动层 (Locomotion Controller)  ← 插件化
        │
平台层 (Gazebo / Isaac Sim / Real Robot)
```

详细架构设计见 [ARCHITECTURE.md](ARCHITECTURE.md)

## 快速开始

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- C++17, Python 3.10+

### 构建

```bash
# 克隆仓库
git clone https://github.com/<your-username>/pnc_nav2.git
cd pnc_nav2

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build --symlink-install

# 环境设置
source install/setup.bash
```

### 运行 2D 仿真验证

```bash
ros2 launch pnc_nav_bringup sim_2d_bringup.launch.py
```

### 运行 3D 导航

```bash
ros2 launch pnc_nav_bringup sim_3d_bringup.launch.py
```

## 包结构

| 包名 | 说明 |
|------|------|
| `pnc_nav_core` | 核心框架，插件基类，导航服务器 |
| `pnc_nav_interfaces` | 自定义消息、服务、动作定义 |
| `pnc_nav_map` | 地图管理、转换、可通行性分析 |
| `pnc_nav_localization` | 定位模块接口 |
| `pnc_nav_planners` | 规划器插件集合 (全局/局部/跟踪) |
| `pnc_nav_control` | 运动控制 (RL/MPC步态) |
| `pnc_nav_simulation` | 仿真环境与模型 |
| `pnc_nav_bringup` | 启动文件与参数配置 |
| `pnc_nav_utils` | 可视化、评估、工具 |

## 开发路线

- [x] Phase 1: 2D 仿真验证 (框架 + 接口)
- [ ] Phase 2: 3D 定位与建图
- [ ] Phase 3: 3D 规划与控制 (核心)
- [ ] Phase 4: 机器狗仿真
- [ ] Phase 5: 真机部署

## 插件扩展

实现自定义规划器只需继承基类并注册插件：

```cpp
#include "pnc_nav_core/global_planner_base.hpp"

class MyPlanner : public pnc_nav_core::GlobalPlannerBase {
public:
  void configure(...) override;
  nav_msgs::msg::Path createPlan(...) override;
  void cleanup() override;
};

PLUGINLIB_EXPORT_CLASS(MyPlanner, pnc_nav_core::GlobalPlannerBase)
```

## License

MIT License
