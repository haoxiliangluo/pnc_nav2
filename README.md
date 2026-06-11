# PNC Nav2 — 三维导航规划与控制系统

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

面向轮足机器人的三维导航系统，核心聚焦**规划与控制 (Planning & Control)**，支持插件化算法热插拔。

## ✨ 特性

- **插件化架构**: 全局规划、局部规划、路径跟踪均可通过 pluginlib 热插拔
- **已实现算法**: A* 2D、RRT*、Pure Pursuit、Stanley、DWA（待完善）
- **真实地图集成**: 支持 map_server 发布的 OccupancyGrid
- **渐进式开发**: 2D差速车验证 → 3D四足机器狗 → 真机部署
- **完整仿真栈**: TurtleBot3 (2D) + Unitree Go2 (3D) + LIO-SAM

## 🏗️ 系统架构

```
导航层 (NavServer)
  ├─ Global Planner (插件) → AStar2D / RRTStar
  ├─ Path Tracker (插件)   → PurePursuit / Stanley
  └─ Local Planner (插件)  → DWA (阶段2)
         │
运动层 (cmd_vel → 机器人控制)
         │
平台层 (Gazebo / 真实硬件)
```

详细架构设计见 [ARCHITECTURE.md](ARCHITECTURE.md)

## 🚀 快速开始

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- C++17

### 构建

```bash
# 克隆仓库
git clone <your-repo-url>
cd pnc_nav2

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install

# 环境设置
source install/setup.bash
```

### 运行 2D 导航测试（阶段1已完成）

```bash
# 一键启动（包含 Gazebo + TurtleBot3 + NavServer + RViz）
./start_nav.sh
```

启动后：
1. 等待10秒，系统自动激活 map_server
2. RViz 中会显示灰色静态地图和机器人
3. 使用顶部工具栏的 **"2D Goal Pose"** 设置目标点
4. 观察红色路径规划和机器人移动

### 运行 3D 导航（Go2 机器狗，未来）

```bash
ros2 launch pnc_nav_bringup sim_3d_bringup.launch.py
```

## 📦 包结构

| 包名 | 说明 | 状态 |
|------|------|------|
| `pnc_nav_core` | NavServer + 插件基类 + 地图适配器 | ✅ 已完成 |
| `pnc_nav_planners` | 规划器插件（A*、RRT*、PP、Stanley、DWA） | ✅ 已实现 |
| `pnc_nav_bringup` | 启动文件与参数配置 | ✅ 已完成 |
| `pnc_nav_utils` | 评估工具（性能测试、地图编辑器） | ✅ 已完成 |
| `pnc_nav_interfaces` | 自定义消息定义 | ✅ 已完成 |

### third_party 集成

- **turtlebot3** - 2D测试平台（含地图）
- **simdog_go2** - Go2完整仿真栈（LIO-SAM + Velodyne + 四足控制）

## 📋 开发路线

- [x] **Phase 1: 2D 闭环验证** ✅
  - [x] NavServer 框架
  - [x] A* 2D 全局规划
  - [x] Pure Pursuit 路径跟踪
  - [x] 集成 map_server（真实地图）
  - [x] TurtleBot3 仿真测试
  
- [ ] **Phase 2: 局部规划与避障**
  - [ ] DWA 动态窗口法
  - [ ] Costmap 膨胀层
  - [ ] 路径平滑
  
- [ ] **Phase 3: 3D 定位与建图**
  - [ ] AMCL 定位
  - [ ] LIO-SAM 集成
  - [ ] OctoMap 3D 地图

- [ ] **Phase 4: Go2 机器狗导航**
  - [ ] 3D 路径规划
  - [ ] 地形适应性控制

详见 [NAV2_LEARNING_AND_IMPLEMENTATION_ROADMAP.md](NAV2_LEARNING_AND_IMPLEMENTATION_ROADMAP.md)

## 🔌 插件扩展示例

实现自定义规划器：

```cpp
#include "pnc_nav_core/global_planner_base.hpp"

class MyPlanner : public pnc_nav_core::GlobalPlannerBase {
public:
  void configure(const rclcpp::Node::SharedPtr & node, const std::string & name) override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  void cleanup() override;
};

PLUGINLIB_EXPORT_CLASS(MyPlanner, pnc_nav_core::GlobalPlannerBase)
```

在 `plugins.xml` 中注册，在 `nav_params.yaml` 中配置即可热插拔。

## 🧪 测试与评估

```bash
# 导航性能评估
python3 src/pnc_nav_utils/evaluation/nav_evaluator.py

# 规划器对比测试
python3 src/pnc_nav_utils/evaluation/planner_tester.py
```

## 📄 License

MIT License
