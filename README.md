# PNC Nav2 — 三维导航规划与控制系统

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

面向轮足机器人的三维导航系统，核心聚焦**规划与控制 (Planning & Control)**，支持插件化算法热插拔。

本仓库为个人独立维护的学习/实践项目，后续可能开源。仅依赖官方 ROS 2 Jazzy、Gazebo Harmonic（`ros_gz`）与 Nav2 基础包（`map_server` / lifecycle_manager），不依赖任何第三方机器人产品 SDK、deb 或私有环境脚本。

## ✨ 特性

- **插件化架构**: 全局规划、局部规划、路径跟踪均可通过 pluginlib 热插拔
- **已实现算法**: A* 2D、RRT*、Pure Pursuit、Stanley、DWA（待完善）
- **真实地图集成**: 支持 `map_server` 发布的 OccupancyGrid（自带 `maps/111`）
- **渐进式开发**: 2D差速车验证 → 3D四足机器狗 → 真机部署
- **仿真栈**: Gazebo Sim Harmonic（playground + 简易差速车）+ Unitree Go2（3D，规划中）

## 🏗️ 系统架构

```
导航层 (NavServer)
  ├─ Global Planner (插件) → AStar2D / RRTStar
  ├─ Path Tracker (插件)   → PurePursuit / Stanley
  └─ Local Planner (插件)  → DWA (阶段2)
         │
运动层 (cmd_vel → 机器人控制)
         │
平台层 (Gazebo Sim / 真实硬件)
```

详细架构设计见 [ARCHITECTURE.md](ARCHITECTURE.md)

## 🚀 快速开始

### 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic（通过 `ros-jazzy-ros-gz`）
- `ros-jazzy-nav2-map-server`、`ros-jazzy-nav2-lifecycle-manager`
- C++17

### 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  python3-colcon-common-extensions \
  python3-rosdep

cd /path/to/pnc_nav2
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 构建

```bash
cd /path/to/pnc_nav2
source /opt/ros/jazzy/setup.bash

colcon build --packages-select \
  pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_sim pnc_nav_bringup \
  --parallel-workers 4

source install/setup.bash
```

若本机 `gz` 命令找不到，把 vendor 路径加入 PATH：

```bash
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:${PATH}"
```

### Docker（推荐：独立容器 `pnc_nav`）

项目专用容器已配置为挂载本仓库到相同路径 `/home/hao/pnc_nav2`，并在 `~/.bashrc` 中自动：

- `source /opt/ros/jazzy/setup.bash`
- 将 `gz` 加入 `PATH`
- 若已编译则 `source install/setup.bash`
- `cd` 到工作区

```bash
# 进入容器
docker exec -it pnc_nav bash

# 重建镜像/容器（在 docker/ 目录）
cd docker
docker build -t pnc_nav2:jazzy -f Dockerfile.jazzy .
docker rm -f pnc_nav
docker run -dit --name pnc_nav --restart unless-stopped --privileged --net=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/hao/pnc_nav2:/home/hao/pnc_nav2:rw \
  -v /dev:/dev -w /home/hao/pnc_nav2 \
  pnc_nav2:jazzy sleep infinity
```

### 运行 2D 导航测试（阶段1）

```bash
# 推荐入口：Gazebo Harmonic playground + maps/111 + NavServer + RViz
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py

# 或一键脚本
./start_nav.sh

# 兼容旧入口名（均转发到 gz_sim_2d_bringup）
ros2 launch pnc_nav_bringup sim_2d_bringup.launch.py
ros2 launch pnc_nav_bringup turtlebot3_test.launch.py
```

启动后：
1. 等待 Gazebo 加载 playground，`map_server` 自动激活
2. RViz 显示 `maps/111`（约 615×1334 @ 0.05 m）与简易差速车
3. 使用顶部工具栏的 **"2D Goal Pose"** 设置目标点
4. 观察路径规划与机器人移动（`/cmd_vel` → gz DiffDrive，`/odom` / TF 更新）

当前 Phase 1 主路径使用 `Nav2CostmapAdapter` 订阅 `map_server` 发布的 `/map`
(`nav_msgs/msg/OccupancyGrid`)。仿真由本仓库 `pnc_nav_sim` 提供（stock `gz-sim-diff-drive-system`）。

### 运行 3D 导航（Go2 机器狗，未来）

```bash
ros2 launch pnc_nav_bringup sim_3d_bringup.launch.py
```

## 📦 包结构

| 包名 | 说明 | 状态 |
|------|------|------|
| `pnc_nav_core` | NavServer + 插件基类 + 地图适配器 | ✅ 已完成 |
| `pnc_nav_planners` | 规划器插件（A*、RRT*、PP、Stanley、DWA） | ✅ 已实现 |
| `pnc_nav_sim` | Gazebo Harmonic playground + 简易差速车 + ros_gz 桥 | ✅ 已完成 |
| `pnc_nav_bringup` | 启动文件、`maps/111`、参数与 RViz | ✅ 已完成 |
| `pnc_nav_utils` | 评估工具（性能测试、地图编辑器） | ✅ 已完成 |
| `pnc_nav_interfaces` | 自定义消息定义 | ✅ 已完成 |

## 📋 开发路线

- [x] **Phase 1: 2D 闭环验证** ✅
  - [x] NavServer 框架
  - [x] A* 2D 全局规划
  - [x] Pure Pursuit 路径跟踪
  - [x] 集成 map_server（`maps/111`）
  - [x] Gazebo Sim playground 仿真测试

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

更多工具说明见 [src/pnc_nav_utils/evaluation/README.md](src/pnc_nav_utils/evaluation/README.md)。

## 📄 License

MIT License
