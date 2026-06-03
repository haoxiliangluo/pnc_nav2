# PNC Nav2 — 三维导航规划与控制系统 架构设计

## 项目定位

面向规控岗面试的开源项目，核心展示：**三维导航中的规划与控制能力**。
支持仿真验证 → 真实机器狗部署的完整链路，规划和控制模块插件化可替换。

---

## 一、系统总体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Mission Layer (任务层)                            │
│   Goal Manager / Behavior Tree / State Machine                          │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │ goal pose / task command
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Navigation Layer (导航层)                            │
│  ┌──────────────┐  ┌──────────────────┐  ┌──────────────────────────┐  │
│  │ Localization │  │ Global Planner   │  │ Local Planner            │  │
│  │ (定位模块)    │  │ (全局规划-插件)   │  │ (局部规划/避障-插件)      │  │
│  └──────┬───────┘  └────────┬─────────┘  └────────────┬─────────────┘  │
│         │                   │                          │                │
│         ▼                   ▼                          ▼                │
│  ┌──────────────┐  ┌──────────────────┐  ┌──────────────────────────┐  │
│  │ Map Server   │  │ Costmap / Trav.  │  │ Path Tracker             │  │
│  │ (地图服务)    │  │ (代价地图/可通行) │  │ (路径跟踪控制器-插件)     │  │
│  └──────────────┘  └──────────────────┘  └────────────┬─────────────┘  │
└────────────────────────────────────────────────────────┼────────────────┘
                                                         │ cmd_vel / twist
                                                         ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Motion Layer (运动层)                                │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ Locomotion Controller (步态控制器-插件)                            │   │
│  │   - RL Policy (强化学习策略)                                      │   │
│  │   - MPC Controller                                               │   │
│  │   - CPG-based Controller                                         │   │
│  └──────────────────────────────────┬───────────────────────────────┘   │
│                                     │ joint_commands / motor_commands    │
└─────────────────────────────────────┼───────────────────────────────────┘
                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Platform Layer (平台层)                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────────────┐  │
│  │ Gazebo Sim      │  │ Isaac Sim       │  │ Real Robot (Unitree/   │  │
│  │ (仿真环境)      │  │ (高保真仿真)     │  │  自研机器狗)            │  │
│  └─────────────────┘  └─────────────────┘  └────────────────────────┘  │
│                                                                         │
│  Hardware Abstraction Layer (HAL) — 统一传感器/执行器接口                  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 二、模块详细设计

### 2.1 插件化设计原则

所有规划和控制模块通过 **ROS 2 Plugin (pluginlib)** 机制实现热插拔：

```cpp
// 全局规划器基类
class GlobalPlannerBase {
public:
  virtual void configure(const rclcpp::Node::SharedPtr& node, 
                         const std::string& name,
                         const std::shared_ptr<CostmapInterface>& costmap) = 0;
  virtual nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                          const geometry_msgs::msg::PoseStamped& goal) = 0;
  virtual void cleanup() = 0;
  virtual ~GlobalPlannerBase() = default;
};

// 局部规划器基类
class LocalPlannerBase {
public:
  virtual void configure(...) = 0;
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommand(
      const geometry_msgs::msg::PoseStamped& current_pose,
      const geometry_msgs::msg::Twist& current_vel) = 0;
  virtual bool isGoalReached(...) = 0;
  virtual void cleanup() = 0;
};

// 步态控制器基类
class LocomotionControllerBase {
public:
  virtual void configure(const rclcpp::Node::SharedPtr& node) = 0;
  virtual JointCommands computeJointCommands(
      const geometry_msgs::msg::Twist& cmd_vel,
      const RobotState& current_state) = 0;
  virtual void cleanup() = 0;
};
```

### 2.2 各模块说明

| 模块 | 职责 | 默认实现 | 可替换为 |
|------|------|----------|----------|
| **Localization** | 3D定位 | Fast-LIO2 / LIO-SAM | SLAM Toolbox, ORB-SLAM3 |
| **Map Server** | 地图存储与服务 | OctoMap + 点云地图 | Voxblox, TSDF |
| **Costmap/Traversability** | 可通行性分析 | 2.5D高程图 + 距离变换 | 学习型可通行性 |
| **Global Planner** | 全局路径规划 | A* on OctoMap | PCT Planner, RRT*, PRM |
| **Local Planner** | 局部避障规划 | DWA 3D / TEB | MPC, ego-planner, DRL |
| **Path Tracker** | 路径跟踪 | Pure Pursuit 3D | Stanley, MPC Tracker |
| **Locomotion** | 步态控制 | 预训练RL策略 | MPC, CPG, 模仿学习 |

### 2.3 地图表示与转换

```
传感器数据 (LiDAR/Depth)
        │
        ▼
┌─────────────────┐
│ SLAM (建图)      │ ──▶ 点云地图 (.pcd)
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────────────┐
│           Map Converter (地图转换)            │
│                                             │
│  点云 ──▶ OctoMap (3D占据栅格)               │
│       ──▶ 2.5D高程图 (Elevation Map)         │
│       ──▶ ESDF (欧氏距离场)                  │
│       ──▶ 可通行性图 (Traversability Map)     │
│       ──▶ 2D投影栅格 (兼容Nav2)              │
└─────────────────────────────────────────────┘
```

---

## 三、ROS 2 包结构

```
pnc_nav2/                            # 顶层工作空间
├── README.md
├── ARCHITECTURE.md                  # 本文件
├── docker/                          # Docker开发环境
│   ├── Dockerfile.humble
│   └── docker-compose.yml
│
├── src/
│   ├── pnc_nav_bringup/            # 启动与配置
│   │   ├── launch/
│   │   │   ├── sim_2d_bringup.launch.py      # 2D仿真验证
│   │   │   ├── sim_3d_bringup.launch.py      # 3D仿真完整导航
│   │   │   └── real_robot_bringup.launch.py  # 真实机器人
│   │   ├── config/
│   │   │   ├── nav_params.yaml               # 导航参数
│   │   │   ├── planner_plugins.yaml          # 规划器插件配置
│   │   │   └── robot_params.yaml             # 机器人参数
│   │   └── rviz/
│   │
│   ├── pnc_nav_interfaces/         # 自定义消息/服务/动作
│   │   ├── msg/
│   │   │   ├── TraversabilityMap.msg
│   │   │   ├── RobotState.msg
│   │   │   └── JointCommands.msg
│   │   ├── srv/
│   │   │   ├── SetMap.srv
│   │   │   └── SwitchPlanner.srv
│   │   └── action/
│   │       └── NavigateTo3DPose.action
│   │
│   ├── pnc_nav_core/               # 核心框架 — 插件基类 + 导航服务器
│   │   ├── include/pnc_nav_core/
│   │   │   ├── global_planner_base.hpp
│   │   │   ├── local_planner_base.hpp
│   │   │   ├── path_tracker_base.hpp
│   │   │   ├── controller_base.hpp
│   │   │   ├── costmap_interface.hpp
│   │   │   └── nav_server.hpp
│   │   └── src/
│   │       └── nav_server.cpp        # 导航状态机/行为树
│   │
│   ├── pnc_nav_map/                # 地图管理
│   │   ├── map_server/              # OctoMap服务、地图加载/保存
│   │   ├── map_converter/           # 点云→OctoMap→高程图→2D栅格
│   │   └── traversability/          # 可通行性分析
│   │
│   ├── pnc_nav_localization/       # 定位模块
│   │   ├── lidar_odom/              # LiDAR里程计 (Fast-LIO2接口)
│   │   └── pose_estimator/          # 位姿融合
│   │
│   ├── pnc_nav_planners/           # 规划器插件集合 ★核心展示★
│   │   ├── global_planners/
│   │   │   ├── astar_3d/            # A* on OctoMap
│   │   │   ├── rrt_star_3d/         # RRT* 三维
│   │   │   ├── pct_planner/         # PCT点云断层规划 (进阶)
│   │   │   └── nav2_adapter/        # 桥接Nav2 GlobalPlanner (fallback)
│   │   ├── local_planners/
│   │   │   ├── dwa_3d/              # 3D DWA
│   │   │   ├── teb_3d/              # TEB时间弹性带
│   │   │   ├── mpc_local/           # MPC局部规划
│   │   │   └── drl_local/           # 深度强化学习局部规划 (进阶)
│   │   └── path_trackers/
│   │       ├── pure_pursuit_3d/     # 3D Pure Pursuit
│   │       ├── stanley_3d/          # Stanley控制器
│   │       └── mpc_tracker/         # MPC路径跟踪
│   │
│   ├── pnc_nav_control/            # 运动控制 ★核心展示★
│   │   ├── locomotion_interface/    # 步态控制器接口
│   │   ├── rl_locomotion/           # RL步态策略 (libtorch推理)
│   │   ├── mpc_locomotion/          # MPC步态控制
│   │   └── vel_smoother/            # 速度平滑
│   │
│   ├── pnc_nav_sim/                # 仿真环境（经典 Gazebo）
│   │   ├── worlds/                  # Gazebo 世界文件
│   │   ├── urdf/diff_drive/         # 差速小车（自有）
│   │   ├── urdf/unitree_go2/        # Go2（复用 go2_description 网格 + planar_move）
│   │   └── urdf/sensors/            # 通用传感器宏（Mid360 / IMU）
│   │
│   └── pnc_nav_utils/              # 工具库
│       ├── visualization/           # RViz可视化工具
│       ├── evaluation/              # 性能评估 (路径长度、平滑度、耗时)
│       └── bag_tools/               # rosbag录制与回放工具
│
├── third_party/                     # 第三方依赖
│   ├── octomap/
│   ├── grid_map/
│   └── unitree_ros2/
│
└── docs/                            # 文档
    ├── design/                      # 设计文档
    ├── tutorials/                   # 使用教程
    └── api/                         # API文档
```

---

## 四、开发路线图 (Roadmap)

### Phase 1: 2D仿真验证 (基础架构)
**目标**: 验证整体框架、接口设计、插件机制

- [ ] 搭建ROS 2 Humble工作空间
- [ ] 实现 `pnc_nav_core` 插件框架
- [ ] 2D Gazebo环境 + 差速小车模型
- [ ] 实现 A* 全局规划 (2D)
- [ ] 实现 DWA 局部规划 (2D)
- [ ] 实现 Pure Pursuit 路径跟踪
- [ ] Nav2 adapter 作为对比基线
- [ ] 基本的 evaluation 指标输出

### Phase 2: 3D定位与建图
**目标**: 完成三维感知链路

- [ ] 集成 Fast-LIO2 / LIO-SAM
- [ ] 点云地图 → OctoMap 转换
- [ ] 2.5D高程图生成
- [ ] 可通行性分析 (坡度、粗糙度、台阶检测)
- [ ] ESDF距离场生成
- [ ] 地图服务 (保存/加载/切换)

### Phase 3: 3D规划与控制 ★核心★
**目标**: 展示规控能力，插件化多算法对比

- [ ] A* 3D on OctoMap
- [ ] RRT* 3D
- [ ] DWA 3D / TEB 3D
- [ ] MPC 局部规划
- [ ] MPC 路径跟踪
- [ ] 3D Pure Pursuit
- [ ] 性能对比评估框架 (benchmark)
- [ ] 可视化对比工具

### Phase 4: 机器狗仿真
**目标**: 四足机器人完整导航闭环

- [ ] Unitree Go2 / A1 Gazebo模型集成
- [ ] RL步态控制器 (libtorch推理)
- [ ] cmd_vel → 步态 → 关节指令 完整链路
- [ ] 复杂3D地形仿真 (楼梯、斜坡、障碍)
- [ ] 端到端导航演示

### Phase 5: 进阶与真机 (扩展)
**目标**: 强化学习训练 + 真机部署

- [ ] RL/IL 训练步态策略 (Isaac Gym)
- [ ] DRL 局部规划训练
- [ ] PCT Planner 集成
- [ ] Sim-to-Real 迁移
- [ ] 真实机器狗部署与调试

---

## 五、关键接口定义

### 5.1 Topic 接口

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/odom` | nav_msgs/Odometry | 定位→导航 | 机器人里程计 |
| `/map/octomap` | octomap_msgs/Octomap | 地图→规划 | 3D占据地图 |
| `/map/elevation` | grid_map_msgs/GridMap | 地图→规划 | 2.5D高程图 |
| `/map/traversability` | grid_map_msgs/GridMap | 地图→规划 | 可通行性图 |
| `/global_plan` | nav_msgs/Path | 全局规划→局部规划 | 全局路径 |
| `/local_plan` | nav_msgs/Path | 局部规划→可视化 | 局部轨迹 |
| `/cmd_vel` | geometry_msgs/Twist | 控制→运动 | 速度指令 |
| `/joint_commands` | pnc_nav_interfaces/JointCommands | 运动→平台 | 关节指令 |
| `/point_cloud` | sensor_msgs/PointCloud2 | 传感器→定位/地图 | 点云数据 |
| `/imu` | sensor_msgs/Imu | 传感器→定位 | IMU数据 |

### 5.2 Service 接口

| Service | 类型 | 说明 |
|---------|------|------|
| `/switch_global_planner` | SwitchPlanner | 运行时切换全局规划器 |
| `/switch_local_planner` | SwitchPlanner | 运行时切换局部规划器 |
| `/save_map` | SetMap | 保存当前地图 |
| `/load_map` | SetMap | 加载指定地图 |

### 5.3 Action 接口

| Action | 类型 | 说明 |
|--------|------|------|
| `/navigate_to_3d_pose` | NavigateTo3DPose | 导航到3D目标点 (含反馈) |

---

## 六、技术选型

| 维度 | 选择 | 理由 |
|------|------|------|
| ROS版本 | ROS 2 Humble | LTS，生态成熟，Nav2兼容 |
| 语言 | C++ (核心) + Python (工具/训练) | 性能 + 灵活性 |
| 构建 | colcon + CMake | ROS 2标准 |
| 3D地图 | OctoMap + Grid Map | 成熟、轻量、Nav2生态兼容 |
| 仿真 | Gazebo Classic → Ignition | 社区支持好 |
| RL框架 | Isaac Gym (训练) + libtorch (部署) | 高效并行训练 |
| CI/CD | GitHub Actions | 自动构建+测试 |
| 容器化 | Docker | 环境一致性 |

---

## 七、面试亮点设计

1. **插件化架构**: 展示软件工程能力，pluginlib实现算法热插拔
2. **多算法对比**: 同一场景下A*/RRT*/MPC/DRL的benchmark，展示对算法的理解深度
3. **分层解耦**: 规划层与控制层完全解耦，展示系统设计能力
4. **Sim-to-Real**: 仿真→真机的完整链路，展示工程落地能力
5. **评估体系**: 量化指标 (路径长度、平滑度、计算耗时、成功率)，展示科学思维
6. **渐进式开发**: 2D验证→3D扩展→真机部署，展示项目管理能力

---

## 八、与Nav2的关系

```
本项目 ≠ 重新造轮子

策略：
- 定位/建图: 使用成熟方案 (Fast-LIO2, OctoMap)，留接口不深入
- 全局规划: 自研3D规划器 ★重点★，同时提供Nav2 adapter对比
- 局部规划: 自研多种3D局部规划器 ★重点★
- 路径跟踪: 自研多种控制器 ★重点★
- 步态控制: RL策略推理 ★亮点★
- 行为管理: 可复用Nav2的BT框架
- 恢复行为: 可复用Nav2的recovery机制
```
