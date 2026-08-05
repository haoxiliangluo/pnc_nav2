# 阶段1验证清单

## ✅ 功能验证

- [x] 系统启动无报错
  ```bash
  ./start_nav.sh
  ```

- [x] RViz 显示完整
  - [x] 灰色静态地图可见
  - [x] 机器人模型可见
  - [x] TF树正常（map → odom → base_link）
  - [x] 工具栏显示 "2D Goal Pose" 按钮

- [x] 导航闭环验证
  - [x] 点击 "2D Goal Pose" 设置目标
  - [x] 红色全局路径生成（/global_plan）
  - [x] 机器人开始移动
  - [x] 到达目标后停止

- [x] 日志检查
  ```bash
  # NavServer 启动日志（maps/111）
  [nav_server]: NavServer initialized, control freq: 20.0 Hz
  [nav_server]: Received map: 615x1334, resolution=0.050
  [nav_server]: AStar2D activated
  [map_io]: Read map .../maps/111/111.png: 615 X 1334 map @ 0.05 m/cell
  ```

## 📊 已实现功能

### 核心框架
- [x] NavServer 状态机
- [x] 插件系统（pluginlib）
- [x] TF监听与位姿获取
- [x] Nav2CostmapAdapter（订阅 /map）

### 规划器
- [x] AStar2D - 2D栅格搜索
- [x] RRTStar3D - 快速随机树
- [x] PurePursuit3D - 路径跟踪
- [x] Stanley3D - 前轮转向控制
- [x] DWA3D - 动态窗口（框架，待完善）

### 仿真环境
- [x] Gazebo Sim Harmonic playground（`pnc_nav_sim`）
- [x] map_server 发布静态地图（Map/111）
- [x] RViz 可视化配置

### 工具
- [x] 性能评估脚本（nav_evaluator.py）
- [x] 规划器测试工具（planner_tester.py）
- [x] 地图编辑器（costmap_editor.py）

## ⚠️ 已知限制（不影响阶段1）

- [ ] A* 路径未平滑（直角转弯）
- [ ] Pure Pursuit 不避障（只跟踪全局路径）
- [ ] 无定位系统（使用静态 map→odom TF）
- [ ] DWA 未完整实现

## 🎯 阶段1目标达成

**核心验证通过**：2D 差速机器人能收到目标点、规划路径、跟踪路径并到达目标。

**代码质量**：
- 插件化架构清晰
- 代码结构符合 ROS2 规范
- 第三方依赖整理完毕

**可演示能力**：
- 一键启动完整系统
- RViz 交互式设置目标
- 路径规划可视化

---

**下一阶段**：实现 DWA 局部规划器，增加实时避障能力。




  double PurePursuit2D::computeCurvature(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point)
    {//有当前位子,前视点,前瞻距离,然后计算曲率
        double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;
        double lookahead_dist = std::hypot(dx, dy);
        if(lookahead_dist < 1e-6)
        {
            return 0.0;
        }
        // double path_yaw = std::atan2(dy, dx);
        // double robot_yaw = tf2::getYaw(current_pose.pose.orientation);
        // double delta = path_yaw - robot_yaw;
        // while (delta > M_PI) delta -= 2.0 * M_PI;
        // while (delta < -M_PI) delta += 2.0 * M_PI;
        double delta = atan2(dy, dx) - tf2::getYaw(current_pose.pose.orientation);
        heading_error_ = delta;// 航向误差
        // cross_track_error_ = std::sin(heading_error_) * lookahead_dist;// 横向误差 = 路径与机器人连线的距离 * sin(路径与机器人航向的夹角)
        // return 2.0 * cross_track_error_ / (lookahead_dist * lookahead_dist);
        return 2.0 * std::sin(heading_error_) / lookahead_dist; // 曲率 = 2 * 横向误差 / 前视距离^2
    }
