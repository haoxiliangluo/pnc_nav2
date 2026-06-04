# PNC Nav2 学习与实现路线

这份文档用于指导你自己实现 `pnc_nav2`，目标不是复刻官方 Nav2，而是学习它的架构思想，写出一个面向四足机器人、3D 导航、规划与控制展示的“小型 Nav2-like 系统”。

核心原则：

- 先跑通 2D 闭环，再扩展 3D。
- 先写清楚架构和接口，再追求算法复杂度。
- 官方 Nav2 用来学习工程组织，不直接大量搬代码。
- 面试重点放在“我为什么这么拆、每层负责什么、怎么从 2D 扩到 3D”。

---

## 一、官方 Nav2 去哪学

官方仓库位置：

```text
/home/hao/navigation2-humble
```

建议只看关键包，不要整仓库硬啃。

| 学习目标 | 官方 Nav2 目录 | 你自己的对应位置 | 要学什么 |
|---|---|---|---|
| 插件接口设计 | `nav2_core` | `src/pnc_nav_core/include/pnc_nav_core/*_base.hpp` | 如何定义 planner/controller 插件基类 |
| 全局规划服务器 | `nav2_planner` | `src/pnc_nav_core/src/nav_server.cpp` 的 `PLANNING` 状态 | 如何加载 planner plugin、调用 `createPlan()` |
| 控制服务器 | `nav2_controller` | `LocalPlannerBase`、`PathTrackerBase`、`FOLLOWING` 状态 | 如何接收路径、循环输出 `cmd_vel` |
| 行为树任务层 | `nav2_bt_navigator`、`nav2_behavior_tree` | 后续 Mission Layer | 如何组织导航任务、恢复行为、任务切换 |
| 代价地图 | `nav2_costmap_2d` | `CostmapInterface`、`Nav2CostmapAdapter`、后续 `pnc_nav_map` | costmap 查询、障碍层、膨胀层、坐标转换 |
| 启动配置 | `nav2_bringup` | `src/pnc_nav_bringup` | launch、params、RViz、仿真如何组合 |
| 算法插件写法 | `nav2_navfn_planner`、`nav2_smac_planner`、`nav2_regulated_pure_pursuit_controller`、`nav2_dwb_controller` | `src/pnc_nav_planners` | 参数、生命周期、pluginlib 注册、可视化 |

推荐阅读顺序：

1. `nav2_core/include/nav2_core/global_planner.hpp`
2. `nav2_core/include/nav2_core/controller.hpp`
3. `nav2_planner/include/nav2_planner/planner_server.hpp`
4. `nav2_controller/include/nav2_controller/controller_server.hpp`
5. `nav2_navfn_planner` 或 `nav2_smac_planner`
6. `nav2_regulated_pure_pursuit_controller`
7. `nav2_dwb_controller`
8. 最后再看 `nav2_bt_navigator` 和 `nav2_behavior_tree`

---

## 二、你自己的项目往哪写

你的项目位置：

```text
/home/hao/pnc_nav2
```

当前主线应该是：

```text
pnc_nav_core      -> 框架、插件接口、导航服务器
pnc_nav_planners  -> A*、RRT*、DWA、Pure Pursuit、Stanley 等算法插件
pnc_nav_map       -> 后续 3D 地图、OctoMap、ESDF、可通行性
pnc_nav_bringup   -> launch、参数、RViz、仿真组合
pnc_nav_sim       -> Gazebo world、机器人模型、传感器
```

不要一开始就把所有包都做满。最开始只需要保证下面这条链能跑：

```text
goal_pose
  -> NavServer
  -> AStar3D.createPlan()
  -> publish global_plan
  -> PurePursuit3D 或 DWA3D.computeVelocityCommand()
  -> publish cmd_vel
  -> Gazebo diff_drive robot moves
```

---

## 三、阶段 1：最小 2D 闭环

目标：先把“小 Nav2 骨架”跑通。

这一阶段不要追求 3D、机器狗、OctoMap。只要 2D 差速小车能收到目标点、规划路径、发布速度、在 Gazebo 里动起来。

### 写什么

#### 1. `pnc_nav_core`

重点文件：

```text
src/pnc_nav_core/include/pnc_nav_core/nav_server.hpp
src/pnc_nav_core/src/nav_server.cpp
src/pnc_nav_core/include/pnc_nav_core/global_planner_base.hpp
src/pnc_nav_core/include/pnc_nav_core/local_planner_base.hpp
src/pnc_nav_core/include/pnc_nav_core/path_tracker_base.hpp
```

要做成：

- `NavServer` 能订阅 `goal_pose`
- 能通过 TF 获取当前机器人位姿
- 能加载 `GlobalPlannerBase` 插件
- 能加载 `LocalPlannerBase` 或 `PathTrackerBase` 插件
- 状态机先只保留核心闭环：

```text
IDLE -> PLANNING -> FOLLOWING -> SUCCEEDED
                  -> FAILED
```

面试讲法：

```text
我参考了 Nav2 的 planner_server 和 controller_server，但为了项目早期可控，
先把规划和控制调度合并到一个 NavServer 中。
后续如果系统复杂度上来，可以再拆成独立 PlannerServer 和 ControllerServer。
```

#### 2. `pnc_nav_planners`

重点文件：

```text
src/pnc_nav_planners/src/global_planners/astar_3d/astar_3d.cpp
src/pnc_nav_planners/src/path_trackers/pure_pursuit_3d/pure_pursuit_3d.cpp
src/pnc_nav_planners/src/local_planners/dwa_3d/dwa_3d.cpp
src/pnc_nav_planners/plugins.xml
```

要做成：

- A* 先支持 2D 栅格搜索，z 轴先保留接口。
- Pure Pursuit 先能跟踪全局路径输出 `cmd_vel`。
- DWA 可以稍后写，先不阻塞闭环。
- 每个插件都必须能通过 pluginlib 加载。
- 每个插件都要能从参数文件读取参数。

优先级：

```text
AStar3D 2D mode
PurePursuit3D
DWA3D
RRTStar3D
Stanley3D
```

#### 3. `pnc_nav_bringup`

重点文件：

```text
src/pnc_nav_bringup/launch/sim_2d_bringup.launch.py
src/pnc_nav_bringup/config/nav_params.yaml
```

要做成：

- 启动 Gazebo 2D 世界。
- 启动差速小车模型。
- 启动 `nav_server_node`。
- 参数中选择 A* + Pure Pursuit 或 A* + DWA。
- RViz 能看到机器人、TF、`global_plan`、`local_plan`。

#### 4. `pnc_nav_sim`

重点文件：

```text
src/pnc_nav_sim/launch/diff_drive_sim.launch.py
src/pnc_nav_sim/worlds/simple_maze.world
src/pnc_nav_sim/urdf/diff_drive/diff_drive.urdf.xacro
```

要做成：

- Gazebo 能启动 `simple_maze.world`。
- 差速小车能订阅 `cmd_vel`。
- 能发布 odom 和 TF。

### 阶段完成标准

这一阶段完成时，你应该能做到：

```text
ros2 launch pnc_nav_bringup sim_2d_bringup.launch.py
```

然后在 RViz 发布一个 `goal_pose`，机器人能：

1. 收到目标点
2. 规划出 `global_plan`
3. 发布 `cmd_vel`
4. 在 Gazebo 中移动
5. 到达目标后停止

面试中这就是你的第一个可演示闭环。

---

## 四、阶段 2：算法插件完善

目标：让项目从“能跑”变成“能讲算法”。

### 写什么

#### 1. A* 规划器

重点讲：

- 栅格离散化
- open set / closed set
- 启发式函数
- 对角移动
- 障碍代价
- 路径回溯和平滑

要做成：

- 支持 2D 模式稳定运行。
- 接口保留 3D 模式。
- 查询 `CostmapInterface`，不要直接依赖某一种地图。

#### 2. Pure Pursuit 跟踪器

重点讲：

- 最近路径点搜索
- 前视距离
- 曲率计算
- 根据曲率限速
- 到达终点判断

要做成：

- 能稳定跟踪 A* 输出路径。
- RViz 能看到路径和机器人运动关系。

#### 3. DWA 局部规划器

重点讲：

- 动态窗口
- 速度采样
- 前向仿真
- 障碍物代价
- 路径距离代价
- 目标距离代价

要做成：

- 能输出局部轨迹 `local_plan`。
- 遇到简单障碍能选择绕行轨迹。
- 参数可调，比如速度采样数、仿真时间、代价权重。

### 阶段完成标准

这一阶段完成时，你应该能演示：

- A* 全局路径
- Pure Pursuit 路径跟踪
- DWA 局部轨迹
- 修改参数后机器人行为变化

面试讲法：

```text
我把算法都做成 pluginlib 插件，所以同一个 NavServer 不需要改代码，
只改参数就可以切换不同规划器和控制器。
```

---

## 五、阶段 3：3D 地图与可通行性

目标：让项目真正区别于普通 2D Nav2 demo。

### 写什么

建议新增或补齐：

```text
src/pnc_nav_map
```

内部可以按这个方向组织：

```text
pnc_nav_map/
  map_server/
  map_converter/
  traversability/
```

要做成：

- 支持点云输入。
- 支持 OctoMap 或 3D 占据栅格。
- 支持 2.5D elevation map。
- 支持 traversability 查询。
- 实现一个真正的 `CostmapInterface` 3D 版本。

### 和阶段 1 的关系

阶段 1 的规划器不应该被重写，只应该替换地图来源：

```text
Phase 1:
AStar3D -> Simple 2D costmap

Phase 3:
AStar3D -> OctoMap / Traversability costmap
```

这就是接口抽象的价值。

面试讲法：

```text
Nav2 原生主要面向 2D costmap，我这里把地图查询抽象成 CostmapInterface。
算法只关心 getCost、isOccupied、isInBounds，
所以后续可以从 2D costmap 平滑切换到 OctoMap、ESDF 或可通行性地图。
```

### 阶段完成标准

这一阶段完成时，你应该能做到：

- RViz 中显示点云或 3D 地图。
- A* 在 3D 或 2.5D 地图上规划。
- 可通行性影响路径选择。
- 能解释坡度、台阶、高度变化如何进入代价函数。

---

## 六、阶段 4：四足平台与运动层

目标：把导航层接到四足机器人平台，但不要把自己陷进底层步态控制。

### 写什么

重点不是自己写 12 关节控制，而是保留清晰边界：

```text
Navigation Layer -> cmd_vel -> Locomotion Layer -> joint commands
```

建议方向：

- 继续保留 `cmd_vel` 作为导航层输出。
- 真四足模型可以考虑 CHAMP 或已有 locomotion。
- 你的 `pnc_nav_control` 后续再实现 RL / MPC / CPG 步态接口。
- 面试项目重点仍然放在规划、避障、路径跟踪、可通行性。

### 阶段完成标准

这一阶段完成时，你应该能做到：

- Gazebo 中四足机器人接收 `cmd_vel`。
- 导航层不关心底层是差速小车还是四足机器人。
- 能解释为什么 `cmd_vel` 是导航层和运动层之间的边界。

面试讲法：

```text
我没有一开始把导航和步态控制耦合在一起。
导航层只输出速度意图，运动层负责把速度意图转换成四足机器人的关节控制。
这样同一套规划控制框架可以先在差速小车上验证，再迁移到机器狗。
```

---

## 七、阶段 5：任务层和恢复行为

目标：从简单状态机升级为更接近 Nav2 的任务系统。

### 学什么

看官方：

```text
nav2_bt_navigator
nav2_behavior_tree
```

### 写什么

可以先不直接上复杂 BehaviorTree，而是逐步增加：

- 重新规划
- 卡住检测
- 目标取消
- 超时失败
- 清除局部地图
- 原地旋转恢复
- 后退恢复
- 多目标点导航

### 阶段完成标准

这一阶段完成时，你应该能做到：

- 机器人规划失败后能进入恢复行为。
- 跟踪失败后能重新规划。
- 能取消当前目标。
- 能处理连续目标点。

面试讲法：

```text
我早期用简单状态机降低复杂度。
当基本闭环稳定后，再参考 Nav2 的 BT Navigator，
把导航任务拆成可组合的行为节点。
```

---

## 八、推荐总开发顺序

最推荐按这个顺序自己写：

```text
1. NavServer 最小状态机
2. GlobalPlannerBase / LocalPlannerBase / PathTrackerBase
3. AStar3D 的 2D 模式
4. PurePursuit3D
5. 2D Gazebo bringup
6. RViz 可视化 global_plan / local_plan
7. DWA3D
8. RRTStar3D / Stanley3D
9. pnc_nav_map
10. OctoMap / elevation / traversability
11. 四足平台 cmd_vel 接入
12. 恢复行为和任务层
```

不要反过来从机器狗、CHAMP、LIO-SAM、OctoMap 开始。那些都很吸引人，但会把早期闭环拖慢。

---

## 九、每个阶段你应该能讲什么

### 阶段 1 能讲

```text
我参考 Nav2 架构，实现了一个最小导航闭环。
它能接收目标点，调用全局规划插件，生成路径，再调用控制插件输出速度。
```

### 阶段 2 能讲

```text
我把 A*、Pure Pursuit、DWA 都做成插件。
这样框架和算法解耦，后续算法替换不需要改 NavServer。
```

### 阶段 3 能讲

```text
我通过 CostmapInterface 抽象地图查询，
让同一个规划器既能跑 2D costmap，也能跑 3D OctoMap 或可通行性地图。
```

### 阶段 4 能讲

```text
导航层输出 cmd_vel，运动层负责步态控制。
这个边界让系统可以先用差速小车验证，再迁移到四足机器人。
```

### 阶段 5 能讲

```text
早期我用状态机保证系统简单稳定。
后期参考 Nav2 BT Navigator，把恢复行为和任务逻辑拆成更灵活的行为节点。
```

---

## 十、最小可展示版本定义

如果时间有限，优先做到这个版本：

```text
Gazebo simple_maze
  + diff_drive robot
  + NavServer
  + AStar3D 2D mode
  + PurePursuit3D
  + RViz goal_pose
  + global_plan visualization
  + cmd_vel output
```

这个版本足够面试讲清楚：

- 架构分层
- 插件化
- 规划与控制闭环
- 从 Nav2 学到了什么
- 后续怎么扩展到 3D 和四足机器人

等这个跑通后，再做 DWA、OctoMap、四足平台。
