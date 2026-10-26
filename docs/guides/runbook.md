# Runbook（启动矩阵 + 参数手册）

> 最后核对：commit `ed5280c`（2026-10-23）
> 用途：日常开发"查表"——怎么起、参数什么意思、改完怎么验证

---

## 1. 环境约定（先看这段）

| 项 | 约定 | 为什么 |
|---|---|---|
| `ROS_DOMAIN_ID` | `43` | 与机器上其它 ROS 栈（如 MoveXBot，默认域）隔离，避免话题串台 |
| `GZ_PARTITION` | `pnc_nav` | Gazebo transport 隔离；否则两个仿真会互相"看到"对方的模型 |
| `PATH` | `export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH` | `gz` 命令在 ROS vendor 目录里 |
| `use_sim_time` | `true` | 仿真必须；真机改 `false` |

`./start_nav.sh` 已经把上面三项设好，但它前台常驻；进阶操作建议手工 `ros2 launch`。

---

## 2. 启动矩阵

| 入口 | 起什么 | 状态 |
|---|---|---|
| `pnc_nav_bringup/gz_sim_2d_bringup.launch.py` | **主入口**：Gazebo playground + 差速车 + `maps/111` + map_server + nav_server + RViz | ✅ 可用 |
| `pnc_nav_bringup/sim_2d_bringup.launch.py` | 旧入口名，转发到 `gz_sim_2d_bringup` | ✅ 兼容 |
| `pnc_nav_bringup/turtlebot3_test.launch.py` | 旧入口名，同上 | ✅ 兼容 |
| `pnc_nav_sim/gz_3d_lidar.launch.py` | 同一台差速车 + Velodyne 16 线（`diff_drive_3d.urdf.xacro` + `bridge_3d.yaml`） | ✅ 可用（输出 `/points`） |
| `pnc_nav_bringup/gz_sim_3d_lidar_bringup.launch.py` | **3D 激光完整导航**：3D 车 + maps/111 + map_server + nav_server（点云障碍层）+ RViz | ✅ 可用 |
| `pnc_nav_map/elevation_map.launch.py` | 高程图节点：`/points` → `/elevation_map` + `/traversability_map`（需先起 3D 仿真） | ✅ 可用（Phase 3.1） |
| `gz_sim_3d_lidar_bringup.launch.py use_elevation:=true world:=slope_test.sdf` | 坡道/台阶场景 + 地形代价（Phase 3 验收） | ✅ 可用 |
| `pnc_nav_bringup/amcl_localization.launch.py` | 地图 + AMCL 定位 | ✅ 可用（需先起仿真） |
| `pnc_nav_bringup/cartographer_slam.launch.py` | Cartographer 建图 | ✅ 可用（需先起仿真） |
| `pnc_nav_bringup/sim_3d_bringup.launch.py` | 名义上的 Go2 3D 启动 | ❌ **不可用**：include 外部包 `go2_config` / `lio_sam`，本仓库不含 |
| `./start_nav.sh` | 一键：2D 主入口 + 域隔离 | ✅ 可用 |

### 2.1 主入口的 launch 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_sim_time` | `true` | 使用 `/clock` |
| `params_file` | `pnc_nav_bringup/config/nav_params.yaml` | 参数文件（可传自定义） |
| `map` | `pnc_nav_bringup/maps/111/111.yaml` | 地图 yaml 绝对路径 |
| `use_rviz` | `true` | 是否起 RViz |
| `use_gui` | `true` | 是否起 Gazebo GUI（服务器上设 `false`） |
| `x_pose` / `y_pose` / `z_pose` | `0 / 0 / 0.15` | 机器人初始位姿 |

示例：

```bash
# 服务器/无显示器
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py use_gui:=false use_rviz:=false

# 换地图
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py \
  map:=$HOME/pnc_nav2/src/pnc_nav_bringup/maps/111/111.yaml

# 换参数文件（跑 DWA 对比实验时）
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py params_file:=/tmp/nav_params_dwa.yaml
```

---

## 3. 参数手册

### 3.1 `nav_server.ros__parameters`（框架层）

| 参数 | 类型 | 代码默认 | yaml 现值 | 说明 / 调参建议 |
|---|---|---|---|---|
| `control_frequency` | double | 20.0 | 20.0 | 控制循环频率。调高更跟手但更吃 CPU |
| `global_frame` / `robot_frame` | string | `map` / `base_link` | 同 | TF 查询用 |
| `goal_tolerance_dist` | double | 0.15 | 0.15 | 到达距离容差（传给插件）。太小会"到不了"，太大会"提前停" |
| `goal_tolerance_angle` | double | 0.0 | 0.0 | 终点朝向容差；**0 = 不检查**（仅 DWA2D 支持终点对齐） |
| `max_planning_retries` | int | 3 | 3 | 恢复次数上限。0 = 不恢复，一次失败就结束 |
| `global_planner_plugin` | string | `AStar3D` | `AStar2D` | 默认全局插件 |
| `local_planner_plugin` | string | `""` | `""` | 非空则优先用局部规划器（与 tracker 二选一） |
| `path_tracker_plugin` | string | `PurePursuit3D` | `PurePursuit2D` | 路径跟踪器 |
| `recovery_spin_duration` | double | 1.0 | 1.0 | 恢复时原地旋转时长（s） |
| `recovery_angular_vel` | double | 0.6 | 0.6 | 恢复旋转角速度（rad/s） |
| `feedback_eta_speed` | double | 0.25 | 0.25 | 算 ETA 用的参考速度（m/s） |
| `progress_timeout` | double | 8.0 | 8.0 | 多久没位移判卡住（s）；`<=0` 关闭 |
| `progress_epsilon` | double | 0.05 | 0.05 | 卡住的位移阈值（m） |
| `tf_timeout_ticks` | int | 50 | 50 | 连续多少个周期拿不到 TF 判失败（20Hz → 2.5s） |
| `recovery_collision_check` | bool | true | true | 恢复旋转前做碰撞检查（KI-012） |
| `recovery_safety_radius` | double | 0.35 | 0.35 | 安全检查半径（m），建议 ≥ 车体半径 + 0.1 |

### 3.2 `map_adapter.*`（全局静态代价地图）

| 参数 | yaml 现值 | 说明 / 调参建议 |
|---|---|---|
| `occupied_threshold` | 65 | `/map` 中栅格值 > 此值视为障碍（0–100） |
| `robot_radius` | 0.25 | 机器人内切圆半径（m），决定 INSCRIBED 层 |
| `inflation_radius` | 0.35 | 膨胀半径（m）。调大更保守，但窄通道可能无解 |
| `cost_scaling_factor` | 3.0 | 代价衰减系数，越大衰减越快 |
| `unknown_as_occupied` | true | 未知区域是否当障碍。真机建议 true，调试可 false |

### 3.2b `local_costmap.*`（局部滚动代价地图）

| 参数 | yaml 现值 | 说明 / 调参建议 |
|---|---|---|
| `enable` | true | 关掉后退化为"局部规划器直接用静态图"（旧行为） |
| `scan_topic` | `nav_scan` | 2D 激光话题 |
| `window_size` | 8.0 | 窗口边长（m），机器人居中 |
| `resolution` | 0.05 | 窗口分辨率（m/cell） |
| `obstacle_ttl` | 3.0 | 激光障碍点存活时间（s）；太短会闪、太长有残影 |
| `max_range` | 6.0 | 只用这个范围内的激光（m） |
| `inflation_radius` | 0.35 | 窗口内膨胀半径（m） |
| `robot_radius` | 0.25 | 内切半径（m） |
| `cost_scaling_factor` | 3.0 | 代价衰减系数 |
| `include_static` | true | 是否把静态图障碍叠进窗口（激光能覆盖时影响很小） |
| `unknown_as_occupied` | false | 窗口外是否当障碍 |
| `cloud_topic` | "" | 点云话题；填 `points` 启用 3D 点云障碍层（与 scan 二选一） |
| `cloud_stride` / `cloud_min_z` / `cloud_max_z` / `cloud_max_range` | 8 / 0.10 / 1.50 / 8.0 | 点云降采样与高度带过滤 |

调试：`python3 test/check_costmap_layers.py` 打印窗口尺寸/原点/占用格数；RViz 看 `/costmap` 与 `/obstacle_layer`。

### 3.2c `elevation.*`（2.5D 高程图节点）

| 参数 | 现值 | 说明 |
|---|---|---|
| `cloud_topic` | `points` | 点云话题 |
| `window_size` / `resolution` | 8.0 / 0.1 | 滚动窗口边长 / 高程格分辨率 |
| `stride` | 8 | 每 N 个点取一个 |
| `min_range` / `max_range` | 0.3 / 8.0 | 传感器系水平距离范围（m） |
| `min_z` / `max_z` | **-0.30** / 1.50 | 高度带；**必须包含地面**（与障碍层的 0.10 不同） |
| `recenter_threshold` | 1.0 | 机器人移动超过它才滚动窗口（m） |
| `fill_radius` | 0.4 | 未知格填充半径（补 16 线激光地面环间空洞，取邻域最低 z） |
| `slope_max_deg` / `step_max_m` / `roughness_max_m` | 20 / 0.10 / 0.05 | 可通行性阈值 |
| `publish_rate` | 2.0 | 可视化发布频率（Hz） |

验证：`ros2 topic echo /traversability_map --once`；RViz 用 Map 显示 `/elevation_map`（0=可通行，100=不可通行）。

### 3.3 `global_planner.*`（A* 系列）

| 参数 | 代码默认 | yaml 现值 | 说明 |
|---|---|---|---|
| `resolution` | 0.05 | 0.1 | 搜索分辨率（m）。细化更贴障碍，但更慢 |
| `heuristic_weight` | 1.2 | 1.2 | 启发权重；>1 为加权 A*（更快、可能次优） |
| `max_iterations` | 100000 | 100000 | 搜索上限，防止大图卡死 |
| `allow_unknown` | false | false | 是否允许穿越未知区域 |
| `diagonal_movement` | true | true | 允许对角移动（当前 AStar2D 内未真正读取） |
| `bspline_sample_count` | 10 | 10 | B 样条每段采样点数（平滑用） |
| `height_penalty` | — | 2.0 | **yaml 残留**：AStar2D 未声明/未使用（3D 预留） |
| `use_3d` | — | false | **yaml 残留**：AStar2D 未读取（3D 预留） |

### 3.4 `path_tracker.*`（PurePursuit2D）

| 参数 | 代码默认 | yaml 现值 | 说明 |
|---|---|---|---|
| `lookahead_distance` | 0.6 | 0.8 | 基础前视距离（m）。大 → 更平滑但切弯；小 → 更贴路径但抖 |
| `min_lookahead` / `max_lookahead` | 0.3 / 1.5 | 0.4 / 1.8 | 速度自适应前视的上下限 |
| `lookahead_gain` | 0.5 | 0.5 | 前视距离随速度的增益 |
| `max_linear_vel` | 0.5 | 0.25 | 最大线速度（m/s）。仿真 0.25 ≈ 4s/m |
| `max_angular_vel` | 1.0 | 0.6 | 最大角速度（rad/s） |
| `min_linear_vel` | 0.05 | 0.03 | 最小线速度，太小会原地磨 |

### 3.5 `local_planner.*`（DWA2D，⚠️ 已知问题）

| 参数 | 代码默认 | yaml 现值 | 说明 |
|---|---|---|---|
| `max_vel_x` / `min_vel_x` | 0.4 / 0.0 | 0.2 / 0.0 | 前进速度范围（m/s） |
| `max_vel_theta` | 1.0 | 0.8 | 最大角速度（rad/s） |
| `acc_lim_x` / `acc_lim_theta` | 0.8 / 1.5 | 1.0 / 3.0 | 加速度上限（动态窗口用） |
| `sim_time` | 1.2 | 1.0 | 前向仿真时长（s） |
| `vx_samples` / `vtheta_samples` | 8 / 15 | 12 / 21 | 采样数；越大越慢越稳 |
| `dt` | 0.1 | 0.1 | 仿真步长（s） |
| `path_distance_bias` | 32.0 | 16.0 | 横向偏离路径权重（改用折线投影后可调小） |
| `goal_distance_bias` | 24.0 | 20.0 | 沿路径剩余里程权重（驱动前进） |
| `obstacle_cost_bias` | 0.02 | 0.3 | 避障权重 |
| `heading_distance_bias` | 15.0 | 25.0 | 朝向对齐权重 |

> ✅ DWA2D 已修复"只转不走"（KI-013）：路径代价改折线投影、目标代价改剩余弧长、朝向改前视切线。
> 实测 2/4/5/6m 目标均成功，且能在 0.4m 箱子挡路时绕行（44.4s / 4.21m）。
> baseline 仍默认 `PurePursuit2D`（更快），需要避障时切 DWA2D。

---

## 4. 切换插件（两种方式）

**方式 A：改参数（默认插件）**

```yaml
global_planner_plugin: "pnc_nav_planners::AStar2D"
local_planner_plugin: ""                                   # 非空则优先
path_tracker_plugin: "pnc_nav_planners::PurePursuit2D"
```

改完重启节点（或整个 launch）。

**方式 B：action 里热插拔（只支持全局规划器）**

```bash
ros2 action send_goal /navigate_to_3d_pose pnc_nav_interfaces/action/NavigateTo3DPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0}, orientation: {w: 1.0}}},
    planner_id: 'pnc_nav_examples::StraightLinePlanner'}"
```

可用插件名：

| 类型 | 插件类名 |
|---|---|
| 全局 | `pnc_nav_planners::AStar2D`、`pnc_nav_planners::AStar3D`、`pnc_nav_planners::RRTStar3D`、`pnc_nav_examples::StraightLinePlanner` |
| 跟踪 | `pnc_nav_planners::PurePursuit2D`、`pnc_nav_planners::PurePursuit3D`、`pnc_nav_planners::Stanley3D` |
| 局部 | `pnc_nav_planners::DWA2D`、`pnc_nav_planners::DWA3D` |

> 热插拔会先 `deactivate + cleanup` 旧插件；如果类名写错，本次 goal 以 `error_code=6` 失败。

---

## 5. 常用命令速查

```bash
# 节点 / action / 话题
ros2 node list
ros2 action list
ros2 action info /navigate_to_3d_pose
ros2 topic list
ros2 topic hz /odom            # 期望 ~50Hz
ros2 topic echo /map --once | head -8

# TF
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_monitor map base_link

# 参数
ros2 param list /nav_server
ros2 param get /nav_server max_planning_retries

# 发目标（带 feedback）
ros2 action send_goal /navigate_to_3d_pose pnc_nav_interfaces/action/NavigateTo3DPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback

# 冒烟测试脚本
python3 test/action_smoke_test.py feedback|cancel|unreachable
```

---

## 6. 场景配方

| 想做什么 | 怎么做 |
|---|---|
| 跑快一点 | `path_tracker.max_linear_vel: 0.4`；同时把 `max_angular_vel` 提到 1.0 |
| 更保守（远离障碍） | `map_adapter.inflation_radius: 0.5`、`obstacle_cost_bias` 调大 |
| 关闭恢复行为（调试单次规划） | `max_planning_retries: 0` |
| 关闭卡住检测（看纯跟踪效果） | `progress_timeout: 0` |
| 无界面跑 | `use_gui:=false use_rviz:=false` |
| 看规划耗时/失败原因 | 看 `nav_server` 日志的 `Goal finished: ... error_code=...` |
| 换地图 | 改 `map:=<绝对路径>/xxx.yaml`，注意 `.png/.pgm` 同目录 |
| 确认障碍层在工作 | `python3 test/check_costmap_layers.py`；或 RViz 看 `/obstacle_layer` |
| 动态障碍实验 | Gazebo 里 spawn 一个箱子：`ros2 run ros_gz_sim create -world world_demo -file /tmp/box.sdf -name box -x 3 -y 0 -z 0.4` |
| 换机器人/传感器 | 见 `tutorials/04_maps_and_worlds.md`（🚧） |

---

## 7. 排错索引

| 症状 | 去哪看 |
|---|---|
| `ros2 launch`/`ros2 action` 命令不存在 | 本页 §8 依赖清单 |
| TF 两棵树、位姿跳变（双实例） | `tutorials/01_quickstart.md` §6 |
| action 不返回 / 没有 action server | 检查 `ROS_DOMAIN_ID`、`ros2 node list` |
| 到点不停/停短、原地转圈 | `notes/known_issues.md`（DWA2D 已知问题） |
| Gazebo 资源路径、GUI 缓存、`ros_gz` 版本 | `guides/sim_setup.md`（🚧） |

---

## 8. 依赖清单（宿主机最小集）

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-ros2launch ros-jazzy-ros2action ros-jazzy-ros2topic ros-jazzy-ros2node
```

构建（跳过需要 `abseil` 的 `third_party/cartographer` 与依赖它的 RViz 面板）：

```bash
colcon build --symlink-install --packages-select \
  pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_sim pnc_nav_bringup \
  pnc_nav_examples pnc_nav_map
```

---

## 9. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：启动矩阵、参数手册、插件切换、场景配方（commit `ed5280c`） |
| 2026-10-23 | 新增 `local_costmap.*` 参数表、障碍层调试与动态障碍实验配方 |
| 2026-10-23 | 新增点云障碍层参数、3D 激光 bringup 入口 |
| 2026-10-23 | 新增高程图节点入口与 `elevation.*` 参数表 |
