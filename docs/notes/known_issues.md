# 已知问题（Known Issues）

> 最后核对：commit `216e21d`（2026-10-23）
> 用法：改代码前先扫一眼；修完把条目移到文末"已修复（留档）"，别删——它是踩坑记录。

---

## 未修复

### KI-001 DWA2D 到点停短（约 0.25 m 就停）✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 用 `DWA2D` 时机器人走到离目标 ~0.25–0.30 m 处停止，`isGoalReached` 一直为 false，触发卡住检测 → 恢复 → 死循环，最终 `error_code=3` |
| 证据 | `DWA2D: pose=(2.17,0.19) goal=(0.1,0.2) dist=0.05` + `与目标距离差未达到`（每周期刷屏）；feedback `dist=0.30m` 不变 |
| 根因（推测） | ① DWA 内部"局部目标点"与最终 goal 不是同一个点；② 目标点落在膨胀层内，DWA 不敢靠近；③ 到达判定用 `dist+angle` 双阈值，角度不满足时永远不判成功 |
| 影响 | 默认配置已切回 `PurePursuit2D`，不影响 baseline；但"局部避障"这条线被卡住 |
| 修复方式 | 引入**局部滚动代价地图**（`RollingCostmap`）后，目标点不再落在"全局静态膨胀层的 LETHAL 区"，DWA 能正常逼近并判定到达：实测 `success=true, code=0, 10.06s, 1.86m` |
| 遗留 | 目标点被真实障碍覆盖时仍会失败（这是合理行为，但错误码应区分"不可达"与"超时"） |

### KI-002 日志里的 `static=double_max`（误判为代价溢出）✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 日志里出现 `static=179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000` |
| 含义 | 即 `std::numeric_limits<double>::max()`——静态代价分支返回了"无穷大" |
| 结论 | **不是代价溢出**：`static_cost` 初值是 `double::max()`，表示"本轮没采样到 (vx=0, vθ=0) 的静止轨迹"，只是被当数字打印了出来 |
| 修复方式 | 打印时把它转成 `-1.0`（`static_cost_dbg`），代价计算逻辑未改（本身没问题） |

### KI-003 DWA2D 每周期打 INFO，日志刷屏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 20Hz 下每周期输出 `DWA2D: cmd=...` 与 `与目标距离差未达到` |
| 影响 | 日志文件暴涨、真实错误被淹没、终端不可读 |
| 修复方式 | `isGoalReached` 的 4 条日志降为 `RCLCPP_DEBUG`；实测同一场景刷屏行数 200 → **0** |

### KI-004 `distance_remaining` 与到达判定语义不一致 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | feedback 显示 `dist=0.20 m`（到目标直线距离），但 result 已经 `success=true`；或者机器人已在 0.07 m 却为了"转正朝向"磨 20+ 秒、触发恢复 |
| 原因 | 到达判定由插件各自实现：`PurePursuit2D` 用"到路径末点距离"，`DWA2D` 用"距离 + 朝向"双条件，而 feedback 用"到目标点直线距离" |
| 决定 | **到达判定统一为"距离优先"**：`goal_tolerance_dist`（默认 0.15m）够了就算到达；终点朝向改为**可选**（`goal_tolerance_angle` 默认 **0 = 不检查**） |
| 实现 | `DWA2D::isGoalReached()`：先判距离；`angle_tolerance <= 0` 直接返回 true，否则再判朝向；`NavServer` 与 yaml 默认值同步改为 0.0 |
| 效果 | 绕障场景：`44.4s → 25.5s`（不再为转正朝向磨时间）；目标 6m：11.4s；PP baseline：2m→16.3s / 5m→19.6s 无回归 |
| 说明 | 需要"到点后朝向也对齐"的场景（充电桩、对接）把 `goal_tolerance_angle` 设成 0.1~0.3 即可 |

### KI-005 恢复行为不清代价地图 🟡 部分修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | `RECOVERING` 只做原地旋转 + 重规划 |
| 现状 | 进入恢复时会调用 `RollingCostmap::clear()` 清掉激光障碍点（`CostmapInterface` 新增 `clear()` 虚函数） |
| 影响 | 静态地图下无影响；一旦加入 obstacle layer，"卡住"往往伴随代价地图里的残留障碍，不清理会反复卡 |
| 修复方向 | 在 `CostmapInterface` 增加 `clear()`（或 `resetLayers()`），恢复序列改为：清局部图 → 后退 → 旋转 → 重规划 |

### KI-006 `sim_3d_bringup.launch.py` 不可用

| 项 | 内容 |
|---|---|
| 现象 | launch 里 include `go2_config`（`gazebo_velodyne.launch.py`）与 `lio_sam`（`lidar.launch.py`），这两个包不在本仓库 |
| 影响 | 直接跑会因找不到包失败；README 里"3D 导航（机器狗）"实际不可用 |
| 现状 | `third_party/simdog_go2` 是 submodule，URL 指向作者本地路径 `/home/hao/simdog`，克隆后为空目录 |
| 修复方向 | 把 go2 模型 + CHAMP（或其它 locomotion）抽进 `third_party`，并把 `sim_3d_bringup` 改成只依赖仓库内包；或明确标注为"占位文件" |

### KI-007 全量 `colcon build` 失败（`pnc_nav_rviz` + `third_party/cartographer`）

| 项 | 内容 |
|---|---|
| 现象 | `colcon build` 在 `cartographer` 处失败：`Could not find a package configuration file provided by "absl"` |
| 影响 | 新人一键全量构建失败；`pnc_nav_rviz`（RViz 控制面板）依赖 `cartographer_ros_msgs`，所以也编不出来 |
| 临时规避 | 用 `--packages-select pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_sim pnc_nav_bringup`（教程与 runbook 已这样写） |
| 修复方向 | 给 `third_party/cartographer*` 加 `COLCON_IGNORE`，或装 `libabsl-dev` 等依赖（不推荐，依赖很重）；把 RViz 面板里 `write_state` 客户端改为可选加载 |

### KI-008 两个仿真实例 → TF 冲突（运行期坑）

| 项 | 内容 |
|---|---|
| 现象 | 同一个终端/同一次调试里位姿在两点间跳变；`tf2_echo` 报 `two or more unconnected trees`；DWA 日志里的 `goal=` 值忽大忽小 |
| 根因 | 上一个 launch 的 `gz sim` 没退干净（`pkill -f "ros2 launch"` 不一定带走子进程），两个实例都往同一个 `GZ_PARTITION` 与 `ROS_DOMAIN_ID` 发 `/tf` |
| 修复方向 | ① 启动脚本里加"单实例检查"；② 文档写清理步骤（已在 `01_quickstart.md` §6、`runbook.md`）；③ 长期用 `GZ_PARTITION` 随机后缀 |

### KI-009 `PurePursuit2D` 的 `debug` 参数缺前缀 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 代码里 `node_->declare_parameter("debug", false)`，没有 `<name>.` 前缀 |
| 影响 | 与其它实例/节点参数可能串味；违背插件参数命名约定 |
| 修复 | 改为 `name_ + ".debug"`（`path_tracker.debug`），代码已同步 |

### KI-010 yaml 里存在无效参数（AStar2D 不读）

| 项 | 内容 |
|---|---|
| 现象 | `nav_params.yaml` 里 `global_planner.height_penalty`、`global_planner.use_3d` 在 `AStar2D` 中没有 `declare_parameter` |
| 影响 | 改了不起作用；读者误以为在调参 |
| 修复方向 | 要么在 `AStar2D` 里实现（高度惩罚对 2D 无意义），要么从 yaml 删除并注明"3D 预留" |
| 现状 | 已在 `runbook.md` 参数表标注为"yaml 残留" |

### KI-011 `goal_tolerance_angle` 只有部分插件使用（已澄清，非缺陷）

| 项 | 内容 |
|---|---|
| 现象 | 该参数传给插件，但 `PurePursuit2D` 只按距离判定，`DWA2D` 才用角度 |
| 结论 | 与 KI-004 一并澄清：默认 `0` = 不检查朝向；`PurePursuit2D` 本身不做终点对齐（路径跟踪器的职责边界），需要终点朝向时用 `DWA2D` 并设 `goal_tolerance_angle > 0` |
| 文档 | 已写入 `design/nav_server.md` 参数表与 `guides/runbook.md` |

### KI-012 恢复期的原地旋转不做碰撞检查 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | `RECOVERING` 直接发 `angular.z`，不看代价地图；贴着障碍旋转可能把车体蹭上去 |
| 修复 | 旋转前（以及旋转中的每个周期）用局部滚动代价地图做安全检查：以机器人为心、`recovery_safety_radius`（默认 0.35m）为半径采样 16 个点，任一点 `isOccupied` 为真就跳过旋转、直接回 `PLANNING`；拿不到位姿（TF 问题）同样跳过 |
| 参数 | `recovery_collision_check`（默认 true）、`recovery_safety_radius`（默认 0.35m，建议 ≥ 车体半径 + 余量） |
| 实测 | 车旁 0.5m 放箱子 → 3 次恢复全部 `Recovery rotation blocked`，总耗时 **0.30s** 快速失败；删掉箱子 → 恢复照常旋转（3 次 `Recovery spin done`，3.27s） |
| 盲区 | 安全检查读的是滚动代价地图，"未知区域"按空闲处理，**雷达没看到过的障碍不会被判危险**；真机请把半径设保守 |

### KI-013 DWA2D 在长路径/恢复后出现"只转不走" ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 目标较远（>3m）或进入过一次恢复后，DWA 输出 `vx≈0`、`vθ≈0.01`，机器人原地微动；8s 后卡住检测触发恢复，最终 `error_code=3` |
| 证据 | `DWA2D: cmd=(0.0000, 0.0057) best=33.567 stop=33.567 move=33.571`——**停车与前进代价只差 0.004**，停车永远微弱胜出 |
| 根因（两条） | ① `pathDistanceCost` 量的是"到最近**离散路径点**"的距离：轨迹终点落在两个路径点之间时凭空多出最多半个点距（0.05m）的代价，×path_bias(32)≈1.6，正好抵消前进带来的目标收益；② `headingCost` 朝"目标点"而不是"路径切线"，全局路径绕障时两者打架，前进反而更贵 |
| 修复 | ① 路径代价改为**到路径折线的横向偏差**（点到线段投影）；② 目标代价改为**沿路径的剩余弧长**（`path_length - s`），沿路径前进必然单调下降；③ 朝向代价改为朝**前视点（s+0.6m）**的切线方向；④ 权重调成 `path_distance_bias 16 / goal_distance_bias 20` |
| 效果 | 2.0/4.0/5.0/6.0 m 目标均成功（9.5–24.4s）；绕障实验（0.4m 箱子挡在路中间）**成功绕行**：`success=true, code=0, 44.4s, 4.21m` |
| 遗留 | 终点处需要转正朝向（`goal_tolerance_angle`）时会多花时间，属于"到点对齐"策略问题（KI-004） |

### KI-014 激光视野 240°，背后有盲区

| 项 | 内容 |
|---|---|
| 现象 | 仿真 2D 激光 `angle_min/max = ±2.09 rad`（240° FOV），机器人**正后方 120° 看不到** |
| 影响 | 障碍层不可能标记背后障碍；任何"后退/原地旋转后倒车"的恢复策略都没有后方感知 |
| 证据 | `ros2 topic echo /nav_scan --once`：`angle_min=-2.09, angle_max=2.09, 720` 束 |
| 修复方向 | ① 换 360° 雷达模型；② 恢复行为里"后退"前先旋转扫描；③ 接 3D 激光（`/points`）用点云补盲区 |

### KI-015 3D 点云只做 2.5D 投影，不是真 3D 规划

| 项 | 内容 |
|---|---|
| 现状 | `/points` 按高度带（默认 0.10–1.50m）压成 2D 栅格，生成的是"某一高度切片的可通行性" |
| 影响 | ① 桌底/床底这类"车能钻过去"的空间会被当成障碍；② 有坡度的地形只体现为平面障碍，没有高度/坡度信息 |
| 临时规避 | 调 `cloud_min_z`/`cloud_max_z` 适配场景；或改用 2D 雷达 |
| 修复方向 | Phase 3：OctoMap / 2.5D 高程图 + `CostmapInterface` 的 `getGroundHeight`/`getTraversability`，A* 走 3D 搜索（`AStar3D` 已就位但未验证） |

### KI-016 3D GPU 激光看不到"运行期 spawn"的模型（之前的 3D 绕障结论已撤回）

| 项 | 内容 |
|---|---|
| 现象 | `ros_gz_sim create` 把 0.4m 箱子 spawn 到 (2,0) 后：`gz model --list` 能看到模型、pose 正确，但 `/points` 里该区域只有地面点（z≈-0.31），既没有箱子点，`/obstacle_layer` 也没有对应占用格 |
| 影响 | 之前"3D + DWA 绕障 23.17s/4.07m"的结论**不可靠**——箱子对 3D 激光不可见，机器人很可能是直接穿过；已从文档撤回 |
| 对照 | 2D 激光（`nav_scan`）同样的 spawn 能正常看到（`/obstacle_layer` 半径 0.5m 内 217–256 格），2D 的绕障结论仍然有效 |
| 可能原因 | gz-sim 8 的 `gpu_lidar` 对运行期创建的 static 模型不刷新（渲染侧缓存）；或与 `-s` 无 GUI 的渲染路径有关 |
| 临时规避 | 验证 3D 障碍时把障碍写进 **world SDF**（或在 spawn 后重启仿真），不要依赖运行期 spawn |
| 修复方向 | 确认是 gz 行为还是我们的 bridge/传感器配置；必要时改用 `ray`（CPU）传感器或 spawn 后重启仿真 |

### KI-017 开启地形代价后 A* 规划失败 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 3D + 高程 + `use_elevation_costmap:=true`：从 (3.9,0) 到 (0.5,0) 连续 `Start is occupied` / `no path`，1s 内失败 |
| 根因 | **机器人把自己当成了障碍**：激光/点云打在车体和轮子上，机器人所在格高程被抬高 → 可通行性 0 → A* 起点被占直接放弃 |
| 修复 | ① 高程图与局部点云层都加 **footprint 清除**（`robot_clear_radius=0.45m` 内的点直接丢弃）；② A* 起点被占不再直接失败，打告警继续搜 |
| 实测 | 0→4.0 `success 18.6s/3.90m`；4.0→0.5（原失败用例）`success 57.2s/4.39m`；不再出现 `Start is occupied` |
| 遗留 | 4.0→0.5 耗时偏长（地形代价让路径更绕），后续用指标对比确认是否合理 |

### KI-018 高程坡度估计噪声（16 线激光 + 稀疏点）🟡 坡度幅值已修，覆盖率问题见 KI-019

| 项 | 内容 |
|---|---|
| 现象 | 15° 实测斜坡，相邻格中心差分算出 27–34°，可通行性直接 0（斜坡不可过） |
| 修复（已做） | ① 求梯度前 3×3 均值平滑；② **宽基线中心差分**（`slope_baseline_cells=2`，跨度 0.2m）；③ `slope_max_deg` 20°→25°；④ 测试世界斜坡改实心 0.5m 厚 |
| 效果 | 坡度估计 **27–34° → 13–16°**（实测 15°，误差 ≤3°）；单测 `NoisyRampSlopeIsNotAmplified`（15° + ±0.02m 噪声）通过 |
| 剩余 | 坡面的**高程空洞**导致假台阶（见 KI-019），可通行性仍被压到 0.17–0.5 |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型与链路已通，开环步态推进不可靠（待接 CHAMP 或换平台）

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`），gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`（cmd_vel → 足端轨迹 → 解析 IK，腿长 0.213/0.213）；`gait=walk`（duty 0.75）或 `trot`（duty 0.5） |
| ✅ 已验证 | 站立：`z≈0.19m, roll≈-4°`（关节跟踪站立位 thigh 0.79 / calf -1.56）；节点/话题/桥接/IK 全部正常 |
| ❌ 未达到 | **有效推进**：命令 0.1–0.15 m/s 时，机身在 5–6s 内只位移 0.1–0.2m（≈0.02–0.03 m/s），脚端明显打滑；加大步幅/增益后又会把机器人"踢翻"（roll≈180°） |
| 已排除 | IK 符号（URDF +y 旋转使足端向 -x，已改为 `atan2(-foot_x, r)`）、指令斜率（3–4 rad/s）、增益（p40/d4/cmd10 最稳，p200/cmd30 必翻）、出生高度（0.40→0.55） |
| 调参经验 | gz `JointPositionController`：`p_gain>60 + cmd_max>15` 起步必翻；`cmd_max<10` 或 `p_gain<30` 又托不住身体。开环 IK 步态对参数极敏感 |
| 转机（已采纳轮足方案） | 已接入 **Go2-W**（`go2w_description`，轮足）：腿保持站立位、4 个轮子差速驱动。`profile:=sim_go2w` 下 1m 目标成功（DWA）；**加速度闭环后 48.3s/1.80m → 4.96s/1.08m** |
| 轮足关键参数 | `body_height=0.34`（0.30 时轮子蹭地不转向）、`wheel_radius=0.057`（实测标定）、
`linear_scale=1.15` / `angular_scale=1.75`（补偿滑移）、tracker 用 **DWA** 且 `goal_tolerance_dist=0.25` |
| 足式步态建议 | 若要真足式行走：接 CHAMP（apt 已有 `gz_ros2_control`/`ros2_control`）或做接触/平衡闭环；开环 IK 步态在 gz 里推进不可靠（已实测） |
| 相关 | `profile:=sim_go2` 可跑通 nav→cmd_vel→关节→odom 链路，但行走精度不足，不建议作为导航性能实验平台 |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型/ode 链路已通，闭环与步态质量仍有限

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`）；gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`：cmd_vel → 足端轨迹 → **解析 IK**（Go2 腿长 0.213/0.213）；`gait=walk`（三足支撑，duty 0.75）或 `trot`（对角） |
| 实测（walk, 0.1 m/s） | 站立 roll≈±5°、base z≈0.19；走 6s 位移 **0.71m（≈0.12 m/s）**、roll +4°/pitch -11°（未翻车） |
| 实测（trot） | 关节呈对角相位（corr(FL,RR)=+0.53 vs corr(FL,FR)=+0.23），但开环**会翻车**（roll≈180°） |
| 调参经验 | gz 侧 `p_gain 200→40`、`d_gain 10→4`、`cmd_max 30→10` + 指令斜率限制（4 rad/s）后，站立/慢走才稳定 |
| 闭环（`profile:=sim_go2`） | `/odom`、`map→base`、action、cmd_vel→关节全部打通；但 nav 闭环下机器人会走偏（PP 假设差速瞬时转向，walk 转向慢），**路径跟踪精度待调** |
| 不支持 | 接触时序/平衡反馈（无 IMU/足端力闭环）、CHAMP/RL 步态、地形适应性 |
| 下一步 | ① 给运动层加 IMU 反馈/速度环；② 接 CHAMP（apt 有 `gz_ros2_control`）；③ 用 DWA 替代 PP（转向更平滑） |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足 demo 只是"链路验证"，不是可用的四足仿真

| 项 | 内容 |
|---|---|
| 现状 | 已接入 **Unitree 官方 `go2_description`**（BSD，vendored 在 `src/pnc_nav_sim/go2/`，含 dae 网格与 12 关节），
用我们的 gz `JointPositionController` + `FakeTrotLocomotion` 驱动；另保留极简盒子四足做对照 |
| 已验证 | `cmd_vel=0.3` → `/joint_states` 中 Go2 `FL_thigh_joint` 摆动 **-0.50 ~ +0.46 rad**；静态时关节≈0；模型在 gz 中正常加载（mesh 由 `package://` → `file://` 解析） |
| 不支持 | 真步态（无接触时序/落足，机器人会倒）、CHAMP/RL 集成、四足 odom（腿部里程计/IMU）、与导航栈闭环 |
| 下一步 | 接 CHAMP 或 RL 步态插件；补 odom；再考虑四足进 `bringup` profile |

### KI-019 地形：DWA 上坡已打通（地形只作用于全局规划）

| 项 | 内容 |
|---|---|
| 已修复 | ① 高程填充改**局部平面拟合** + clamp；② `fill_radius 0.8` / `robot_clear_radius 0.7`；③ A* `start_clear_radius 0.8m`；④ **DWA 硬/软代价分离**（只有 LETHAL 才判碰撞）；⑤ **地形层只给全局规划器**（局部规划器用滚动障碍窗口，避免"前进更贵→停住"的局部最优） |
| 实测（slope_test，15° 坡） | 坡度 14.5–16.1°、可通行性 0.36–0.42；**DWA 沿 A* 路径爬坡到平台**（机器人位姿 x≈5.19），平台内 1.07m 目标 12.7s 成功 |
| 已知代价 | 上坡很慢（轮式打滑，0.2–0.3 m/s 的爬坡实际只有 ~0.05–0.1 m/s）；一次性 0→6.4m 目标会超时，建议分段或提高爬坡速度 |
| 置信度门槛（新增） | 填充时若邻域高度跨度 > `fill_max_height_spread`（默认 **0.35m**）则不填、保持未知 → 上层回退静态图。修复"远距离平台被误填成地面 → `Goal is occupied`"：实测同目标 **0 次 occupied 警告** |
| 端到端实测 | 坡道分段任务 `[1.5, 0] → [3.5, 0] → [6.0, 0]`（`config/mission_slope.yaml`）：**SUCCEEDED 3/3，71.0s**（爬坡 + 穿平台） |
| 遗留 | ① 局部规划器不再感知坡度（只跟全局路径）；② 上坡仍慢（打滑，0.05–0.1 m/s），远目标建议分段；③ 跨台阶边缘的格保持未知（靠静态图兜底），所以平台"崖边"附近的可通行性仍不精确 |
| 备注 | DWA 的节流日志在部分运行里不打印（THROTTLE + sim time），属日志问题，以 action 结果/位姿为准 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型与链路已通，开环步态推进不可靠（待接 CHAMP 或换平台）

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`），gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`（cmd_vel → 足端轨迹 → 解析 IK，腿长 0.213/0.213）；`gait=walk`（duty 0.75）或 `trot`（duty 0.5） |
| ✅ 已验证 | 站立：`z≈0.19m, roll≈-4°`（关节跟踪站立位 thigh 0.79 / calf -1.56）；节点/话题/桥接/IK 全部正常 |
| ❌ 未达到 | **有效推进**：命令 0.1–0.15 m/s 时，机身在 5–6s 内只位移 0.1–0.2m（≈0.02–0.03 m/s），脚端明显打滑；加大步幅/增益后又会把机器人"踢翻"（roll≈180°） |
| 已排除 | IK 符号（URDF +y 旋转使足端向 -x，已改为 `atan2(-foot_x, r)`）、指令斜率（3–4 rad/s）、增益（p40/d4/cmd10 最稳，p200/cmd30 必翻）、出生高度（0.40→0.55） |
| 调参经验 | gz `JointPositionController`：`p_gain>60 + cmd_max>15` 起步必翻；`cmd_max<10` 或 `p_gain<30` 又托不住身体。开环 IK 步态对参数极敏感 |
| 转机（已采纳轮足方案） | 已接入 **Go2-W**（`go2w_description`，轮足）：腿保持站立位、4 个轮子差速驱动。`profile:=sim_go2w` 下 1m 目标成功（DWA）；**加速度闭环后 48.3s/1.80m → 4.96s/1.08m** |
| 轮足关键参数 | `body_height=0.34`（0.30 时轮子蹭地不转向）、`wheel_radius=0.057`（实测标定）、
`linear_scale=1.15` / `angular_scale=1.75`（补偿滑移）、tracker 用 **DWA** 且 `goal_tolerance_dist=0.25` |
| 足式步态建议 | 若要真足式行走：接 CHAMP（apt 已有 `gz_ros2_control`/`ros2_control`）或做接触/平衡闭环；开环 IK 步态在 gz 里推进不可靠（已实测） |
| 相关 | `profile:=sim_go2` 可跑通 nav→cmd_vel→关节→odom 链路，但行走精度不足，不建议作为导航性能实验平台 |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型/ode 链路已通，闭环与步态质量仍有限

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`）；gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`：cmd_vel → 足端轨迹 → **解析 IK**（Go2 腿长 0.213/0.213）；`gait=walk`（三足支撑，duty 0.75）或 `trot`（对角） |
| 实测（walk, 0.1 m/s） | 站立 roll≈±5°、base z≈0.19；走 6s 位移 **0.71m（≈0.12 m/s）**、roll +4°/pitch -11°（未翻车） |
| 实测（trot） | 关节呈对角相位（corr(FL,RR)=+0.53 vs corr(FL,FR)=+0.23），但开环**会翻车**（roll≈180°） |
| 调参经验 | gz 侧 `p_gain 200→40`、`d_gain 10→4`、`cmd_max 30→10` + 指令斜率限制（4 rad/s）后，站立/慢走才稳定 |
| 闭环（`profile:=sim_go2`） | `/odom`、`map→base`、action、cmd_vel→关节全部打通；但 nav 闭环下机器人会走偏（PP 假设差速瞬时转向，walk 转向慢），**路径跟踪精度待调** |
| 不支持 | 接触时序/平衡反馈（无 IMU/足端力闭环）、CHAMP/RL 步态、地形适应性 |
| 下一步 | ① 给运动层加 IMU 反馈/速度环；② 接 CHAMP（apt 有 `gz_ros2_control`）；③ 用 DWA 替代 PP（转向更平滑） |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足 demo 只是"链路验证"，不是可用的四足仿真

| 项 | 内容 |
|---|---|
| 现状 | 已接入 **Unitree 官方 `go2_description`**（BSD，vendored 在 `src/pnc_nav_sim/go2/`，含 dae 网格与 12 关节），
用我们的 gz `JointPositionController` + `FakeTrotLocomotion` 驱动；另保留极简盒子四足做对照 |
| 已验证 | `cmd_vel=0.3` → `/joint_states` 中 Go2 `FL_thigh_joint` 摆动 **-0.50 ~ +0.46 rad**；静态时关节≈0；模型在 gz 中正常加载（mesh 由 `package://` → `file://` 解析） |
| 不支持 | 真步态（无接触时序/落足，机器人会倒）、CHAMP/RL 集成、四足 odom（腿部里程计/IMU）、与导航栈闭环 |
| 下一步 | 接 CHAMP 或 RL 步态插件；补 odom；再考虑四足进 `bringup` profile |

### KI-019 地形：高程/坡度已修，DWA+地形仍拒绝上坡

| 项 | 内容 |
|---|---|
| 已修复 | ① 高程填充改**局部平面拟合** + clamp；② `fill_radius 0.8`、`robot_clear_radius 0.7`；③ A* `start_clear_radius 0.8m`；④ **DWA 硬/软代价分离**：只有 `cost >= LETHAL(254)` 才判轨迹非法，地形/膨胀等软代价只做惩罚 |
| 实测（slope_test，15° 坡） | 高程剖线连续，坡度 **14.5–16.1°**，可通行性 0.36–0.42；DWA 平地仍正常（2D 平地 2m → success 9.6s） |
| PP + 地形 | ✅ 上坡到平台（53.6s / 7.21m） |
| DWA + 地形 | ❌ 仍卡坡底（1.7m 处 stuck）；`obstacle_cost_bias` 0.3→0.05 也无效——不是权重问题，而是"前进轨迹的地形代价 > 停住的代价"的局部最优 |
| 下一步 | ① 给 DWA 增加**独立的地形代价项**（与障碍代价分离、权重可单独调小）；② 或坡道场景显式切 PP；③ 或换带地形代价的 MPC |
| 备注 | DWA 的节流日志（THROTTLE + sim time）在部分运行里不打印，属日志问题，不影响控制（以 action 结果与位姿为准） |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型与链路已通，开环步态推进不可靠（待接 CHAMP 或换平台）

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`），gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`（cmd_vel → 足端轨迹 → 解析 IK，腿长 0.213/0.213）；`gait=walk`（duty 0.75）或 `trot`（duty 0.5） |
| ✅ 已验证 | 站立：`z≈0.19m, roll≈-4°`（关节跟踪站立位 thigh 0.79 / calf -1.56）；节点/话题/桥接/IK 全部正常 |
| ❌ 未达到 | **有效推进**：命令 0.1–0.15 m/s 时，机身在 5–6s 内只位移 0.1–0.2m（≈0.02–0.03 m/s），脚端明显打滑；加大步幅/增益后又会把机器人"踢翻"（roll≈180°） |
| 已排除 | IK 符号（URDF +y 旋转使足端向 -x，已改为 `atan2(-foot_x, r)`）、指令斜率（3–4 rad/s）、增益（p40/d4/cmd10 最稳，p200/cmd30 必翻）、出生高度（0.40→0.55） |
| 调参经验 | gz `JointPositionController`：`p_gain>60 + cmd_max>15` 起步必翻；`cmd_max<10` 或 `p_gain<30` 又托不住身体。开环 IK 步态对参数极敏感 |
| 转机（已采纳轮足方案） | 已接入 **Go2-W**（`go2w_description`，轮足）：腿保持站立位、4 个轮子差速驱动。`profile:=sim_go2w` 下 1m 目标成功（DWA）；**加速度闭环后 48.3s/1.80m → 4.96s/1.08m** |
| 轮足关键参数 | `body_height=0.34`（0.30 时轮子蹭地不转向）、`wheel_radius=0.057`（实测标定）、
`linear_scale=1.15` / `angular_scale=1.75`（补偿滑移）、tracker 用 **DWA** 且 `goal_tolerance_dist=0.25` |
| 足式步态建议 | 若要真足式行走：接 CHAMP（apt 已有 `gz_ros2_control`/`ros2_control`）或做接触/平衡闭环；开环 IK 步态在 gz 里推进不可靠（已实测） |
| 相关 | `profile:=sim_go2` 可跑通 nav→cmd_vel→关节→odom 链路，但行走精度不足，不建议作为导航性能实验平台 |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型/ode 链路已通，闭环与步态质量仍有限

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`）；gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`：cmd_vel → 足端轨迹 → **解析 IK**（Go2 腿长 0.213/0.213）；`gait=walk`（三足支撑，duty 0.75）或 `trot`（对角） |
| 实测（walk, 0.1 m/s） | 站立 roll≈±5°、base z≈0.19；走 6s 位移 **0.71m（≈0.12 m/s）**、roll +4°/pitch -11°（未翻车） |
| 实测（trot） | 关节呈对角相位（corr(FL,RR)=+0.53 vs corr(FL,FR)=+0.23），但开环**会翻车**（roll≈180°） |
| 调参经验 | gz 侧 `p_gain 200→40`、`d_gain 10→4`、`cmd_max 30→10` + 指令斜率限制（4 rad/s）后，站立/慢走才稳定 |
| 闭环（`profile:=sim_go2`） | `/odom`、`map→base`、action、cmd_vel→关节全部打通；但 nav 闭环下机器人会走偏（PP 假设差速瞬时转向，walk 转向慢），**路径跟踪精度待调** |
| 不支持 | 接触时序/平衡反馈（无 IMU/足端力闭环）、CHAMP/RL 步态、地形适应性 |
| 下一步 | ① 给运动层加 IMU 反馈/速度环；② 接 CHAMP（apt 有 `gz_ros2_control`）；③ 用 DWA 替代 PP（转向更平滑） |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足 demo 只是"链路验证"，不是可用的四足仿真

| 项 | 内容 |
|---|---|
| 现状 | 已接入 **Unitree 官方 `go2_description`**（BSD，vendored 在 `src/pnc_nav_sim/go2/`，含 dae 网格与 12 关节），
用我们的 gz `JointPositionController` + `FakeTrotLocomotion` 驱动；另保留极简盒子四足做对照 |
| 已验证 | `cmd_vel=0.3` → `/joint_states` 中 Go2 `FL_thigh_joint` 摆动 **-0.50 ~ +0.46 rad**；静态时关节≈0；模型在 gz 中正常加载（mesh 由 `package://` → `file://` 解析） |
| 不支持 | 真步态（无接触时序/落足，机器人会倒）、CHAMP/RL 集成、四足 odom（腿部里程计/IMU）、与导航栈闭环 |
| 下一步 | 接 CHAMP 或 RL 步态插件；补 odom；再考虑四足进 `bringup` profile |

### KI-019 地形：高程/坡度已修，DWA+地形仍拒绝上坡

| 项 | 内容 |
|---|---|
| 已修复 | ① 高程填充改**局部平面拟合**（≥3 邻域、RMS<0.05m）+ clamp；② `fill_radius 0.8`、`robot_clear_radius 0.7`；③ A* 加 `start_clear_radius 0.8m`（起点附近不判阻挡） |
| 实测（slope_test，15° 实心坡） | 高程剖线连续（x=2.0/2.5/3.0/3.5 → 0.06/0.23/0.35/0.49），坡度 **14.5–16.1°**，可通行性 **0.36–0.42**（>默认 lethal 0.3） |
| PP（PurePursuit2D）+ 地形 | ✅ 能上坡并抵达平台（53.6s / 7.21m），说明高程/代价链路是通的 |
| DWA（DWA2D）+ 地形 | ❌ 仍卡在坡底（`stuck`，0.7–1.6m；把 `elevation_lethal_score` 降到 0.2 无效；日志里甚至没有 DWA 的周期打印，待查是否走了不同分支） |
| 判断 | 地形数据与全局规划没问题，问题在**局部规划器对地形代价的适配**（DWA 的代价/碰撞判定对坡面不友好） |
| 下一步 | ① 给 DWA 的 `obstacleCost` 区分"硬障碍(LETHAL)"与"地形代价"（地形只做软惩罚，不参与是否合法的判定）；② 或换带地形代价的控制器（MPC/TEB）；③ 或坡道场景继续用 PP |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型与链路已通，开环步态推进不可靠（待接 CHAMP 或换平台）

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`），gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`（cmd_vel → 足端轨迹 → 解析 IK，腿长 0.213/0.213）；`gait=walk`（duty 0.75）或 `trot`（duty 0.5） |
| ✅ 已验证 | 站立：`z≈0.19m, roll≈-4°`（关节跟踪站立位 thigh 0.79 / calf -1.56）；节点/话题/桥接/IK 全部正常 |
| ❌ 未达到 | **有效推进**：命令 0.1–0.15 m/s 时，机身在 5–6s 内只位移 0.1–0.2m（≈0.02–0.03 m/s），脚端明显打滑；加大步幅/增益后又会把机器人"踢翻"（roll≈180°） |
| 已排除 | IK 符号（URDF +y 旋转使足端向 -x，已改为 `atan2(-foot_x, r)`）、指令斜率（3–4 rad/s）、增益（p40/d4/cmd10 最稳，p200/cmd30 必翻）、出生高度（0.40→0.55） |
| 调参经验 | gz `JointPositionController`：`p_gain>60 + cmd_max>15` 起步必翻；`cmd_max<10` 或 `p_gain<30` 又托不住身体。开环 IK 步态对参数极敏感 |
| 转机（已采纳轮足方案） | 已接入 **Go2-W**（`go2w_description`，轮足）：腿保持站立位、4 个轮子差速驱动。`profile:=sim_go2w` 下 1m 目标成功（DWA）；**加速度闭环后 48.3s/1.80m → 4.96s/1.08m** |
| 轮足关键参数 | `body_height=0.34`（0.30 时轮子蹭地不转向）、`wheel_radius=0.057`（实测标定）、
`linear_scale=1.15` / `angular_scale=1.75`（补偿滑移）、tracker 用 **DWA** 且 `goal_tolerance_dist=0.25` |
| 足式步态建议 | 若要真足式行走：接 CHAMP（apt 已有 `gz_ros2_control`/`ros2_control`）或做接触/平衡闭环；开环 IK 步态在 gz 里推进不可靠（已实测） |
| 相关 | `profile:=sim_go2` 可跑通 nav→cmd_vel→关节→odom 链路，但行走精度不足，不建议作为导航性能实验平台 |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足：模型/ode 链路已通，闭环与步态质量仍有限

| 项 | 内容 |
|---|---|
| 模型 | **Unitree 官方 `go2_description`**（BSD，vendored `src/pnc_nav_sim/go2/`）；gz 关节由 `JointPositionController` 驱动 |
| 步态 | `TrotGaitLocomotion`：cmd_vel → 足端轨迹 → **解析 IK**（Go2 腿长 0.213/0.213）；`gait=walk`（三足支撑，duty 0.75）或 `trot`（对角） |
| 实测（walk, 0.1 m/s） | 站立 roll≈±5°、base z≈0.19；走 6s 位移 **0.71m（≈0.12 m/s）**、roll +4°/pitch -11°（未翻车） |
| 实测（trot） | 关节呈对角相位（corr(FL,RR)=+0.53 vs corr(FL,FR)=+0.23），但开环**会翻车**（roll≈180°） |
| 调参经验 | gz 侧 `p_gain 200→40`、`d_gain 10→4`、`cmd_max 30→10` + 指令斜率限制（4 rad/s）后，站立/慢走才稳定 |
| 闭环（`profile:=sim_go2`） | `/odom`、`map→base`、action、cmd_vel→关节全部打通；但 nav 闭环下机器人会走偏（PP 假设差速瞬时转向，walk 转向慢），**路径跟踪精度待调** |
| 不支持 | 接触时序/平衡反馈（无 IMU/足端力闭环）、CHAMP/RL 步态、地形适应性 |
| 下一步 | ① 给运动层加 IMU 反馈/速度环；② 接 CHAMP（apt 有 `gz_ros2_control`）；③ 用 DWA 替代 PP（转向更平滑） |

### KI-019 坡面高程空洞 → 假台阶（DWA+地形 拒绝上坡）

| 项 | 内容 |
|---|---|
| 现象 | 15° 斜坡剖线：x=2.5 高程 0.02（地面）、x=3.0 高程 0.35（坡面），相邻格差 0.33m → 台阶分项 0 → 可通行性 0.17–0.5；DWA + 地形开启后卡在坡底 x≈1.1，`error_code=3`（stuck） |
| 根因 | 16 线激光从坡底看坡面是**掠射角**，坡面点稀疏；`fillUnknown` 用"邻域最低 z"填洞 → 坡面空洞被地面高度填上，和真实坡面格形成假台阶 |
| 影响 | PP（不看代价地图）仍能上坡（53.6s/7.21m 抵达平台）；DWA + 地形则不敢上坡 |
| 方向 | ① 填充改成"沿表面传播"（只在与邻格高差 < 阈值时扩散），而不是全局取最低 z；② 地面分割（局部平面拟合/RANSAC）后再算坡度与台阶；③ 高程分辨率换 0.2m + 累积多帧；④ 短期可把 `elevation_lethal_score` 调到 0.15 或 `step_max_m` 放大 |
| 备注 | 这是 2.5D + 稀疏激光的固有问题；真正解决要走"表面重建/3D 体素"路线 |

### KI-020 短距离折返时 PP 跟踪跑偏 ✅ 已修复（2026-10-23）

| 项 | 内容 |
|---|---|
| 现象 | 三点任务 `[1,0] → [2,0] → [0.5,0]`：第三点折返后机器人一路开到 (3.9, -11.2)，77s 仍在 following |
| 根因 | ① `findLookaheadPoint` 的索引只增不减，折返时停在路径起点/终点，**追着身后的点**；② 更关键：前视点在正后方时 `heading_error≈π`，曲率公式 `2·sin(δ)/L` 里 **sin(π)≈0 → 曲率 0 → 直着开走** |
| 修复 | ① 前视索引**单调推进**（下一个点更近就前进）；找不到前视点时用末点，不再刷 "No lookahead point"；② **大角度偏差（>2.0 rad）先原地旋转**，不做曲率计算；③ 顺带修 KI-009（`debug` 参数加 `<实例名>.` 前缀） |
| 回归单测 | 新增 `pnc_nav_planners/test/test_pure_pursuit_2d.cpp`：正向路径要前进；**路径在身后时必须原地转（vx=0, |ω|>0.1）** |
| 实测 | 三点任务 `state=SUCCEEDED (3/3)`；折返段首版 50.7s / 2.77m |
| 提速（同日） | ① 旋转阈值参数化 `rotate_threshold=0.6`（转到位再走，避免走走正正）；② 比例旋转 `rotate_gain=2.5`；③ `max_angular_vel` 0.6→1.0、`max_linear_vel` 0.25→0.30；④ 小角度修正时按 `cos(heading_error)` 降速。**折返段 50.7s → 19.9s，三点任务 69s → 32.9s** |

### KI-021 四足 demo 只是"链路验证"，不是可用的四足仿真

| 项 | 内容 |
|---|---|
| 现状 | 已接入 **Unitree 官方 `go2_description`**（BSD，vendored 在 `src/pnc_nav_sim/go2/`，含 dae 网格与 12 关节），
用我们的 gz `JointPositionController` + `FakeTrotLocomotion` 驱动；另保留极简盒子四足做对照 |
| 已验证 | `cmd_vel=0.3` → `/joint_states` 中 Go2 `FL_thigh_joint` 摆动 **-0.50 ~ +0.46 rad**；静态时关节≈0；模型在 gz 中正常加载（mesh 由 `package://` → `file://` 解析） |
| 不支持 | 真步态（无接触时序/落足，机器人会倒）、CHAMP/RL 集成、四足 odom（腿部里程计/IMU）、与导航栈闭环 |
| 下一步 | 接 CHAMP 或 RL 步态插件；补 odom；再考虑四足进 `bringup` profile |

### KI-019 坡面空洞假台阶（地形质量）🟡 已缓解，DWA 上坡仍受限

| 项 | 内容 |
|---|---|
| 已修复 | ① 高程填充改为**局部平面拟合**（≥3 邻域、RMS<0.05m 用平面值，否则退化为最低 z），并 clamp 到邻域 [min,max]；② 填充半径 0.4→0.8（覆盖 robot_clear_radius）；③ A* 增加"起点清除半径"（0.8m 内不判阻挡） |
| 实测（slope_test，15° 实心坡） | 高程剖线连续：x=2.0/2.5/3.0/3.5 高程 0.06/0.23/0.35/0.49，坡度 14.5–16.1°，可通行性 0.36–0.42（> lethal 0.3） |
| 仍受限 | DWA + 地形上坡仍会卡在坡底（`stuck`，50.4s / 1.59m）——地形代价让"前进"相对"停住"没有优势，属于局部规划器对代价地形的适配问题 |
| 下一步 | 给 DWA 加"地形难度加权的前进激励"或降低 lethal_score；或改用带地形代价的 MPC/TEB |



### KI-101 `AStar2D::cleanup()` 段错误（commit `ed5280c`）

```cpp
// 修复前：热插拔（action 带 planner_id）必崩，exit code -11
void AStar2D::cleanup() {
  node_.reset();
  costmap_.reset();
  RCLCPP_INFO(node_->get_logger(), "AStar2D cleaned up");   // ← 空指针
}
```

**教训**：`cleanup()` 里的日志必须写在 `reset()` 之前；插件契约已加"内存所有权"一节（`design/plugin_contract.md` §2.6）。

### KI-102 热插拔时重复加载同一插件（commit `ed5280c`）

`switchGlobalPlanner()` 原来用 `getName()` 字符串和插件全名比较（永远不相等），导致每次发带
`planner_id` 的目标都会卸载+重载插件。改为记录 `current_global_plugin_` 类名做短路。

### KI-103 搜索插件名写错时进程直接退出（commit `ed5280c` 之前）

类名不存在时 pluginlib 抛异常，`NavServer` 未捕获 → 进程退出。现在 `startGoal()` 捕获并让该 goal
以 `error_code=6` 失败（仍会清掉旧插件，需重发）。

---
### KI-022 坡道速度上不去：调 DWA 限速和轮子摩擦都无效（牵引/物理限制）

| 项 | 内容 |
|---|---|
| 现象 | 15° 坡道（`slope_test.sdf`）上速度约 0.05–0.06 m/s，2.5m 爬升要 40s |
| 试过 | ① DWA `max_vel_x 0.2→0.35`、`max_vel_theta 0.8→1.2`、`acc_lim_x 1.0→1.5`；② 驱动轮 `<mu1/mu2> 1.0→2.0</mu1>`、支撑轮 0.2 |
| 结果 | 三段任务总耗时 **71s → 73.4s / 76.3s**（无收益，甚至在噪声内变差） |
| 结论 | gz 的 `DiffDrive` 是速度指令直驱，打滑来自接触模型与坡面法向/切向摩擦，不是控制限速；继续调参数收益极低 |
| 后续 | 真想加速：换轮足/更大轮径、加 `<kp>` 接触刚度实验、或把坡道任务的分段目标改稀（少一次停走） |

### KI-023 LIO-SAM 接入：环境坑多、回环关闭、地图与静态图不一致

| 项 | 内容 |
|---|---|
| ✅ 已验证 | 仿真直行 0.75m：`map→base_link` 与 gz 真值误差 **0.01 m**；导航到点 (1.5,0) 成功 4.69s / 0.65m |
| 环境坑 | `libgtsam-dev` 缺 `CppUnitLite.a`；apt 无 `gtsam_unstable`；GTSAM 自带 Eigen 与系统 3.4.0 冲突；缺 `libmetis-dev`；apt 的 GTSAM cmake config 抢匹配 → 见 `scripts/setup_lio_sam.sh`、`docs/guides/lio_sam_setup.md` |
| 改造 | 上游发 `odom→lidar_link` + `odom→base_link`，与本框架/URDF 冲突 → 改为 **只发 `map→base_link`**（`lidarToBase` 外参），并关闭 IMU 预积分 TF |
| 限制 1 | 回环关闭（`loopClosureEnableFlag=false`），长距离会累积漂移 |
| 限制 2 | LIO-SAM 的世界原点是启动位置，与 `maps/111` 静态图不重合 → 做长任务前要么用对应场景图，要么后续接建图/保存 |
| 限制 3 | `third_party/LIO-SAM` 增加了 `lio_sam` 包，编译依赖 `/opt/gtsam_sys`（`CMAKE_PREFIX_PATH`），未装时 **不要** 全量 `colcon build`（会在 lio_sam 处失败） |

### KI-024 轮足 + 地形：坡上目标点被判 "Goal is occupied"

| 项 | 内容 |
|---|---|
| 背景 | 给 Go2-W 加了 16 线激光 + IMU（`go2w.urdf`）和 `sim_go2w_terrain` profile（点云障碍层 + 高程图 + DWA） |
| 现象 | 坡道任务 3 段：第 1 段 (1.5,0) 用了 **77.7 s**（平地，慢得异常）；第 2 段 (3.5,0) 连续两次 3.3 s 失败，日志 `Goal is occupied` |
| 已知 | 3D 差速车同样的坡道任务能过（71 s 三段），所以不是地形层本身不可用 |
| 怀疑 | ① 地形代价对目标格判 LETHAL 的门槛在轮足 footprint / 高度上不适用（`elevation_lethal_score`）；② 轮足 base 高 0.34m + 雷达 0.30m，高程滤波的 `z` 门限会切到坡面；③ 目标点正好落在坡度最大处 |
| 下一步 | 给地形层加"目标/起点附近 footprint 清除"（差速车 KI-017 的同类修复），并核对 `elevation_lethal_score`；再测一次耗时 |
| 顺便 | 第 1 段 77.7s 也要查：是 DWA 限速 + 地形惩罚叠加，还是轮足爬坡打滑 |


## 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：KI-001…KI-012 + 已修复 3 条 |
| 2026-10-23 | 新增局部滚动代价地图：KI-001/002/003 修复、KI-005 部分修复；新增 KI-013（DWA 只转不走）、KI-014（雷达 240° 盲区） |
| 2026-10-23 | KI-013 修复：路径代价改折线投影、目标代价改剩余弧长、朝向改前视切线；DWA 绕障实测通过 |
| 2026-10-23 | KI-004 修复：到达判定统一为距离优先、终点朝向可选（默认关闭）；KI-011 澄清 |
| 2026-10-23 | 局部代价地图增加 raytrace 清除（含超量程光束）：障碍消失 ~0.9s 内清除，不再等 TTL |
| 2026-10-23 | KI-012 修复：恢复旋转前做碰撞检查（半径可配），贴身时跳过旋转快速失败 |
| 2026-10-23 | 新增点云障碍层（3D 激光 2.5D 投影）与 3D bringup；新增 KI-015（非真 3D 规划） |
| 2026-10-23 | 新增 KI-016（3D 激光看不到运行期 spawn 的模型，撤回 3D 绕障结论）、KI-017（地形代价开启后 A* 规划失败） |
| 2026-10-23 | KI-017 修复（footprint 清除 + A* 起点容错）；新增 KI-018（坡度估计噪声）；slope_test 世界 + world 参数 |
| 2026-10-23 | KI-018 坡度幅值修复（宽基线 2 格 + 平滑，27–34°→13–16°）；新增 KI-019（坡面空洞假台阶，DWA+地形拒绝上坡） |
| 2026-10-23 | 新增任务层与 `/nav_status`、`/mission_status`；新增 KI-020（短距离折返 PP 跑偏） |
| 2026-10-23 | KI-020 修复（前视索引单调推进 + 大角度原地转）并加回归单测；KI-009 修复 |
| 2026-10-23 | 地形质量：平面拟合填充 + 起点清除（KI-019 缓解）；新增 KI-021（四足 demo 只是链路验证） |
| 2026-10-23 | KI-021 更新：已换用官方 go2_description 模型（仍无真步态/odom） |
| 2026-10-23 | 四足：IK walk/trot 步态 + odom + sim_go2 闭环；walk 可站立/慢走，闭环精度与 trot 稳定性待调 |
| 2026-10-23 | 修正 IK 符号与增益后：站立稳定（z 0.19/roll -4°），但开环步态推进仍不可靠 → 建议接 CHAMP 或换轮足 |
| 2026-10-23 | 采纳轮足方案：Go2-W 模型 + 轮速控制 + 标定 + DWA，`sim_go2w` 1m 目标成功 |
| 2026-10-23 | 轮足加速度闭环（/odom 反馈补偿）：直行 0.27/0.30，转向 0.60/0.60；KI-019 更新结论 |
| 2026-10-23 | DWA 硬/软代价分离（LETHAL 才算碰撞）；坡道仍卡 → 需要独立地形代价项 |
| 2026-10-23 | 地形层改为只作用于全局规划：DWA 能沿路径上坡到平台（坡上很慢） |
| 2026-10-23 | 高程填充加置信度门槛（邻域高差 >0.35m 不填）；坡道分段任务 3/3 成功 71s |

| 2026-11-04 | 轮足/坡道调参：DWA max_vel_x 0.2→0.35、驱动轮 mu 1.0→2.0，坡道任务 71s→73s（无收益）→ 新增 KI-022 |
| 2026-11-04 | 3D 定位：LIO-SAM 编译打通并在仿真里跑通（TF 误差 0.01m、导航到点 4.7s）→ 新增 KI-023、guides/lio_sam_setup.md |
| 2026-11-04 | LIO-SAM 长距离漂移实测：3 圈 30.7m，ATE 0.039m（关回环）/0.038m（开回环）；发现 /odom 不能当真值；新增 KI-024 |
