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

### KI-009 `PurePursuit2D` 的 `debug` 参数缺前缀

| 项 | 内容 |
|---|---|
| 现象 | 代码里 `node_->declare_parameter("debug", false)`，没有 `<name>.` 前缀 |
| 影响 | 与其它实例/节点参数可能串味；违背插件参数命名约定 |
| 修复方向 | 改为 `name_ + ".debug"`（注意同步文档和 yaml） |

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

---

## 已修复（留档）

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

## 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：KI-001…KI-012 + 已修复 3 条 |
| 2026-10-23 | 新增局部滚动代价地图：KI-001/002/003 修复、KI-005 部分修复；新增 KI-013（DWA 只转不走）、KI-014（雷达 240° 盲区） |
| 2026-10-23 | KI-013 修复：路径代价改折线投影、目标代价改剩余弧长、朝向改前视切线；DWA 绕障实测通过 |
| 2026-10-23 | KI-004 修复：到达判定统一为距离优先、终点朝向可选（默认关闭）；KI-011 澄清 |
| 2026-10-23 | 局部代价地图增加 raytrace 清除（含超量程光束）：障碍消失 ~0.9s 内清除，不再等 TTL |
| 2026-10-23 | KI-012 修复：恢复旋转前做碰撞检查（半径可配），贴身时跳过旋转快速失败 |
