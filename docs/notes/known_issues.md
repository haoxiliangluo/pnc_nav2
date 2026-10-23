# 已知问题（Known Issues）

> 最后核对：commit `216e21d`（2026-10-23）
> 用法：改代码前先扫一眼；修完把条目移到文末"已修复（留档）"，别删——它是踩坑记录。

---

## 未修复

### KI-001 DWA2D 到点停短（约 0.25 m 就停）

| 项 | 内容 |
|---|---|
| 现象 | 用 `DWA2D` 时机器人走到离目标 ~0.25–0.30 m 处停止，`isGoalReached` 一直为 false，触发卡住检测 → 恢复 → 死循环，最终 `error_code=3` |
| 证据 | `DWA2D: pose=(2.17,0.19) goal=(0.1,0.2) dist=0.05` + `与目标距离差未达到`（每周期刷屏）；feedback `dist=0.30m` 不变 |
| 根因（推测） | ① DWA 内部"局部目标点"与最终 goal 不是同一个点；② 目标点落在膨胀层内，DWA 不敢靠近；③ 到达判定用 `dist+angle` 双阈值，角度不满足时永远不判成功 |
| 影响 | 默认配置已切回 `PurePursuit2D`，不影响 baseline；但"局部避障"这条线被卡住 |
| 临时规避 | 用 `PurePursuit2D`；或把 `goal_tolerance_dist` 调大（不推荐，会掩盖问题） |
| 修复方向 | 统一到达判定（见 KI-004）；把"目标不可达"与"到达"区分开；给 DWA 内部目标点加日志验证 |

### KI-002 DWA2D 静态代价溢出

| 项 | 内容 |
|---|---|
| 现象 | 日志里出现 `static=179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000` |
| 含义 | 即 `std::numeric_limits<double>::max()`——静态代价分支返回了"无穷大" |
| 影响 | 轨迹评分被污染，选出的速度可能不合理（也是 KI-001 的可能诱因） |
| 修复方向 | 检查 `evaluateTrajectory` 里 static cost 的初值/归一化；用 `std::isfinite` 断言 + 单测钉住 |

### KI-003 DWA2D 每周期打 INFO，日志刷屏

| 项 | 内容 |
|---|---|
| 现象 | 20Hz 下每周期输出 `DWA2D: cmd=...` 与 `与目标距离差未达到` |
| 影响 | 日志文件暴涨、真实错误被淹没、终端不可读 |
| 修复方向 | 降为 `RCLCPP_DEBUG` 或 `RCLCPP_INFO_THROTTLE(..., 2000, ...)`；由参数 `local_planner.debug` 控制 |

### KI-004 `distance_remaining` 与到达判定语义不一致

| 项 | 内容 |
|---|---|
| 现象 | feedback 显示 `dist=0.20 m`（到目标点直线距离），但 result 已经是 `success=true`；反过来也可能 `dist=0.10` 却判定失败 |
| 原因 | feedback 用的是"到 goal 的欧氏距离"；到达判定由插件决定：`PurePursuit2D` 用"到路径末点距离"，`DWA2D` 用"距离+角度" |
| 影响 | 评估脚本按 feedback 统计会出现"到达误差"为负/异常值；使用者困惑 |
| 修复方向 | 把到达判定统一到 NavServer（用一个 `GoalChecker` 语义），插件只负责输出速度；或统一 feedback 也按"到路径末点"计算 |

### KI-005 恢复行为不清代价地图

| 项 | 内容 |
|---|---|
| 现象 | `RECOVERING` 只做原地旋转 + 重规划 |
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

### KI-011 `goal_tolerance_angle` 只有部分插件使用

| 项 | 内容 |
|---|---|
| 现象 | 该参数传给插件，但 `PurePursuit2D` 只按距离判定，`DWA2D` 才用角度 |
| 影响 | 调参时"角度容差不生效"，容易误判 |
| 修复方向 | 与 KI-004 一起统一到 GoalChecker 语义 |

### KI-012 恢复期的原地旋转不做碰撞检查

| 项 | 内容 |
|---|---|
| 现象 | `RECOVERING` 直接发 `angular.z`，不看代价地图 |
| 影响 | 仿真无所谓；真机贴着障碍时旋转可能碰撞 |
| 修复方向 | 旋转前用 `costmap_->isOccupied()` 检查机器人 footprint 周边；不安全则改为后退或直接失败 |

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
