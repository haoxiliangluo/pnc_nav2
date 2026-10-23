# PNC Nav2 文档

> 面向三维导航的规划与控制框架（学习 / 教学项目）：插件化、可仿真、可对比。
> 本目录是文档的**唯一入口**，本页同时给出"从零上手"和"快速读懂"两条路线。

---

## 一、30 秒速览

```bash
# 1) 构建（只构建导航相关包，third_party/cartographer 需要 abseil，默认跳过）
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_sim pnc_nav_bringup

# 2) 起仿真 + 地图 + 导航服务器
source install/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py

# 3) 发一个目标（action 入口）
ros2 action send_goal /navigate_to_3d_pose pnc_nav_interfaces/action/NavigateTo3DPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

---

## 二、两条阅读路线

### 路线 A：从零上手（教学）

按顺序读，每篇都有"检查点"和可复制命令：

1. [tutorials/01_quickstart.md](tutorials/01_quickstart.md) —— 30 分钟跑起来，发第一个目标
2. [tutorials/02_first_plugin.md](tutorials/02_first_plugin.md) —— 写第一个规划器插件并热插拔 ✅
3. `tutorials/03_how_a_cycle_runs.md` —— 走读一次控制循环（🚧 计划中）
4. `tutorials/04_maps_and_worlds.md` —— 换地图 / 改世界 / 加传感器（🚧 计划中）
5. `tutorials/05_evaluate.md` —— 用评估脚本做算法对比（🚧 计划中）
6. [tutorials/06_troubleshooting.md](tutorials/06_troubleshooting.md) —— 真实踩坑合集（🚧 计划中，先看 [guides/sim_setup.md](guides/sim_setup.md) 的坑位章节）

### 路线 B：快速读懂（自用 / 面试 / 评审）

想看"架构为什么这么设计、和官方 Nav2 差在哪、代价是什么"，读这几篇（按重要性）：

1. [design/nav_server.md](design/nav_server.md) —— 状态机、action 契约、错误码、取消/抢占时序 ✅
2. [notes/vs_nav2.md](notes/vs_nav2.md) —— 与官方 Nav2 的逐项对照与取舍 ✅
3. [notes/decisions.md](notes/decisions.md) —— 关键决策记录 ADR ✅
4. [design/plugin_contract.md](design/plugin_contract.md) —— 插件契约（线程模型 / 调用顺序 / 参数命名约定）✅
5. [design/implementation_status.md](design/implementation_status.md) —— 设计 vs 现状矩阵 ✅

---

## 三、文档地图

| 文档 | 回答什么问题 | 状态 |
|---|---|---|
| [design/nav_server.md](design/nav_server.md) | 导航服务器怎么运转？action/feedback/result 语义？错误码？取消和抢占谁先谁后？ | ✅ |
| [design/plugin_contract.md](design/plugin_contract.md) | 怎么写一个插件？调用顺序、线程模型、单位与坐标系约定？ | ✅ |
| `design/map_and_costmap.md` | `CostmapInterface` 的边界语义？膨胀/未知怎么处理？ | 🚧 |
| [design/frames_topics.md](design/frames_topics.md) | TF 树长什么样？有哪些话题、类型、方向、频率？ | ✅ |
| [design/implementation_status.md](design/implementation_status.md) | 设计与代码的差距在哪？哪些是愿景？ | ✅ |
| [tutorials/01_quickstart.md](tutorials/01_quickstart.md) | 怎么在 30 分钟内跑通并发出第一个目标？ | ✅ |
| [tutorials/02_first_plugin.md](tutorials/02_first_plugin.md) | 怎么写第一个规划器插件、怎么热插拔？ | ✅ |
| [guides/runbook.md](guides/runbook.md) | 启动矩阵、参数手册、调参建议 | ✅ |
| [guides/sim_setup.md](guides/sim_setup.md) | 仿真栈怎么搭？`ROS_DOMAIN_ID`/`GZ_PARTITION` 为什么重要？ | 🚧 |
| [notes/vs_nav2.md](notes/vs_nav2.md) | 和 Nav2 的取舍对照（面试高频） | ✅ |
| [notes/decisions.md](notes/decisions.md) | 每个关键决策的理由与代价 | ✅ |
| [notes/known_issues.md](notes/known_issues.md) | 已知缺陷与下一步 | ✅ |
| `plan/phase3_3d_design.md` | 3D 链路（OctoMap / 高程图 / Traversability）怎么落地 | 🚧 |

---

## 四、其它文档在哪

| 文件 | 定位 |
|---|---|
| [../README.md](../README.md) | 项目门面：特性、构建、运行、包结构、路线 |
| [../ARCHITECTURE.md](../ARCHITECTURE.md) | **目标架构（愿景）**：包含尚未实现的模块（`pnc_nav_map`、`pnc_nav_localization`、`pnc_nav_control`） |
| [../NAV2_LEARNING_AND_IMPLEMENTATION_ROADMAP.md](../NAV2_LEARNING_AND_IMPLEMENTATION_ROADMAP.md) | 学习路线与阶段计划 |
| [phase1_验证清单.md](phase1_验证清单.md) | 阶段 1（2D 闭环）验证记录 |
| [phase2_验证清单.md](phase2_验证清单.md) | 阶段 2（Action + 恢复闭环）验证记录 |
| [../archive/](../archive) | 历史分析（simdog 选包、旧代码评审），仅存档 |

> ⚠️ `ARCHITECTURE.md` 描述的是**目标形态**，当前实现的契约请看 `docs/design/`。

---

## 五、文档写作约定

为了让"教学 / 自用 / 面试"三种用途共用一套文档：

1. **三层结构**：概念（给人看）→ 契约（给代码看）→ 取舍（给评审看）
2. **事实唯一来源**：接口语义只写在 `design/`，教程里只引用不复制
3. **可验证**：每个操作步骤都要写"你应该看到什么"（命令 + 预期输出）
4. **版本锚点**：每篇开头写"最后核对：commit `xxxxxxx`"
5. **文档与代码同 PR**：改接口必须同步改文档
6. **图用 Mermaid**：状态机、时序、TF 树优先画图
7. **只写踩过的坑**：现象 → 原因 → 修复三段式
