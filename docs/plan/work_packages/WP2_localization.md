# WP2 定位集成（profile 化）

> 负责人：定位 agent ｜ 状态：已实现（sim2d_amcl / sim2d_cartographer / real+amcl）
> 依赖：`nav2_amcl`（apt）、`amcl_localization.launch.py`（已有，未接入 profile）

## 目标
把"`map→odom` 由谁提供"变成 profile 的一个选项：仿真静态 TF（默认）/ AMCL / Cartographer，
让 `real` profile 真正可用。

## 交付物
- `bringup.launch.py` 新增 `localization` 参数：`static | amcl | cartographer`
- profile 组合：
  | profile | localization 默认 |
  |---|---|
  | `sim2d` | static |
  | `sim3d` / `sim3d_elevation` | static |
  | `sim2d_amcl`（新增） | amcl |
  | `real` | amcl（外部提供 scan） |
- 文档：runbook 增加组合表；`design/frames_topics.md` 补充"谁发布 map→odom"

## 接口契约
| 项 | 约定 |
|---|---|
| `map→odom` | `static`：`static_transform_publisher` 恒等；`amcl`：`nav2_amcl` 发布 |
| 依赖输入 | AMCL 需要 `/map`（map_server）+ `/scan`（2D 激光）+ `initialpose` |
| 生命周期 | AMCL 由 `lifecycle_manager` 管理（与 map_server 同一或独立 manager） |
| 冲突处理 | `localization=amcl` 时**不得**再起 static `map→odom`（否则两棵树打架，KI-008 同类问题） |

## 验收
1. `profile:=sim2d_amcl` 启动后 `ros2 run tf2_ros tf2_echo map base_link` 有输出
2. `ros2 topic echo /amcl_pose --once` 有位姿
3. 发 2m 目标仍成功（规划/跟踪链路不变）
4. `localization=static` 行为与现在完全一致（回归）

## 不做
- Cartographer 实跑（依赖 absl/protobuf，当前仓库环境未装，见 KI-007）；只留配置与开关
- 3D 定位（LIO-SAM）
