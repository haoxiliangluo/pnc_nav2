# 任务层（Mission Layer）设计与契约

> 最后核对：本次提交（2026-10-23）
> 代码：`src/pnc_nav_task/`（`mission_runner` 纯逻辑 + `mission_executor_node` 收发）
> 相关：[nav_server.md](nav_server.md)（动作/错误码）、[../plan/phase3_3d_design.md](../plan/phase3_3d_design.md)

---

## 1. 为什么要有这一层

导航层只回答"从 A 到 B 怎么走"；**"按顺序去 3 个点、失败重试、随时暂停/取消"** 属于任务层。
把任务逻辑单独成层的好处：

- 导航层保持简单（一次只服务一个 goal，新目标抢占旧目标）
- 任务策略（重试、循环、暂停）可以**单元测试**（`mission_runner` 不依赖 ROS）
- 以后换任务形态（行为树/多机器人调度）只改这一层

```
mission_executor ──(action)──▶ nav_server ──▶ 规划/控制
      │                            │
      └── /mission_status          └── /nav_status
```

---

## 2. 接口

### 2.1 输入

| 接口 | 类型 | 说明 |
|---|---|---|
| 参数 `mission.waypoints` | double[] | 扁平数组 `[x1, y1, x2, y2, ...]`（frame 由 `mission.frame_id` 决定，默认 `map`） |
| 参数 `mission.max_retries` | int | 每个点失败后重试次数（总尝试 = 1 + max_retries，默认 1 → 最多 2 次） |
| 参数 `mission.loop` | bool | 跑完是否从头再来（默认 false） |
| 参数 `mission.auto_start` | bool | 启动即开始（默认 false） |
| 服务 `mission/start` | `std_srvs/Trigger` | 从头开始；空 waypoints → 直接 FAILED |
| 服务 `mission/pause` | `std_srvs/Trigger` | 暂停：取消当前 goal，状态 PAUSED |
| 服务 `mission/resume` | `std_srvs/Trigger` | 继续（重发当前点） |
| 服务 `mission/cancel` | `std_srvs/Trigger` | 取消整个任务 |

### 2.2 输出

| 接口 | 类型 | 说明 |
|---|---|---|
| `/mission_status` | `pnc_nav_interfaces/MissionStatus` | 状态、当前序号/总数、重试次数、当前点位、message；2Hz + 状态变化即发 |
| action `navigate_to_3d_pose` | `NavigateTo3DPose` | 对导航层的调用（`planner_id` 空、`max_planning_time` 0） |

导航侧另有 `/nav_status`（`NavStatus`：state / error_code / 剩余距离 / 恢复次数 / 耗时），
方便 HMI/监控不解析 action。

---

## 3. 状态机

```
        start()                 最后一颗点成功
IDLE ──────────▶ RUNNING ─────────────────▶ SUCCEEDED
  ▲                 │  ▲                          │
  │      pause()    │  │  resume()                │ start()
  │                 ▼  │                          │
  │              PAUSED ┘                          │
  │                 │                              │
  └─────────────────┴──── cancel() ──▶ CANCELLED  │
                                     ◀─────────────┘
RUNNING ── 所有重试用尽 ──▶ FAILED
```

| 事件 | 行为 |
|---|---|
| 目标成功 | `retries=0`；还有下一个点 → `index++`；没有 → `SUCCEEDED`（`loop=true` 时回到 0） |
| 目标失败 | `retries < max_retries` → `retries++` 重发同一个点；否则 `FAILED` |
| 暂停 | 取消当前 goal；**暂停期间返回的结果不推进任务** |
| 取消 | 取消当前 goal；状态 `CANCELLED`，后续结果全部忽略 |
| 空 waypoints | `start()` 直接 `FAILED`（`no waypoints configured`） |

---

## 4. 实现要点

- **纯逻辑分离**：`MissionRunner`（`include/pnc_nav_task/mission_runner.hpp`）不碰 ROS，
  7 个 gtest 覆盖：顺序推进、循环、重试、成功清零重试、暂停/恢复、取消、越界取值
- **调度**：`mission_executor_node` 200ms tick；RUNNING 且没有在飞目标时才发下一个
- **结果判定**：`rclcpp_action::ResultCode::SUCCEEDED && result.success` 才算成功；
  CANCELED 不算失败（暂停/取消场景）
- **失败不自动跳过点位**：当前策略是"重试用尽 → 整任务失败"，需要"跳过"语义时在
  `MissionRunner` 加策略（任务层改动，不影响导航层）

---

## 5. 使用示例

```bash
# 1) 起导航（含健康监控）
ros2 launch pnc_nav_bringup bringup.launch.py profile:=sim2d

# 2) 起任务（配置在 pnc_nav_task/config/mission.yaml）
ros2 launch pnc_nav_task mission.launch.py auto_start:=true

# 3) 运行中干预
ros2 service call /mission/pause   std_srvs/srv/Trigger
ros2 service call /mission/resume  std_srvs/srv/Trigger
ros2 service call /mission/cancel  std_srvs/srv/Trigger
ros2 topic echo /mission_status
```

实测：
- 平地点任务 `[1,0] → [2,0] → [0.5,0]`：`SUCCEEDED (3/3)`，32.9s（折返段 19.9s）
- **坡道分段任务**（`config/mission_slope.yaml`：坡底 1.5 → 2.5 → 3.5，配合地形代价 + DWA）：
  `SUCCEEDED (3/3)`，30.3s——验证"分段下发"能解决"一次性远目标爬坡超时"的问题；

运行中 cancel → `state=CANCELLED`，导航 goal 返回 `code=5`（CANCELED），任务不推进。

---

## 6. 已知限制 / 下一步

| 限制 | 说明 |
|---|---|
| 只有顺序执行 | 无行为树、无条件分支、无并行 |
| 无中途插队 | 不支持运行中追加点位（需要 start 重启任务） |
| 无 checkpoint | 进程重启后任务丢失（MoveXBot 的 `task_checkpoint` 是参考） |
| 无"跳过失败点"策略 | 目前失败即整体失败 |
| 折返段偏慢 | KI-020 修复后能走完，但 180° 折返要 50s（原地转 + 走走正正），后续可优化预判 |

下一步优先级：**插队/跳过策略 → checkpoint → 行为树化（如果出现 3 种以上恢复策略）**。

## 7. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：MissionRunner 纯逻辑 + mission_executor_node + 7 个单测；/nav_status 外发 |
| 2026-10-23 | 三点任务（含折返）实测通过：SUCCEEDED 3/3（KI-020 修复） |
