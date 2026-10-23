# Phase 2 验证清单 — Action 接口 + 最小恢复闭环

> 日期：2026-10-23
> 环境：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic（无 GUI，`use_gui:=false use_rviz:=false`）
> 配置：`AStar2D` + `PurePursuit2D`（baseline），`max_planning_retries=3`，`progress_timeout=8s`
> 隔离：`ROS_DOMAIN_ID=43`、`GZ_PARTITION=pnc_nav`

## ✅ 用例结果

| # | 用例 | 命令 | 期望 | 实测结果 |
|---|------|------|------|----------|
| 1 | 可达目标 | `action_smoke_test.py feedback`（goal 2.0, 0.0） | success=true, error_code=0 | ✅ `success=true, code=0, 20.48s, 1.93m, "goal reached"` |
| 2 | 中途取消 | `action_smoke_test.py cancel`（goal 5.0, 0.0 → 2s 后 cancel） | error_code=4，车停下 | ✅ `success=false, code=4, 4.31s, 0.27m, "goal cancelled by client"` |
| 3 | 不可达目标 | `action_smoke_test.py unreachable`（goal 999, 999） | error_code=3，不卡死 | ✅ `success=false, code=3, 12.71s, "recovery attempts exhausted after 4 tries"` |
| 4 | 兼容入口 `/goal_pose` | `ros2 topic pub --once /goal_pose ...`（0.5, 1.0） | 同 action 效果 | ✅ `idle → planning → following → succeeded`，终位姿 (0.45, 1.02) |
| 5 | feedback 连续性 | 用例 1 期间 | 20Hz 级 feedback | ✅ 状态/剩余距离/ETA/恢复次数持续刷新 |

日志证据（用例 3）：
```
State transition: planning -> recovering
Recovery spin done, replanning
State transition: recovering -> planning
Global planner failed to find a path (recovery 3/3)
Goal finished: success=false error_code=3(MAX_RETRIES) time=12.7s msg=recovery attempts exhausted after 4 tries
State transition: planning -> failed
```

## 🧩 本次改动落点

| 文件 | 改动 |
|---|---|
| `pnc_nav_interfaces/action/NavigateTo3DPose.action` | Result 增加 `uint16 error_code` |
| `pnc_nav_core/include/pnc_nav_core/nav_server.hpp` | action 服务句柄、恢复/看门狗/会话状态、`NavErrorCode` 枚举 |
| `pnc_nav_core/src/nav_server.cpp` | action server（goal/cancel/accepted）、feedback、result、抢占、`RECOVERING` 实现、卡住检测、TF 超时失败、错误码 |
| `pnc_nav_core/{package.xml,CMakeLists.txt}` | 增加 `rclcpp_action`、`pnc_nav_interfaces` 依赖 |
| `pnc_nav_bringup/config/nav_params.yaml` | 新增 `recovery_spin_duration / recovery_angular_vel / feedback_eta_speed / progress_timeout / progress_epsilon / tf_timeout_ticks` |
| `test/action_smoke_test.py` | 新增 action 冒烟测试（feedback / cancel / unreachable） |
| `pnc_nav_rviz/src/pnc_panel.cpp`、`nav_server.cpp` | 清除调试 `fprintf` |

## ⚠️ 验证中暴露的问题（下一轮）

1. **DWA2D 停在目标前 ~0.25 m**：`isGoalReached` 与路径跟踪器的到达判定不一致，目标点落在膨胀层内时 DWA 不敢靠近（用例 1 首次用 DWA 时 `dist=0.30` 反复恢复）。
2. **DWA2D 静态代价溢出**：日志出现 `static=1.7976931348623157e308`（`double` 上界），静态代价分支未正确计算。
3. **DWA2D 每周期 INFO 刷屏**（`与目标距离差未达到`）：应降级为 DEBUG 或 throttle。
4. **到达判定语义**：feedback 的 `distance_remaining` 是"到目标点距离"，而 tracker 判定用的是"到路径末点距离"，两者不一致会出现 `dist=0.20` 但已成功的情况——下一步统一为到目标点距离。
5. **恢复动作仍是最小实现**：清代价地图待 obstacle layer 加入后再补；原地旋转时长/速度需要按车体调参。

## 🎯 下一步建议

1. 修 DWA2D 的 `static cost` 溢出 + 到达判定（统一用 goal 距离）+ 日志降级
2. costmap 增加 obstacle layer + 滚动窗口（订阅 `/scan`），让"局部避障"名副其实
3. 恢复行为扩展：清局部代价地图 → 后退 → 旋转 → 重规划，并按失败原因区分错误码
