# WP1 运动层骨架（Phase 4 入口）

> 负责人：运动层 agent ｜ 状态：已实现（骨架 + 四足 demo 链路）
> 依赖：`pnc_nav_core` 的 `LocomotionControllerBase`（已存在，未接入）

## 目标
把 `cmd_vel → 关节指令` 这条边界打通：导航层只发速度，运动层负责把它变成关节命令。
第一版**不接真机/真步态**，用一个"假步态"插件证明链路和契约成立。

## 交付物
```
src/pnc_nav_locomotion/
├── include/pnc_nav_locomotion/
│   ├── locomotion_server.hpp        # pluginlib 加载 + /cmd_vel → /joint_commands
│   └── fake_trot_locomotion.hpp     # 假步态插件（12 关节正弦）
├── src/...
├── plugins.xml
├── config/locomotion.yaml
├── launch/locomotion.launch.py
└── test/test_fake_trot.cpp
```

## 接口契约
| 项 | 约定 |
|---|---|
| 输入 | `/cmd_vel` (`geometry_msgs/Twist`)，超时 `cmd_vel_timeout`（默认 0.5s）无消息 → 视为零速 |
| 回调节奏 | `locomotion.control_frequency`（默认 100Hz，步态通常 200–1000Hz，仿真先 100） |
| 输出 | `/joint_commands` (`pnc_nav_interfaces/JointCommands`)，含 position/velocity/kp/kd |
| 插件基类 | `pnc_nav_core::LocomotionControllerBase`（configure/computeJointCommands/getControlFrequency/emergencyStop/getName） |
| 安全 | 收到 `std_srvs/Trigger` 急停服务后输出零速+中性关节，需 `/locomotion/reset` 恢复 |

## 验收
1. `ros2 launch pnc_nav_locomotion locomotion.launch.py` 后 `/joint_commands` 频率 ≈ 配置值
2. 发 `cmd_vel.linear.x=0.3` → 关节位置呈周期性（正弦），幅值随速度增大
3. 停发 cmd_vel > timeout → 关节回到中性位
4. 单测：零速→中性；速度增大→幅值增大；急停→中性

## 不做
- 真步态（RL/MPC/CPG）、Go2 模型接入、CHAMP 集成（后续 WP4）
