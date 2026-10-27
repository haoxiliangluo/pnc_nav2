// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_LOCOMOTION__FAKE_TROT_LOCOMOTION_HPP_
#define PNC_NAV_LOCOMOTION__FAKE_TROT_LOCOMOTION_HPP_

#include <string>

#include "pnc_nav_core/controller_base.hpp"

namespace pnc_nav_locomotion
{

/**
 * @class FakeTrotLocomotion
 * @brief 假步态插件：把 cmd_vel 变成 12 个关节的正弦指令
 *
 * 它不是真步态（没有接触时序/动力学），只用来把
 * `导航层 cmd_vel → 运动层关节指令` 这条边界打通。
 * 真步态（CHAMP / RL / MPC）以后作为同一个基类的另一个插件接入。
 */
class FakeTrotLocomotion : public pnc_nav_core::LocomotionControllerBase
{
public:
  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name) override;

  pnc_nav_core::JointCommands computeJointCommands(
    const geometry_msgs::msg::Twist & cmd_vel,
    const pnc_nav_core::RobotState & current_state) override;

  double getControlFrequency() const override {return frequency_;}
  pnc_nav_core::JointCommands emergencyStop(
    const pnc_nav_core::RobotState & current_state) override;
  void cleanup() override;
  std::string getName() const override {return "FakeTrotLocomotion";}

  /// 最近一次输出的摆动幅值（测试/调试用）
  double lastAmplitude() const {return last_amplitude_;}

private:
  pnc_nav_core::JointCommands neutral() const;
  void resetPhase() {phase_ = 0.0;}

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  double frequency_{100.0};      // 输出频率 (Hz)
  double gait_frequency_{2.0};   // 步态频率 (Hz)
  double max_stride_{0.35};      // 最大摆动幅值 (rad)
  double amplitude_speed_{0.3};  // 达到最大幅值对应的线速度 (m/s)
  double knee_lift_{0.25};       // 抬腿幅值 (rad)
  double hip_yaw_gain_{0.2};     // 角速度 → 髋关节耦合

  double phase_{0.0};
  double last_amplitude_{0.0};
};

}  // namespace pnc_nav_locomotion

#endif  // PNC_NAV_LOCOMOTION__FAKE_TROT_LOCOMOTION_HPP_
