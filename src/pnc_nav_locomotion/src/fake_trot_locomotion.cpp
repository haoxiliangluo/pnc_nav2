// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_locomotion/fake_trot_locomotion.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace pnc_nav_locomotion
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kJoints = 12;   // 4 腿 × 3 关节

const char * kJointNames[kJoints] = {
  "FL_hip", "FL_thigh", "FL_knee",
  "FR_hip", "FR_thigh", "FR_knee",
  "RL_hip", "RL_thigh", "RL_knee",
  "RR_hip", "RR_thigh", "RR_knee",
};

// 对角步态（trot）：FL/RL 同相，FR/RR 反相
const double kLegPhaseOffset[4] = {0.0, kPi, 0.0, kPi};
}  // namespace

void FakeTrotLocomotion::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  node_ = node;
  name_ = name;

  frequency_ = node_->declare_parameter<double>(name_ + ".frequency", frequency_);
  gait_frequency_ = node_->declare_parameter<double>(name_ + ".gait_frequency", gait_frequency_);
  max_stride_ = node_->declare_parameter<double>(name_ + ".max_stride", max_stride_);
  amplitude_speed_ = node_->declare_parameter<double>(name_ + ".amplitude_speed", amplitude_speed_);
  knee_lift_ = node_->declare_parameter<double>(name_ + ".knee_lift", knee_lift_);
  hip_yaw_gain_ = node_->declare_parameter<double>(name_ + ".hip_yaw_gain", hip_yaw_gain_);

  frequency_ = std::max(10.0, frequency_);
  gait_frequency_ = std::clamp(gait_frequency_, 0.2, 5.0);
  amplitude_speed_ = std::max(0.05, amplitude_speed_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[FakeTrotLocomotion] configured: %.0f Hz output, gait %.1f Hz, max_stride=%.2f rad",
    frequency_, gait_frequency_, max_stride_);
}

pnc_nav_core::JointCommands FakeTrotLocomotion::neutral() const
{
  pnc_nav_core::JointCommands out;
  out.joint_names.assign(kJointNames, kJointNames + kJoints);
  out.positions.assign(kJoints, 0.0);
  out.velocities.assign(kJoints, 0.0);
  out.torques.assign(kJoints, 0.0);
  out.kp.assign(kJoints, 60.0);
  out.kd.assign(kJoints, 2.0);
  return out;
}

pnc_nav_core::JointCommands FakeTrotLocomotion::computeJointCommands(
  const geometry_msgs::msg::Twist & cmd_vel,
  const pnc_nav_core::RobotState & /*current_state*/)
{
  const double vx = std::clamp(cmd_vel.linear.x, -0.6, 0.6);
  const double wz = std::clamp(cmd_vel.angular.z, -1.2, 1.2);

  // 幅值：线速度主导，转向也给一点（原地转时腿部也要动）
  const double speed_ratio = std::clamp(std::abs(vx) / amplitude_speed_, 0.0, 1.0);
  const double turn_ratio = std::clamp(std::abs(wz) / 1.0, 0.0, 1.0);
  last_amplitude_ = max_stride_ * std::max(speed_ratio, 0.4 * turn_ratio);

  auto out = neutral();
  out.joint_names.assign(kJointNames, kJointNames + kJoints);

  if (last_amplitude_ < 1e-6) {
    return out;   // 停住：输出中性位
  }

  phase_ += 2.0 * kPi * gait_frequency_ / frequency_;
  if (phase_ > 2.0 * kPi) {
    phase_ -= 2.0 * kPi;
  }

  const double direction = (vx >= 0.0) ? 1.0 : -1.0;
  for (int leg = 0; leg < 4; ++leg) {
    const double leg_phase = phase_ + kLegPhaseOffset[leg];
    const double swing = direction * last_amplitude_ * std::sin(leg_phase);
    const double lift = knee_lift_ * last_amplitude_ / std::max(1e-6, max_stride_) *
      std::max(0.0, std::sin(leg_phase + kPi / 2.0));

    out.positions[leg * 3 + 0] = -hip_yaw_gain_ * wz;   // 髋：转向耦合
    out.positions[leg * 3 + 1] = swing;                 // 大腿：前后摆
    out.positions[leg * 3 + 2] = lift;                  // 膝：抬腿
  }
  return out;
}

pnc_nav_core::JointCommands FakeTrotLocomotion::emergencyStop(
  const pnc_nav_core::RobotState & /*current_state*/)
{
  resetPhase();
  last_amplitude_ = 0.0;
  return neutral();
}

void FakeTrotLocomotion::cleanup()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "[FakeTrotLocomotion] cleanup");
  }
  node_.reset();
}

}  // namespace pnc_nav_locomotion

PLUGINLIB_EXPORT_CLASS(
  pnc_nav_locomotion::FakeTrotLocomotion, pnc_nav_core::LocomotionControllerBase)
