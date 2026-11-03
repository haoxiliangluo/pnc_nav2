// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_locomotion/wheel_legged_locomotion.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace pnc_nav_locomotion
{

namespace
{
constexpr int kLegJoints = 12;
constexpr int kTotalJoints = 16;

const char * kJointNames[kTotalJoints] = {
  "FL_hip", "FL_thigh", "FL_calf",
  "FR_hip", "FR_thigh", "FR_calf",
  "RL_hip", "RL_thigh", "RL_calf",
  "RR_hip", "RR_thigh", "RR_calf",
  "FL_wheel", "FR_wheel", "RL_wheel", "RR_wheel",
};
}  // namespace

void WheelLeggedLocomotion::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  node_ = node;
  name_ = name;

  frequency_ = node_->declare_parameter<double>(name_ + ".frequency", frequency_);
  body_height_ = node_->declare_parameter<double>(name_ + ".body_height", body_height_);
  thigh_length_ = node_->declare_parameter<double>(name_ + ".thigh_length", thigh_length_);
  calf_length_ = node_->declare_parameter<double>(name_ + ".calf_length", calf_length_);
  wheel_radius_ = node_->declare_parameter<double>(name_ + ".wheel_radius", wheel_radius_);
  track_ = node_->declare_parameter<double>(name_ + ".track", track_);
  max_linear_ = node_->declare_parameter<double>(name_ + ".max_linear", max_linear_);
  max_wheel_speed_ = node_->declare_parameter<double>(
    name_ + ".max_wheel_speed", max_wheel_speed_);
  linear_scale_ = node_->declare_parameter<double>(name_ + ".linear_scale", linear_scale_);
  angular_scale_ = node_->declare_parameter<double>(name_ + ".angular_scale", angular_scale_);
  use_velocity_feedback_ = node_->declare_parameter<bool>(
    name_ + ".use_velocity_feedback", use_velocity_feedback_);
  velocity_gain_ = node_->declare_parameter<double>(name_ + ".velocity_gain", velocity_gain_);

  frequency_ = std::max(10.0, frequency_);
  body_height_ = std::clamp(body_height_, 0.15, thigh_length_ + calf_length_ - 0.01);
  wheel_radius_ = std::max(0.02, wheel_radius_);

  solveLegIk(0.0, -body_height_, stand_thigh_, stand_calf_);
  RCLCPP_INFO(
    node_->get_logger(),
    "[WheelLeggedLocomotion] configured: %.0f Hz, body=%.2fm, wheel_r=%.3fm, "
    "track=%.2fm, stand=(thigh %.2f, calf %.2f)",
    frequency_, body_height_, wheel_radius_, track_, stand_thigh_, stand_calf_);
}

void WheelLeggedLocomotion::solveLegIk(
  double foot_x, double foot_z, double & thigh, double & calf) const
{
  const double r = std::max(0.05, -foot_z);
  double d = std::hypot(foot_x, r);
  d = std::clamp(d, 0.06, thigh_length_ + calf_length_ - 0.002);

  const double cos_knee =
    (thigh_length_ * thigh_length_ + calf_length_ * calf_length_ - d * d) /
    (2.0 * thigh_length_ * calf_length_);
  calf = -std::acos(std::clamp(cos_knee, -1.0, 1.0));

  const double cos_alpha =
    (thigh_length_ * thigh_length_ + d * d - calf_length_ * calf_length_) /
    (2.0 * thigh_length_ * d);
  const double alpha = std::acos(std::clamp(cos_alpha, -1.0, 1.0));
  // 与 Go2 URDF 一致：thigh 绕 +y 旋转使足端向 -x
  thigh = std::atan2(-foot_x, r) + alpha;
}

pnc_nav_core::JointCommands WheelLeggedLocomotion::computeJointCommands(
  const geometry_msgs::msg::Twist & cmd_vel,
  const pnc_nav_core::RobotState & current_state)
{
  pnc_nav_core::JointCommands out;
  out.joint_names.assign(kJointNames, kJointNames + kTotalJoints);
  out.positions.assign(kTotalJoints, 0.0);
  out.velocities.assign(kTotalJoints, 0.0);
  out.torques.assign(kTotalJoints, 0.0);
  out.kp.assign(kTotalJoints, 40.0);
  out.kd.assign(kTotalJoints, 2.0);

  // 腿：站立位（位置控制）
  for (int leg = 0; leg < 4; ++leg) {
    out.positions[leg * 3 + 1] = stand_thigh_;
    out.positions[leg * 3 + 2] = stand_calf_;
  }

  // 轮：差速（速度控制）
  // 前馈标定 + 可选闭环补偿：
  // 前馈：cmd 0.3 → 实测 0.26（×1.15）；cmd 0.6 rad/s → 实测 0.34（×1.75）
  const double dt = 1.0 / std::max(10.0, frequency_);
  if (use_velocity_feedback_) {
    const double vx_cmd = cmd_vel.linear.x;
    const double wz_cmd = cmd_vel.angular.z;

    if (std::abs(vx_cmd) > 1e-3) {
      const double err = vx_cmd - current_state.linear_vel_x;
      comp_linear_ += velocity_gain_ * err * dt / std::max(0.10, std::abs(vx_cmd));
      comp_linear_ = std::clamp(comp_linear_, 0.5, 3.0);
    } else {
      comp_linear_ = 1.0;
    }

    if (std::abs(wz_cmd) > 1e-3) {
      const double err = wz_cmd - current_state.angular_vel_z;
      comp_angular_ += velocity_gain_ * err * dt / std::max(0.20, std::abs(wz_cmd));
      comp_angular_ = std::clamp(comp_angular_, 0.5, 4.0);
    } else {
      comp_angular_ = 1.0;
    }
  }

  const double vx = std::clamp(
    cmd_vel.linear.x * linear_scale_ * comp_linear_, -max_linear_, max_linear_);
  const double wz = std::clamp(
    cmd_vel.angular.z * angular_scale_ * comp_angular_, -1.5, 1.5);
  const double v_left = vx - wz * track_ * 0.5;
  const double v_right = vx + wz * track_ * 0.5;
  const double w_left = std::clamp(v_left / wheel_radius_, -max_wheel_speed_, max_wheel_speed_);
  const double w_right = std::clamp(v_right / wheel_radius_, -max_wheel_speed_, max_wheel_speed_);

  out.velocities[kLegJoints + 0] = w_left;    // FL
  out.velocities[kLegJoints + 2] = w_left;    // RL
  out.velocities[kLegJoints + 1] = w_right;   // FR
  out.velocities[kLegJoints + 3] = w_right;   // RR

  return out;
}

pnc_nav_core::JointCommands WheelLeggedLocomotion::emergencyStop(
  const pnc_nav_core::RobotState & /*current_state*/)
{
  return computeJointCommands(geometry_msgs::msg::Twist{}, pnc_nav_core::RobotState{});
}

void WheelLeggedLocomotion::cleanup()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "[WheelLeggedLocomotion] cleanup");
  }
  node_.reset();
}

}  // namespace pnc_nav_locomotion

PLUGINLIB_EXPORT_CLASS(
  pnc_nav_locomotion::WheelLeggedLocomotion, pnc_nav_core::LocomotionControllerBase)
