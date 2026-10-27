// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_locomotion/trot_gait_locomotion.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace pnc_nav_locomotion
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kJoints = 12;

const char * kJointNames[kJoints] = {
  "FL_hip", "FL_thigh", "FL_calf",
  "FR_hip", "FR_thigh", "FR_calf",
  "RL_hip", "RL_thigh", "RL_calf",
  "RR_hip", "RR_thigh", "RR_calf",
};

// 相位偏移（以周期比例计）
const double kTrotPhase[4] = {0.0, 0.5, 0.5, 0.0};   // 对角：FL+RR / FR+RL
const double kWalkPhase[4] = {0.0, 0.5, 0.25, 0.75}; // 迈步：四腿依次
}  // namespace

void TrotGaitLocomotion::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  node_ = node;
  name_ = name;

  frequency_ = node_->declare_parameter<double>(name_ + ".frequency", frequency_);
  cycle_time_ = node_->declare_parameter<double>(name_ + ".cycle_time", cycle_time_);
  body_height_ = node_->declare_parameter<double>(name_ + ".body_height", body_height_);
  step_height_ = node_->declare_parameter<double>(name_ + ".step_height", step_height_);
  max_stride_ = node_->declare_parameter<double>(name_ + ".max_stride", max_stride_);
  l1_ = node_->declare_parameter<double>(name_ + ".thigh_length", l1_);
  l2_ = node_->declare_parameter<double>(name_ + ".calf_length", l2_);
  max_joint_speed_ = node_->declare_parameter<double>(name_ + ".max_joint_speed", max_joint_speed_);
  gait_ = node_->declare_parameter<std::string>(name_ + ".gait", gait_);
  duty_ = node_->declare_parameter<double>(name_ + ".duty", duty_);
  if (gait_ == "trot") {
    duty_ = 0.5;
  } else if (gait_ == "walk") {
    duty_ = 0.75;
  }
  duty_ = std::clamp(duty_, 0.5, 0.9);

  frequency_ = std::max(10.0, frequency_);
  cycle_time_ = std::clamp(cycle_time_, 0.2, 2.0);
  body_height_ = std::clamp(body_height_, 0.15, l1_ + l2_ - 0.01);

  // 站立位：足端在正下方
  solveLegIk(0.0, -body_height_, stand_thigh_, stand_calf_);
  last_positions_.assign(kJoints, 0.0);

  RCLCPP_INFO(
    node_->get_logger(),
    "[TrotGaitLocomotion] configured: %.0f Hz, cycle=%.2fs, body_height=%.2fm, "
    "stand=(thigh %.2f, calf %.2f)",
    frequency_, cycle_time_, body_height_, stand_thigh_, stand_calf_);
}

void TrotGaitLocomotion::solveLegIk(
  double foot_x, double foot_z, double & thigh, double & calf) const
{
  // 髋系：x 前，z 向下为负；r 为髋到足端的距离
  const double r = std::max(0.05, -foot_z);
  double d = std::hypot(foot_x, r);
  d = std::clamp(d, 0.06, l1_ + l2_ - 0.002);

  const double cos_knee =
    (l1_ * l1_ + l2_ * l2_ - d * d) / (2.0 * l1_ * l2_);
  calf = -std::acos(std::clamp(cos_knee, -1.0, 1.0));

  const double cos_alpha = (l1_ * l1_ + d * d - l2_ * l2_) / (2.0 * l1_ * d);
  const double alpha = std::acos(std::clamp(cos_alpha, -1.0, 1.0));
  thigh = std::atan2(foot_x, r) + alpha;
}

pnc_nav_core::JointCommands TrotGaitLocomotion::standPose() const
{
  pnc_nav_core::JointCommands out;
  out.joint_names.assign(kJointNames, kJointNames + kJoints);
  out.positions.assign(kJoints, 0.0);
  out.velocities.assign(kJoints, 0.0);
  out.torques.assign(kJoints, 0.0);
  out.kp.assign(kJoints, 60.0);
  out.kd.assign(kJoints, 2.0);
  for (int leg = 0; leg < 4; ++leg) {
    out.positions[leg * 3 + 1] = stand_thigh_;
    out.positions[leg * 3 + 2] = stand_calf_;
  }
  return out;
}

pnc_nav_core::JointCommands TrotGaitLocomotion::computeJointCommands(
  const geometry_msgs::msg::Twist & cmd_vel,
  const pnc_nav_core::RobotState & /*current_state*/)
{
  const double vx = std::clamp(cmd_vel.linear.x, -0.3, 0.3);
  const double wz = std::clamp(cmd_vel.angular.z, -1.0, 1.0);

  // 零速：保持站立（同样做斜率限制）
  if (std::abs(vx) < 1e-3 && std::abs(wz) < 1e-3) {
    auto stand = standPose();
    const double max_delta = max_joint_speed_ / frequency_;
    for (size_t i = 0; i < stand.positions.size() && i < last_positions_.size(); ++i) {
      stand.positions[i] = last_positions_[i] + std::clamp(
        stand.positions[i] - last_positions_[i], -max_delta, max_delta);
    }
    last_positions_ = stand.positions;
    return stand;
  }

  // 步幅：线速度 × 周期，限幅；转向按左右腿差速（近似 v = vx ∓ wz*half_width）
  const double half_width = 0.10;
  const double stride = std::clamp(vx * cycle_time_, -max_stride_, max_stride_);

  auto out = standPose();
  phase_ += 1.0 / (cycle_time_ * frequency_);   // 0..1
  if (phase_ >= 1.0) {
    phase_ -= 1.0;
  }

  for (int leg = 0; leg < 4; ++leg) {
    const double sign = (leg == 0 || leg == 3) ? 1.0 : -1.0;   // FL/RR vs FR/RL
    const double leg_stride = std::clamp(
      stride - wz * half_width * cycle_time_ * sign, -max_stride_, max_stride_);

    double s = phase_ + (gait_ == "trot" ? kTrotPhase[leg] : kWalkPhase[leg]);
    if (s >= 1.0) {
      s -= 1.0;
    }

    double foot_x = 0.0;
    double lift = 0.0;
    const double swing = 1.0 - duty_;
    if (s < swing) {
      // 摆动相：从后往前，抬腿
      const double u = s / swing;
      foot_x = -0.5 * leg_stride + leg_stride * u;
      lift = step_height_ * std::sin(kPi * u);
    } else {
      // 支撑相：从前往后（相对机身），不抬腿
      const double u = (s - swing) / duty_;
      foot_x = 0.5 * leg_stride - leg_stride * u;
    }

    double thigh = stand_thigh_;
    double calf = stand_calf_;
    solveLegIk(foot_x, -(body_height_) + lift, thigh, calf);

    out.positions[leg * 3 + 0] = 0.0;   // 髋（无侧摆）
    out.positions[leg * 3 + 1] = thigh;
    out.positions[leg * 3 + 2] = calf;
  }

  // 斜率限制：每个控制周期最多变化 max_joint_speed_ / frequency_（防止起步/换相冲击）
  const double max_delta = max_joint_speed_ / frequency_;
  for (size_t i = 0; i < out.positions.size() && i < last_positions_.size(); ++i) {
    out.positions[i] = last_positions_[i] + std::clamp(
      out.positions[i] - last_positions_[i], -max_delta, max_delta);
  }
  last_positions_ = out.positions;
  return out;
}

pnc_nav_core::JointCommands TrotGaitLocomotion::emergencyStop(
  const pnc_nav_core::RobotState & /*current_state*/)
{
  phase_ = 0.0;
  return standPose();
}

void TrotGaitLocomotion::cleanup()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "[TrotGaitLocomotion] cleanup");
  }
  node_.reset();
}

}  // namespace pnc_nav_locomotion

PLUGINLIB_EXPORT_CLASS(
  pnc_nav_locomotion::TrotGaitLocomotion, pnc_nav_core::LocomotionControllerBase)
