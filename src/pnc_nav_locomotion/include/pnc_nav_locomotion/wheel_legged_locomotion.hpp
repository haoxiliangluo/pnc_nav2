// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_LOCOMOTION__WHEEL_LEGGED_LOCOMOTION_HPP_
#define PNC_NAV_LOCOMOTION__WHEEL_LEGGED_LOCOMOTION_HPP_

#include <string>

#include "pnc_nav_core/controller_base.hpp"

namespace pnc_nav_locomotion
{

/**
 * @class WheelLeggedLocomotion
 * @brief 轮足（Go2-W）运动层：腿保持站立位，轮子按差速把 cmd_vel 变成轮速
 *
 * 12 个腿关节用位置控制（站立），4 个轮子用速度控制：
 *   v_left  = vx - wz * track/2,  v_right = vx + wz * track/2
 *   wheel omega = v / wheel_radius
 * 输出 JointCommands 里 16 个关节：腿给 positions，轮子给 velocities。
 */
class WheelLeggedLocomotion : public pnc_nav_core::LocomotionControllerBase
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
  std::string getName() const override {return "WheelLeggedLocomotion";}

private:
  void solveLegIk(double foot_x, double foot_z, double & thigh, double & calf) const;

  rclcpp::Node::SharedPtr node_;
  std::string name_;

  double frequency_{100.0};
  double body_height_{0.30};
  double thigh_length_{0.213};
  double calf_length_{0.213};
  double wheel_radius_{0.09};
  double track_{0.35};          // 左右轮距 (m)
  double max_linear_{0.6};
  double max_wheel_speed_{25.0};
  double linear_scale_{1.0};    // 线速度标定系数（补偿滑移）
  double angular_scale_{1.0};   // 角速度标定系数（补偿转向滑移）
  bool use_velocity_feedback_{true};// 是否用实测速度做闭环补偿
  double velocity_gain_{2.0};   // 补偿增益
  double comp_linear_{1.0};     // 线速度补偿状态
  double comp_angular_{1.0};    // 角速度补偿状态

  double stand_thigh_{0.0};
  double stand_calf_{0.0};
};

}  // namespace pnc_nav_locomotion

#endif  // PNC_NAV_LOCOMOTION__WHEEL_LEGGED_LOCOMOTION_HPP_
