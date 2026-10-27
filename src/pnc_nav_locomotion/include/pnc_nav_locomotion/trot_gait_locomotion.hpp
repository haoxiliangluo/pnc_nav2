// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_LOCOMOTION__TROT_GAIT_LOCOMOTION_HPP_
#define PNC_NAV_LOCOMOTION__TROT_GAIT_LOCOMOTION_HPP_

#include <string>

#include "pnc_nav_core/controller_base.hpp"

namespace pnc_nav_locomotion
{

/**
 * @class TrotGaitLocomotion
 * @brief 解析 IK 小跑步态（Go2 尺寸）：cmd_vel → 足端轨迹 → 关节角
 *
 * 与 FakeTrot 的区别：不是直接摆正弦关节，而是
 *   1) 按 cmd_vel 规划每条腿的足端轨迹（支撑相/摆动相 + 抬腿）
 *   2) 用 2 连杆解析 IK 求 thigh/calf，保证关节角在 Go2 限位内
 * 零速时输出"站立位"（thigh≈0.79, calf≈-1.56），不会像 FakeTrot 那样把腿压塌。
 *
 * 仍没有平衡控制/接触时序反馈，属于"开环步态"。
 */
class TrotGaitLocomotion : public pnc_nav_core::LocomotionControllerBase
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
  std::string getName() const override {return "TrotGaitLocomotion";}

  /// 站立位大腿/小腿角（测试用）
  double standThigh() const {return stand_thigh_;}
  double standCalf() const {return stand_calf_;}

private:
  /// 单腿 IK：足端在髋系 (x 前, z 下为负) → (thigh, calf)
  void solveLegIk(double foot_x, double foot_z, double & thigh, double & calf) const;
  pnc_nav_core::JointCommands standPose() const;

  rclcpp::Node::SharedPtr node_;
  std::string name_;

  double frequency_{100.0};
  double cycle_time_{0.5};      // 步态周期 (s)
  std::string gait_{"walk"};    // walk（三足支撑）| trot（对角）
  double duty_{0.75};           // 支撑相占比（walk 0.75 / trot 0.5）
  double max_joint_speed_{4.0}; // 关节指令斜率上限 (rad/s)，防止起步冲击翻车
  double body_height_{0.30};    // 髋到足端的垂直距离 (m)
  double step_height_{0.06};    // 抬腿高度 (m)
  double max_stride_{0.10};     // 单腿最大步幅 (m)
  double l1_{0.213};            // thigh 长度 (m, Go2)
  double l2_{0.213};            // calf 长度 (m, Go2)

  double phase_{0.0};
  std::vector<double> last_positions_;// 上一帧指令（斜率限制用）
  double stand_thigh_{0.0};
  double stand_calf_{0.0};
};

}  // namespace pnc_nav_locomotion

#endif  // PNC_NAV_LOCOMOTION__TROT_GAIT_LOCOMOTION_HPP_
