// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__CONTROLLER_BASE_HPP_
#define PNC_NAV_CORE__CONTROLLER_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace pnc_nav_core
{

/**
 * @struct RobotState
 * @brief 机器人当前状态（用于步态控制器输入）
 */
struct RobotState
{
  // 基座姿态 (roll, pitch, yaw)
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};

  // 基座角速度
  double angular_vel_x{0.0};
  double angular_vel_y{0.0};
  double angular_vel_z{0.0};

  // 基座线速度
  double linear_vel_x{0.0};
  double linear_vel_y{0.0};
  double linear_vel_z{0.0};

  // 关节状态 (12 joints for quadruped: 3 per leg × 4 legs)
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  // 足端接触状态 (4 legs)
  std::vector<bool> foot_contacts;
};

/**
 * @struct JointCommands
 * @brief 关节指令输出
 */
struct JointCommands
{
  std::vector<double> positions;     // 目标关节角度
  std::vector<double> velocities;    // 目标关节速度
  std::vector<double> torques;       // 前馈力矩
  std::vector<double> kp;            // 位置增益
  std::vector<double> kd;            // 速度增益
};

/**
 * @class LocomotionControllerBase
 * @brief 步态控制器插件基类
 *
 * 将高层速度指令 (cmd_vel) 转换为关节级指令。
 * 可实现 RL策略推理、MPC、CPG 等步态控制算法。
 */
class LocomotionControllerBase
{
public:
  using SharedPtr = std::shared_ptr<LocomotionControllerBase>;

  virtual ~LocomotionControllerBase() = default;

  /**
   * @brief 配置控制器
   * @param node ROS节点指针
   * @param name 控制器实例名称
   */
  virtual void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() = 0;

  /**
   * @brief 计算关节指令
   * @param cmd_vel 目标速度指令 (来自局部规划器)
   * @param current_state 机器人当前状态
   * @return 关节指令
   */
  virtual JointCommands computeJointCommands(
    const geometry_msgs::msg::Twist & cmd_vel,
    const RobotState & current_state) = 0;

  /**
   * @brief 获取控制频率 (Hz)
   * 步态控制通常需要高频率 (200-1000Hz)
   */
  virtual double getControlFrequency() const = 0;

  /**
   * @brief 紧急停止
   */
  virtual JointCommands emergencyStop(const RobotState & current_state) = 0;

  virtual std::string getName() const = 0;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__CONTROLLER_BASE_HPP_
