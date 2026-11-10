// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License
//
// locomotion_node：运动层入口
//   /cmd_vel ──▶ LocomotionController 插件 ──▶ /joint_commands
// 服务：/locomotion/estop、/locomotion/reset（std_srvs/Trigger）

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "pnc_nav_core/controller_base.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "pnc_nav_interfaces/msg/joint_commands.hpp"

namespace pnc_nav_locomotion
{

class LocomotionNode : public rclcpp::Node
{
public:
  LocomotionNode() : Node("locomotion")
  {
    plugin_name_ = declare_parameter<std::string>(
      "locomotion.plugin", "pnc_nav_locomotion::FakeTrotLocomotion");
    instance_name_ = declare_parameter<std::string>("locomotion.instance_name", "fake_trot");
    cmd_vel_timeout_ = declare_parameter<double>("locomotion.cmd_vel_timeout", 0.5);
    joint_topic_ = declare_parameter<std::string>("locomotion.joint_topic", "joint_commands");
    cmd_topic_ = declare_parameter<std::string>("locomotion.cmd_vel_topic", "cmd_vel");
    odom_topic_ = declare_parameter<std::string>("locomotion.odom_topic", "odom");

    loader_ = std::make_unique<pluginlib::ClassLoader<pnc_nav_core::LocomotionControllerBase>>(
      "pnc_nav_core", "pnc_nav_core::LocomotionControllerBase");
  }

  void initialize()
  {
    try {
      controller_ = loader_->createSharedInstance(plugin_name_);
      controller_->configure(shared_from_this(), instance_name_);
      controller_->activate();
      RCLCPP_INFO(get_logger(), "loaded locomotion plugin: %s", plugin_name_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "failed to load locomotion plugin '%s': %s",
        plugin_name_.c_str(), ex.what());
      return;
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = *msg;
        last_cmd_ = std::chrono::steady_clock::now();
      });

    joint_pub_ = create_publisher<pnc_nav_interfaces::msg::JointCommands>(joint_topic_, 10);

    // 速度反馈：插件可以用 RobotState 里的实测速度做闭环补偿
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {odom_ = *msg;});

    using Trigger = std_srvs::srv::Trigger;
    estop_srv_ = create_service<Trigger>(
      "locomotion/estop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        estop_ = true;
        res->success = true;
        res->message = "estop engaged";
        RCLCPP_WARN(get_logger(), "estop engaged");
      });
    reset_srv_ = create_service<Trigger>(
      "locomotion/reset",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        estop_ = false;
        cmd_ = geometry_msgs::msg::Twist();
        res->success = true;
        res->message = "estop released";
        RCLCPP_INFO(get_logger(), "estop released");
      });

    const double freq = std::max(10.0, controller_->getControlFrequency());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&LocomotionNode::tick, this));

    RCLCPP_INFO(
      get_logger(), "locomotion ready: plugin=%s, %s -> %s @ %.0f Hz, cmd_timeout=%.2fs",
      controller_->getName().c_str(), cmd_topic_.c_str(), joint_topic_.c_str(),
      freq, cmd_vel_timeout_);
  }

private:
  void tick()
  {
    if (!controller_) {
      return;
    }
    pnc_nav_core::RobotState state;
    // 姿态（姿态补偿用）：odom 四元数 -> roll/pitch/yaw
    {
      const auto & q = odom_.pose.pose.orientation;
      const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
      const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
      const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      state.roll = std::atan2(sinr_cosp, cosr_cosp);
      state.pitch = std::asin(std::clamp(sinp, -1.0, 1.0));
      state.yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    state.linear_vel_x = odom_.twist.twist.linear.x;
    state.linear_vel_y = odom_.twist.twist.linear.y;
    state.angular_vel_z = odom_.twist.twist.angular.z;
    pnc_nav_interfaces::msg::JointCommands msg;

    if (estop_) {
      const auto cmd = controller_->emergencyStop(state);
      fillMessage(msg, cmd);
      joint_pub_->publish(msg);
      return;
    }

    geometry_msgs::msg::Twist cmd = cmd_;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_).count() >
      cmd_vel_timeout_)
    {
      cmd = geometry_msgs::msg::Twist();   // 超时：当作零速
    }

    const auto joint_cmd = controller_->computeJointCommands(cmd, state);
    fillMessage(msg, joint_cmd);
    joint_pub_->publish(msg);
  }

  void fillMessage(
    pnc_nav_interfaces::msg::JointCommands & msg,
    const pnc_nav_core::JointCommands & cmd)
  {
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.joint_names = cmd.joint_names;
    msg.positions = cmd.positions;
    msg.velocities = cmd.velocities;
    msg.torques = cmd.torques;
    msg.kp = cmd.kp;
    msg.kd = cmd.kd;
  }

  std::string plugin_name_;
  std::string instance_name_;
  std::string joint_topic_;
  std::string cmd_topic_;
  std::string odom_topic_;
  double cmd_vel_timeout_{0.5};

  std::unique_ptr<pluginlib::ClassLoader<pnc_nav_core::LocomotionControllerBase>> loader_;
  pnc_nav_core::LocomotionControllerBase::SharedPtr controller_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<pnc_nav_interfaces::msg::JointCommands>::SharedPtr joint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist cmd_;
  nav_msgs::msg::Odometry odom_;
  std::chrono::steady_clock::time_point last_cmd_{};
  bool estop_{false};
};

}  // namespace pnc_nav_locomotion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pnc_nav_locomotion::LocomotionNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
