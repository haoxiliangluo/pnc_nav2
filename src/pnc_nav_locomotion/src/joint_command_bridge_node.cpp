// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License
//
// joint_command_bridge_node：把 JointCommands 拆成每个关节的 Float64 话题，
// 经 ros_gz_bridge 送给 gz 的 JointPositionController（四足 demo 用）。

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pnc_nav_interfaces/msg/joint_commands.hpp"

namespace pnc_nav_locomotion
{

class JointCommandBridgeNode : public rclcpp::Node
{
public:
  JointCommandBridgeNode() : Node("joint_command_bridge")
  {
    prefix_ = declare_parameter<std::string>("joint_bridge.prefix", "/model/quad/joint");
    suffix_ = declare_parameter<std::string>("joint_bridge.suffix", "/cmd_pos");
    input_topic_ = declare_parameter<std::string>("joint_bridge.input_topic", "joint_commands");

    sub_ = create_subscription<pnc_nav_interfaces::msg::JointCommands>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&JointCommandBridgeNode::onJointCommands, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "joint_command_bridge: %s -> %s/<joint>%s (每 100 帧打印一次)",
      input_topic_.c_str(), prefix_.c_str(), suffix_.c_str());
  }

private:
  void onJointCommands(const pnc_nav_interfaces::msg::JointCommands::SharedPtr msg)
  {
    const size_t n = std::min(msg->joint_names.size(), msg->positions.size());
    for (size_t i = 0; i < n; ++i) {
      const std::string & name = msg->joint_names[i];
      auto it = pubs_.find(name);
      if (it == pubs_.end()) {
        it = pubs_.emplace(
          name, create_publisher<std_msgs::msg::Float64>(
            prefix_ + "/" + name + suffix_, rclcpp::QoS(10))).first;
      }
      std_msgs::msg::Float64 value;
      value.data = msg->positions[i];
      it->second->publish(value);
    }

    if (++frame_ % 100 == 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "桥接 %zu 个关节指令（示例 %s=%.3f）", n,
        msg->joint_names.empty() ? "-" : msg->joint_names.front().c_str(),
        msg->positions.empty() ? 0.0 : msg->positions.front());
    }
  }

  std::string prefix_;
  std::string suffix_;
  std::string input_topic_;
  size_t frame_{0};
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
  rclcpp::Subscription<pnc_nav_interfaces::msg::JointCommands>::SharedPtr sub_;
};

}  // namespace pnc_nav_locomotion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_nav_locomotion::JointCommandBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
