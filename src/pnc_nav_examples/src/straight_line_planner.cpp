// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_examples/straight_line_planner.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace pnc_nav_examples
{

void StraightLinePlanner::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap)
{
  node_ = node;
  name_ = name;
  costmap_ = costmap;

  // 约定：插件自己的参数一律挂在 <插件实例名>. 命名空间下
  node_->declare_parameter(name_ + ".step", 0.1);
  node_->declare_parameter(name_ + ".max_length", 50.0);

  step_ = node_->get_parameter(name_ + ".step").as_double();
  max_length_ = node_->get_parameter(name_ + ".max_length").as_double();
  if (step_ <= 1e-3) {
    step_ = 0.1;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[%s] configured: step=%.3f m max_length=%.1f m",
    getName().c_str(), step_, max_length_);
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = start.header.frame_id.empty() ? "map" : start.header.frame_id;

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double dz = goal.pose.position.z - start.pose.position.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);

  // 演示"规划失败"的返回约定：返回空路径，NavServer 会进入 RECOVERING
  if (length > max_length_) {
    RCLCPP_WARN(
      node_->get_logger(), "[%s] goal too far (%.1f m > %.1f m), returning empty path",
      getName().c_str(), length, max_length_);
    return path;
  }

  const double yaw = std::atan2(dy, dx);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / step_)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start.pose.position.x + dx * t;
    pose.pose.position.y = start.pose.position.y + dy * t;
    pose.pose.position.z = start.pose.position.z + dz * t;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(
    node_->get_logger(), "[%s] straight-line path: %.2f m, %zu poses",
    getName().c_str(), length, path.poses.size());
  return path;
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] cleanup", getName().c_str());
  costmap_.reset();
  node_.reset();
}

}  // namespace pnc_nav_examples

// 这一行是插件能被 pluginlib 发现的"开关"
PLUGINLIB_EXPORT_CLASS(pnc_nav_examples::StraightLinePlanner, pnc_nav_core::GlobalPlannerBase)
