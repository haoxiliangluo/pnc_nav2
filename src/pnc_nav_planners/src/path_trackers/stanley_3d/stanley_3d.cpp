// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/path_trackers/stanley_3d.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace pnc_nav_planners
{

void Stanley3D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  node_ = node;
  name_ = name;

  node_->declare_parameter(name_ + ".k_cross_track", 1.0);
  node_->declare_parameter(name_ + ".k_heading", 1.0);
  node_->declare_parameter(name_ + ".k_soft", 1.0);
  node_->declare_parameter(name_ + ".max_steer", 0.8);
  node_->declare_parameter(name_ + ".max_linear_vel", 0.5);
  node_->declare_parameter(name_ + ".min_linear_vel", 0.05);

  k_cross_track_ = node_->get_parameter(name_ + ".k_cross_track").as_double();
  k_heading_ = node_->get_parameter(name_ + ".k_heading").as_double();
  k_soft_ = node_->get_parameter(name_ + ".k_soft").as_double();
  max_steer_ = node_->get_parameter(name_ + ".max_steer").as_double();
  max_linear_vel_ = node_->get_parameter(name_ + ".max_linear_vel").as_double();
  min_linear_vel_ = node_->get_parameter(name_ + ".min_linear_vel").as_double();

  RCLCPP_INFO(node_->get_logger(), "Stanley3D configured: k_ct=%.2f, k_h=%.2f",
    k_cross_track_, k_heading_);
}

void Stanley3D::activate() {}
void Stanley3D::deactivate() {}

void Stanley3D::cleanup()
{
  path_.poses.clear();
  node_.reset();
}

void Stanley3D::setPath(const nav_msgs::msg::Path & path)
{
  path_ = path;
  current_waypoint_idx_ = 0;
}

size_t Stanley3D::findClosestPoint(const geometry_msgs::msg::PoseStamped & pose) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t closest = current_waypoint_idx_;

  // 从当前索引附近搜索，避免全路径遍历
  size_t search_start = (current_waypoint_idx_ > 5) ? current_waypoint_idx_ - 5 : 0;
  size_t search_end = std::min(current_waypoint_idx_ + 50, path_.poses.size());

  for (size_t i = search_start; i < search_end; ++i) {
    double dx = pose.pose.position.x - path_.poses[i].pose.position.x;
    double dy = pose.pose.position.y - path_.poses[i].pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest = i;
    }
  }
  return closest;
}

geometry_msgs::msg::TwistStamped Stanley3D::computeVelocityCommand(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & current_vel)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = current_pose.header;

  if (path_.poses.size() < 2) return cmd;

  // 找最近路径点
  current_waypoint_idx_ = findClosestPoint(current_pose);
  size_t next_idx = std::min(current_waypoint_idx_ + 1, path_.poses.size() - 1);

  // 路径切线方向（期望航向）
  double path_dx = path_.poses[next_idx].pose.position.x -
                   path_.poses[current_waypoint_idx_].pose.position.x;
  double path_dy = path_.poses[next_idx].pose.position.y -
                   path_.poses[current_waypoint_idx_].pose.position.y;
  double path_yaw = std::atan2(path_dy, path_dx);

  // 当前航向
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  // 航向误差
  heading_error_ = path_yaw - current_yaw;
  while (heading_error_ > M_PI) heading_error_ -= 2.0 * M_PI;
  while (heading_error_ < -M_PI) heading_error_ += 2.0 * M_PI;

  // 横向误差（带符号）
  double dx = current_pose.pose.position.x - path_.poses[current_waypoint_idx_].pose.position.x;
  double dy = current_pose.pose.position.y - path_.poses[current_waypoint_idx_].pose.position.y;
  // 横向误差 = 投影到路径法线方向
  cross_track_error_ = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);

  // 当前速度
  double speed = std::sqrt(current_vel.linear.x * current_vel.linear.x +
                           current_vel.linear.y * current_vel.linear.y);
  speed = std::max(speed, 0.01);  // 防止除零

  // Stanley 控制律: δ = θ_e + arctan(k * e / (v + k_soft))
  double steer = k_heading_ * heading_error_ +
                 std::atan2(k_cross_track_ * cross_track_error_, speed + k_soft_);
  steer = std::clamp(steer, -max_steer_, max_steer_);

  // 线速度：根据转向角减速
  double linear_vel = max_linear_vel_ * (1.0 - 0.5 * std::abs(steer) / max_steer_);
  linear_vel = std::clamp(linear_vel, min_linear_vel_, max_linear_vel_);

  // 接近终点减速
  double dx_goal = path_.poses.back().pose.position.x - current_pose.pose.position.x;
  double dy_goal = path_.poses.back().pose.position.y - current_pose.pose.position.y;
  double dist_to_goal = std::sqrt(dx_goal*dx_goal + dy_goal*dy_goal);
  if (dist_to_goal < 1.0) {
    linear_vel *= dist_to_goal;
    linear_vel = std::max(linear_vel, min_linear_vel_);
  }

  cmd.twist.linear.x = linear_vel;
  cmd.twist.angular.z = steer;

  return cmd;
}

bool Stanley3D::isPathCompleted(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double dist_tolerance)
{
  if (path_.poses.empty()) return true;

  double dx = path_.poses.back().pose.position.x - current_pose.pose.position.x;
  double dy = path_.poses.back().pose.position.y - current_pose.pose.position.y;
  double dist = std::sqrt(dx*dx + dy*dy);

  return dist < dist_tolerance;
}

}  // namespace pnc_nav_planners

PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::Stanley3D, pnc_nav_core::PathTrackerBase)
