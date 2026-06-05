// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/path_trackers/pure_pursuit_3d.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace pnc_nav_planners
{

void PurePursuit3D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  node_ = node;
  name_ = name;

  node_->declare_parameter(name_ + ".lookahead_distance", 0.6);
  node_->declare_parameter(name_ + ".min_lookahead", 0.3);
  node_->declare_parameter(name_ + ".max_lookahead", 1.5);
  node_->declare_parameter(name_ + ".lookahead_gain", 0.5);
  node_->declare_parameter(name_ + ".max_linear_vel", 0.5);
  node_->declare_parameter(name_ + ".max_angular_vel", 1.0);
  node_->declare_parameter(name_ + ".min_linear_vel", 0.05);

  lookahead_distance_ = node_->get_parameter(name_ + ".lookahead_distance").as_double();
  min_lookahead_ = node_->get_parameter(name_ + ".min_lookahead").as_double();
  max_lookahead_ = node_->get_parameter(name_ + ".max_lookahead").as_double();
  lookahead_gain_ = node_->get_parameter(name_ + ".lookahead_gain").as_double();
  max_linear_vel_ = node_->get_parameter(name_ + ".max_linear_vel").as_double();
  max_angular_vel_ = node_->get_parameter(name_ + ".max_angular_vel").as_double();
  min_linear_vel_ = node_->get_parameter(name_ + ".min_linear_vel").as_double();

  RCLCPP_INFO(node_->get_logger(), "PurePursuit3D configured: Ld=%.2f", lookahead_distance_);
}

void PurePursuit3D::activate()
{
  RCLCPP_INFO(node_->get_logger(), "PurePursuit3D activated");
}

void PurePursuit3D::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "PurePursuit3D deactivated");
}

void PurePursuit3D::cleanup()
{
  path_.poses.clear();
  node_.reset();
}

bool PurePursuit3D::setPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return false;
  }

  path_ = path;
  current_waypoint_idx_ = 0;
  return true;
}

double PurePursuit3D::computeAdaptiveLookahead(double current_speed) const
{
  double ld = min_lookahead_ + lookahead_gain_ * std::abs(current_speed);
  return std::clamp(ld, min_lookahead_, max_lookahead_);
}

geometry_msgs::msg::PoseStamped PurePursuit3D::findLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double lookahead_dist) const
{
  geometry_msgs::msg::PoseStamped lookahead_point;

  if (path_.poses.empty()) {
    return current_pose;
  }

  double robot_x = current_pose.pose.position.x;
  double robot_y = current_pose.pose.position.y;
  double robot_z = current_pose.pose.position.z;

  // 从当前路径点开始向前搜索
  for (size_t i = current_waypoint_idx_; i < path_.poses.size(); ++i) {
    double dx = path_.poses[i].pose.position.x - robot_x;
    double dy = path_.poses[i].pose.position.y - robot_y;
    double dz = path_.poses[i].pose.position.z - robot_z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist >= lookahead_dist) {
      lookahead_point = path_.poses[i];
      return lookahead_point;
    }
  }

  // 如果没找到，返回路径终点
  return path_.poses.back();
}

double PurePursuit3D::computeCurvature(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & lookahead_point) const
{
  // 获取机器人朝向
  double yaw = tf2::getYaw(current_pose.pose.orientation);

  // 前视点在机器人坐标系下的位置
  double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
  double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;

  // 转换到机器人坐标系
  double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
  double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);

  (void)local_x;  // Pure Pursuit 只用 local_y

  // 前视距离
  double ld_sq = dx*dx + dy*dy;
  if (ld_sq < 1e-6) return 0.0;

  // 曲率 = 2 * y / L^2
  double curvature = 2.0 * local_y / ld_sq;
  return curvature;
}

geometry_msgs::msg::TwistStamped PurePursuit3D::computeVelocityCommand(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & current_vel)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = current_pose.header;

  if (path_.poses.empty()) {
    return cmd;  // 零速度
  }

  // 更新当前最近路径点索引
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = current_waypoint_idx_; i < path_.poses.size(); ++i) {
    double dx = path_.poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = path_.poses[i].pose.position.y - current_pose.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < min_dist) {
      min_dist = dist;
      current_waypoint_idx_ = i;
    } else if (dist > min_dist + 1.0) {
      break;  // 已经过了最近点
    }
  }

  // 计算横向误差
  cross_track_error_ = min_dist;

  // 自适应前视距离
  double speed = std::sqrt(current_vel.linear.x * current_vel.linear.x +
                           current_vel.linear.y * current_vel.linear.y);
  double ld = computeAdaptiveLookahead(speed);

  // 找前视点
  auto lookahead_point = findLookaheadPoint(current_pose, ld);

  // 计算曲率
  double curvature = computeCurvature(current_pose, lookahead_point);

  // 计算航向误差
  double target_yaw = std::atan2(
    lookahead_point.pose.position.y - current_pose.pose.position.y,
    lookahead_point.pose.position.x - current_pose.pose.position.x);
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  heading_error_ = target_yaw - current_yaw;
  // 归一化到 [-pi, pi]
  while (heading_error_ > M_PI) heading_error_ -= 2.0 * M_PI;
  while (heading_error_ < -M_PI) heading_error_ += 2.0 * M_PI;

  // 线速度：根据曲率减速
  double linear_vel = max_linear_vel_ / (1.0 + std::abs(curvature) * 2.0);
  linear_vel = std::clamp(linear_vel, min_linear_vel_, max_linear_vel_);

  // 接近终点时减速
  double dx_goal = path_.poses.back().pose.position.x - current_pose.pose.position.x;
  double dy_goal = path_.poses.back().pose.position.y - current_pose.pose.position.y;
  double dist_to_goal = std::sqrt(dx_goal*dx_goal + dy_goal*dy_goal);
  if (dist_to_goal < ld) {
    linear_vel *= (dist_to_goal / ld);
    linear_vel = std::max(linear_vel, min_linear_vel_);
  }

  // 角速度 = 线速度 × 曲率
  double angular_vel = linear_vel * curvature;
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  cmd.twist.linear.x = linear_vel;
  cmd.twist.angular.z = angular_vel;

  return cmd;
}

bool PurePursuit3D::isPathCompleted(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double dist_tolerance)
{
  if (path_.poses.empty()) return true;

  double dx = path_.poses.back().pose.position.x - current_pose.pose.position.x;
  double dy = path_.poses.back().pose.position.y - current_pose.pose.position.y;
  double dz = path_.poses.back().pose.position.z - current_pose.pose.position.z;
  double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

  return dist < dist_tolerance;
}

}  // namespace pnc_nav_planners

PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::PurePursuit3D, pnc_nav_core::PathTrackerBase)
