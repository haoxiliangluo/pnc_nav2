// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/local_planners/dwa_3d.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace pnc_nav_planners
{

void DWA3D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap)
{
  node_ = node;
  name_ = name;
  costmap_ = costmap;

  node_->declare_parameter(name_ + ".max_vel_x", 0.5);
  node_->declare_parameter(name_ + ".max_vel_y", 0.3);
  node_->declare_parameter(name_ + ".max_vel_theta", 1.0);
  node_->declare_parameter(name_ + ".min_vel_x", -0.1);
  node_->declare_parameter(name_ + ".acc_lim_x", 1.0);
  node_->declare_parameter(name_ + ".acc_lim_y", 0.8);
  node_->declare_parameter(name_ + ".acc_lim_theta", 2.0);
  node_->declare_parameter(name_ + ".sim_time", 1.5);
  node_->declare_parameter(name_ + ".vx_samples", 10);
  node_->declare_parameter(name_ + ".vy_samples", 5);
  node_->declare_parameter(name_ + ".vtheta_samples", 20);
  node_->declare_parameter(name_ + ".dt", 0.1);
  node_->declare_parameter(name_ + ".path_distance_bias", 32.0);
  node_->declare_parameter(name_ + ".goal_distance_bias", 24.0);
  node_->declare_parameter(name_ + ".obstacle_cost_bias", 0.01);

  max_vel_x_ = node_->get_parameter(name_ + ".max_vel_x").as_double();
  max_vel_y_ = node_->get_parameter(name_ + ".max_vel_y").as_double();
  max_vel_theta_ = node_->get_parameter(name_ + ".max_vel_theta").as_double();
  min_vel_x_ = node_->get_parameter(name_ + ".min_vel_x").as_double();
  acc_lim_x_ = node_->get_parameter(name_ + ".acc_lim_x").as_double();
  acc_lim_y_ = node_->get_parameter(name_ + ".acc_lim_y").as_double();
  acc_lim_theta_ = node_->get_parameter(name_ + ".acc_lim_theta").as_double();
  sim_time_ = node_->get_parameter(name_ + ".sim_time").as_double();
  vx_samples_ = node_->get_parameter(name_ + ".vx_samples").as_int();
  vy_samples_ = node_->get_parameter(name_ + ".vy_samples").as_int();
  vtheta_samples_ = node_->get_parameter(name_ + ".vtheta_samples").as_int();
  dt_ = node_->get_parameter(name_ + ".dt").as_double();
  path_distance_bias_ = node_->get_parameter(name_ + ".path_distance_bias").as_double();
  goal_distance_bias_ = node_->get_parameter(name_ + ".goal_distance_bias").as_double();
  obstacle_cost_bias_ = node_->get_parameter(name_ + ".obstacle_cost_bias").as_double();

  RCLCPP_INFO(node_->get_logger(), "DWA3D configured");
}

void DWA3D::activate() {}
void DWA3D::deactivate() {}

void DWA3D::cleanup()
{
  global_path_.poses.clear();
  node_.reset();
  costmap_.reset();
}

bool DWA3D::setPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return false;
  }

  global_path_ = path;
  current_goal_ = path.poses.back();
  return true;
}

void DWA3D::computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_vel,
  double & min_vx, double & max_vx,
  double & min_vy, double & max_vy,
  double & min_vtheta, double & max_vtheta) const
{
  // 动态窗口 = 速度限制 ∩ 加速度限制
  min_vx = std::max(min_vel_x_, current_vel.linear.x - acc_lim_x_ * dt_);
  max_vx = std::min(max_vel_x_, current_vel.linear.x + acc_lim_x_ * dt_);

  min_vy = std::max(-max_vel_y_, current_vel.linear.y - acc_lim_y_ * dt_);
  max_vy = std::min(max_vel_y_, current_vel.linear.y + acc_lim_y_ * dt_);

  min_vtheta = std::max(-max_vel_theta_, current_vel.angular.z - acc_lim_theta_ * dt_);
  max_vtheta = std::min(max_vel_theta_, current_vel.angular.z + acc_lim_theta_ * dt_);
}

Trajectory DWA3D::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double vx, double vy, double vtheta) const
{
  Trajectory traj;
  traj.vx = vx;
  traj.vy = vy;
  traj.vtheta = vtheta;

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double z = current_pose.pose.position.z;
  double yaw = tf2::getYaw(current_pose.pose.orientation);

  int steps = static_cast<int>(sim_time_ / dt_);
  for (int i = 0; i < steps; ++i) {
    x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt_;
    y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt_;
    yaw += vtheta * dt_;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = current_pose.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    traj.poses.push_back(pose);
  }

  return traj;
}

double DWA3D::pathDistanceCost(const Trajectory & traj) const
{
  if (traj.poses.empty() || global_path_.poses.empty()) return 0.0;

  // 轨迹末端到最近路径点的距离
  const auto & end = traj.poses.back();
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & p : global_path_.poses) {
    double dx = end.pose.position.x - p.pose.position.x;
    double dy = end.pose.position.y - p.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

double DWA3D::goalDistanceCost(const Trajectory & traj) const
{
  if (traj.poses.empty()) return 0.0;

  const auto & end = traj.poses.back();
  double dx = end.pose.position.x - current_goal_.pose.position.x;
  double dy = end.pose.position.y - current_goal_.pose.position.y;
  return std::sqrt(dx*dx + dy*dy);
}

double DWA3D::obstacleCost(const Trajectory & traj) const
{
  if (!costmap_) return 0.0;

  double max_cost = 0.0;
  for (const auto & pose : traj.poses) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double z = pose.pose.position.z;

    if (costmap_->isOccupied(x, y, z)) {
      return std::numeric_limits<double>::max();  // 碰撞
    }

    double cost = static_cast<double>(costmap_->getCost(x, y, z));
    max_cost = std::max(max_cost, cost);
  }
  return max_cost;
}

geometry_msgs::msg::TwistStamped DWA3D::computeVelocityCommand(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & current_vel)
{
  geometry_msgs::msg::TwistStamped best_cmd;
  best_cmd.header = current_pose.header;

  // 计算动态窗口
  double min_vx, max_vx, min_vy, max_vy, min_vtheta, max_vtheta;
  computeDynamicWindow(current_vel, min_vx, max_vx, min_vy, max_vy, min_vtheta, max_vtheta);

  double best_cost = std::numeric_limits<double>::max();

  // 速度空间采样
  double dvx = (vx_samples_ > 1) ? (max_vx - min_vx) / (vx_samples_ - 1) : 0.0;
  double dvy = (vy_samples_ > 1) ? (max_vy - min_vy) / (vy_samples_ - 1) : 0.0;
  double dvtheta = (vtheta_samples_ > 1) ? (max_vtheta - min_vtheta) / (vtheta_samples_ - 1) : 0.0;

  for (int i = 0; i < vx_samples_; ++i) {
    double vx = min_vx + i * dvx;
    for (int j = 0; j < vy_samples_; ++j) {
      double vy = min_vy + j * dvy;
      for (int k = 0; k < vtheta_samples_; ++k) {
        double vtheta = min_vtheta + k * dvtheta;

        // 前向仿真
        Trajectory traj = simulateTrajectory(current_pose, vx, vy, vtheta);

        // 计算代价
        double obs_cost = obstacleCost(traj);
        if (obs_cost >= std::numeric_limits<double>::max()) continue;  // 碰撞

        double path_cost = pathDistanceCost(traj);
        double goal_cost = goalDistanceCost(traj);

        double total_cost = path_distance_bias_ * path_cost +
                            goal_distance_bias_ * goal_cost +
                            obstacle_cost_bias_ * obs_cost;

        if (total_cost < best_cost) {
          best_cost = total_cost;
          best_cmd.twist.linear.x = vx;
          best_cmd.twist.linear.y = vy;
          best_cmd.twist.angular.z = vtheta;
          best_trajectory_ = traj;
        }
      }
    }
  }

  return best_cmd;
}

bool DWA3D::isGoalReached(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & goal,
  double dist_tolerance,
  double angle_tolerance)
{
  double dx = current_pose.pose.position.x - goal.pose.position.x;
  double dy = current_pose.pose.position.y - goal.pose.position.y;
  double dz = current_pose.pose.position.z - goal.pose.position.z;
  double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (dist > dist_tolerance) return false;

  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double dyaw = current_yaw - goal_yaw;
  // 归一化到 [-π, π]
  while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
  while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

  return std::abs(dyaw) < angle_tolerance;
}

nav_msgs::msg::Path DWA3D::getLocalPlan() const
{
  nav_msgs::msg::Path path;
  if (!best_trajectory_.poses.empty()) {
    path.header = best_trajectory_.poses.front().header;
    path.poses = best_trajectory_.poses;
  }
  return path;
}

}  // namespace pnc_nav_planners

PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::DWA3D, pnc_nav_core::LocalPlannerBase)
