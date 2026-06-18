
// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/local_planners/dwa_2d.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
namespace pnc_nav_planners
{

namespace
{

double sampleValue(double min_value, double max_value, int samples, int index)
{
  if (samples <= 1) {
    return 0.5 * (min_value + max_value);
  }
  return min_value + (max_value - min_value) * index / static_cast<double>(samples - 1);
}

}  // namespace

void DWA2D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap)
{
  node_ = node;
  name_ = name;
  costmap_ = costmap;

  node_->declare_parameter(name_ + ".max_vel_x", 0.4);
  node_->declare_parameter(name_ + ".max_vel_y", 0.0);
  node_->declare_parameter(name_ + ".max_vel_theta", 1.0);
  node_->declare_parameter(name_ + ".min_vel_x", 0.0);
  node_->declare_parameter(name_ + ".acc_lim_x", 0.8);
  node_->declare_parameter(name_ + ".acc_lim_y", 0.0);
  node_->declare_parameter(name_ + ".acc_lim_theta", 1.5);
  node_->declare_parameter(name_ + ".sim_time", 1.2);
  node_->declare_parameter(name_ + ".vx_samples", 8);
  node_->declare_parameter(name_ + ".vy_samples", 1);
  node_->declare_parameter(name_ + ".vtheta_samples", 15);
  node_->declare_parameter(name_ + ".dt", 0.1);
  node_->declare_parameter(name_ + ".path_distance_bias", 32.0);
  node_->declare_parameter(name_ + ".goal_distance_bias", 24.0);
  node_->declare_parameter(name_ + ".obstacle_cost_bias", 0.02);

  max_vel_x_ = node_->get_parameter(name_ + ".max_vel_x").as_double();
  max_vel_y_ = node_->get_parameter(name_ + ".max_vel_y").as_double();
  max_vel_theta_ = node_->get_parameter(name_ + ".max_vel_theta").as_double();
  min_vel_x_ = node_->get_parameter(name_ + ".min_vel_x").as_double();
  acc_lim_x_ = node_->get_parameter(name_ + ".acc_lim_x").as_double();
  acc_lim_y_ = node_->get_parameter(name_ + ".acc_lim_y").as_double();
  acc_lim_theta_ = node_->get_parameter(name_ + ".acc_lim_theta").as_double();
  sim_time_ = node_->get_parameter(name_ + ".sim_time").as_double();
  vx_samples_ = std::max(1, static_cast<int>(node_->get_parameter(name_ + ".vx_samples").as_int()));
  vy_samples_ = std::max(1, static_cast<int>(node_->get_parameter(name_ + ".vy_samples").as_int()));
  vtheta_samples_ = std::max(
    1, static_cast<int>(node_->get_parameter(name_ + ".vtheta_samples").as_int()));
  dt_ = std::max(0.01, node_->get_parameter(name_ + ".dt").as_double());
  path_distance_bias_ = node_->get_parameter(name_ + ".path_distance_bias").as_double();
  goal_distance_bias_ = node_->get_parameter(name_ + ".goal_distance_bias").as_double();
  obstacle_cost_bias_ = node_->get_parameter(name_ + ".obstacle_cost_bias").as_double();

  RCLCPP_INFO(node_->get_logger(), "DWA2D parameters declared");
}

void DWA2D::activate()
{
  RCLCPP_INFO(node_->get_logger(), "DWA2D activated");
}

void DWA2D::deactivate()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "DWA2D deactivated");
  }
}

void DWA2D::cleanup()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "DWA2D cleaned up");
    RCLCPP_INFO(node_->get_logger(), "DWA2D cleanup completed");
  }
  global_path_.poses.clear();
  best_trajectory_.poses.clear();
  node_.reset();
  costmap_.reset();
}

bool DWA2D::setPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty path");
    return false;
  }
  global_path_ = path;
  current_goal_ = path.poses.back();
  best_trajectory_.poses.clear();
  return true;
}

// 判断是否到达目标
  bool DWA2D::isGoalReached(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal,
    double dist_tolerance,
    double angle_tolerance) 
    {   // 先做异常处理,然后计算距离和角度差,最后判断是否在容差范围内
        if(global_path_.poses.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "No global path set");
            return false;
        }
        double dx = goal.pose.position.x - current_pose.pose.position.x;
        double dy = goal.pose.position.y - current_pose.pose.position.y;
        double distance = std::hypot(dx,dy);
        double goal_yaw = tf2::getYaw(goal.pose.orientation);
        double current_yaw = tf2::getYaw(current_pose.pose.orientation);
        double dtheta = goal_yaw - current_yaw;
        dtheta = std::atan2(std::sin(dtheta),std::cos(dtheta));
        if(distance<=dist_tolerance)
        {
            RCLCPP_INFO(node_->get_logger(),"与目标距离差达到");
            if(std::abs(dtheta) <= angle_tolerance)
            {
                RCLCPP_INFO(node_->get_logger(),"与目标角度差达到");
                return true;
            }
            RCLCPP_INFO(node_->get_logger(),"与目标角度差未达到");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),"与目标距离差未达到");
        return false;
    }



nav_msgs::msg::Path DWA2D::getLocalPlan() const
{
  nav_msgs::msg::Path local_plan;
  if (!best_trajectory_.poses.empty()) {
    local_plan.header = best_trajectory_.poses.front().header;
    local_plan.poses = best_trajectory_.poses;
  }
  return local_plan;
}

void DWA2D::computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_vel,
  double & min_vx, double & max_vx,
  double & min_vy, double & max_vy,
  double & min_vtheta, double & max_vtheta) const
{
  min_vx = std::max(min_vel_x_, current_vel.linear.x - acc_lim_x_ * dt_);
  max_vx = std::min(max_vel_x_, current_vel.linear.x + acc_lim_x_ * dt_);
  min_vy = std::max(-max_vel_y_, current_vel.linear.y - acc_lim_y_ * dt_);
  max_vy = std::min(max_vel_y_, current_vel.linear.y + acc_lim_y_ * dt_);
  min_vtheta = std::max(-max_vel_theta_, current_vel.angular.z - acc_lim_theta_ * dt_);
  max_vtheta = std::min(max_vel_theta_, current_vel.angular.z + acc_lim_theta_ * dt_);
}

  // 前向仿真生成轨迹
  Trajectory DWA2D::simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double vx, double vy, double vtheta) const
    {
        Trajectory traj;
        traj.vx = vx;
        traj.vy = vy;
        traj.vtheta = vtheta;

        double x = current_pose.pose.position.x;
        double y = current_pose.pose.position.y;
        double yaw = tf2::getYaw(current_pose.pose.orientation);

        int steps = static_cast<int>(sim_time_/dt_);
        for(int i = 0; i < steps; i++)
        {
            x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt_;
            y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt_;
            yaw+= vtheta * dt_;
            tf2::Quaternion q;
            q.setRPY(0,0, yaw);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = current_pose.header;
            pose.pose.position.x = x;
            pose.pose.position.y= y;
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w(); 
            traj.poses.push_back(pose);
        }

        return traj;
    }
double DWA2D::computeCost(const Trajectory & traj) const
{
  if (traj.poses.empty()) {
    return std::numeric_limits<double>::max();
  }
  const double obstacle_cost = obstacleCost(traj);
  if (!std::isfinite(obstacle_cost)) {
    return obstacle_cost;
  }
    double total_cost = path_distance_bias_ * pathDistanceCost(traj) + goal_distance_bias_ * 
  goalDistanceCost(traj) + obstacle_cost_bias_ * obstacle_cost;
  return total_cost;
}

double DWA2D::pathDistanceCost(const Trajectory & traj) const
{
  if (traj.poses.empty() || global_path_.poses.empty()) {
    return std::numeric_limits<double>::max();
  }

  double min_cost = std::numeric_limits<double>::max();
  const auto & endpoint = traj.poses.back();

  for (const auto & path_pose : global_path_.poses) {
      double dx = endpoint.pose.position.x - path_pose.pose.position.x;
      double dy = endpoint.pose.position.y - path_pose.pose.position.y;
      double cost = std::hypot(dx,dy);
      min_cost = std::min(min_cost,cost);
  }
  return min_cost;
}
  double DWA2D::goalDistanceCost(const Trajectory & traj) const
  {
    if(traj.poses.empty())return 0.0;
    double dx = traj.poses.back().pose.position.x - current_goal_.pose.position.x;
    double dy = traj.poses.back().pose.position.y - current_goal_.pose.position.y;
    double distans = std::hypot(dx,dy);
    return distans;
  }
  double DWA2D::obstacleCost(const Trajectory & traj) const
  { double max_cost = 0.0;
    if(!costmap_)return 0.0;
    for(auto & obs :traj.poses)
    {
      if(costmap_->isOccupied(obs.pose.position.x,obs.pose.position.y,0.0))
      {
        return std::numeric_limits<double>::max();
      }

      double cost = costmap_->getCost(obs.pose.position.x,obs.pose.position.y,0.0);
      max_cost = std::max(max_cost,cost);
    }
    return max_cost;
  }
  // 计算速度命令
  geometry_msgs::msg::TwistStamped DWA2D::computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) 
    {
      geometry_msgs::msg::TwistStamped best_cmd;
      best_cmd.header = current_pose.header;
  double best_cost = std::numeric_limits<double>::max();
  double min_vx, max_vx, min_vy, max_vy, min_vtheta, max_vtheta;
  computeDynamicWindow(current_vel, min_vx, max_vx, min_vy, max_vy, min_vtheta, max_vtheta);
  for(int i=0; i < vx_samples_ ; i++)
  {
    double vx = sampleValue(min_vx, max_vx, vx_samples_, i);
    for(int j =0; j< vy_samples_ ;j++)
    {
      double vy = sampleValue(min_vy, max_vy, vy_samples_, j);
      for(int k=0;k<vtheta_samples_ ;k++)
      {
        double vtheta = sampleValue(min_vtheta, max_vtheta, vtheta_samples_, k);
        auto traj = simulateTrajectory(current_pose,vx,vy,vtheta);
        double total_cost = computeCost(traj);
        if(total_cost < best_cost)
        {
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
}


PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::DWA2D, pnc_nav_core::LocalPlannerBase)
