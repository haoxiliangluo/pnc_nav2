
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


  void DWA2D::configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap) 
{
    node_ = node;
    name_ = name;
    costmap_ = costmap;

    node_->declare_parameter(name_ + ".max_vel_x", 1.0);
    node_->declare_parameter(name_ + ".max_vel_y", 0.6);
    node_->declare_parameter(name_ + ".max_vel_theta", 2.0);
    node_->declare_parameter(name_ + ".min_vel_x", -0.1);
    node_->declare_parameter(name_ + ".acc_lim_x", 1.0);
    node_->declare_parameter(name_ + ".acc_lim_y", 0.8);
    node_->declare_parameter(name_ + ".acc_lim_theta", 2.0);
    node_->declare_parameter(name_ + ".sim_time", 1.0);
    node_->declare_parameter(name_ + ".vx_samples", 20);
    node_->declare_parameter(name_ + ".vy_samples", 10);
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

    RCLCPP_INFO(node_->get_logger(), "DWA2D parameters declared");


}
  void DWA2D::activate() 
  {
    RCLCPP_INFO(node_->get_logger(), "DWA2D activated");

  }
  void DWA2D::deactivate()
  {
    RCLCPP_INFO(node_->get_logger(), "DWA2D deactivated");
  }
  void DWA2D::cleanup() 
  {
    RCLCPP_INFO(node_->get_logger(), "DWA2D cleaned up");
    global_path_.poses.clear();
    //best_trajectory_.poses.clear();
    node_.reset();
    costmap_.reset();
    RCLCPP_INFO(node_->get_logger(), "DWA2D cleanup completed");
  }

  bool DWA2D::setPath(const nav_msgs::msg::Path & path) 
  {
    if(path.poses.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Received empty path");
        return false;
    }
    global_path_ = path;
    current_goal_ = path.poses.back();
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
  }
  // 计算动态窗口
  void DWA2D::computeDynamicWindow(
    const geometry_msgs::msg::Twist & current_vel,
    double & min_vx, double & max_vx,
    double & min_vy, double & max_vy,
    double & min_vtheta, double & max_vtheta) const
  { // 动态窗口内的最大最小xy速度,还有角速度,在系统最大最小速度和加速度约束下以及前向仿真时间长度;
//   // 速度限制
//   double max_vel_x_{1.0};
//   double max_vel_y_{0.6};
//   double max_vel_theta_{2.0};
//   double min_vel_x_{-0.1};// 允许后退
//   // 加速度限制
//   double acc_lim_x_{1.0};
//   double acc_lim_y_{0.8};
//   double acc_lim_theta_{2.0};
//   // 采样参数
//   double sim_time_{1.0};// 前向仿真时间长度 (s)
//   int vx_samples_{20};// 前向仿真时间内的速度采样数量
//   int vy_samples_{10};// 侧向速度采样数量
//   int vtheta_samples_{20};// 角速度采样数量
//   double dt_{0.1};// 前向仿真时间步长
    min_vx = std::max(min_vel_x_,current_vel.linear.x - acc_lim_x_ * dt_);
    max_vx = std::min(max_vel_x_,current_vel.linear.x + acc_lim_x_ * dt_);
    min_vy = std::max(-max_vel_y_,current_vel.linear.y - acc_lim_y_ * dt_);
    max_vy = std::min(max_vel_y_,current_vel.linear.y + acc_lim_y_ * dt_);
    min_vtheta = std::max(-max_vel_theta_,current_vel.angular.z - acc_lim_theta_ * dt_);
    max_vtheta = std::min(max_vel_theta_,current_vel.angular.z + acc_lim_theta_ * dt_);
    //这里是不是要有安全避战距离v2=2ax
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
  double DWA2D::computeCost(const Trajectory & traj) const{
    double total_cost = path_distance_bias_ * pathDistanceCost(traj) + goal_distance_bias_ * 
    goalDistanceCost(traj) + obstacle_cost_bias_ * obstacleCost(traj);
    return total_cost;
  }
  // 代价函数
  double DWA2D::pathDistanceCost(const Trajectory & traj) const
  {
    auto psoes = traj.poses.back();
    double min_cost = std::numeric_limits<double>::max();

    for(auto &  tpose:traj.poses)
    {
      double dx = psoes.pose.position.x - tpose.pose.position.x;
      double dy = psoes.pose.position.y - tpose.pose.position.y;
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
        //先计算动态窗口然后前向仿真生成轨迹,对轨迹进行评价函数打分,选择分数最高的一个,最后发布出去
        double min_vx,max_vx,min_vy,max_vy,min_vtheta, max_vtheta;
        computeDynamicWindow(current_vel, min_vx,max_vx,min_vy,max_vy,min_vtheta, max_vtheta);
  //         // 采样参数
  // double sim_time_{1.0};// 前向仿真时间长度 (s)
  // int vx_samples_{20};// 前向仿真时间内的速度采样数量
  // int vy_samples_{10};// 侧向速度采样数量
  // int vtheta_samples_{20};// 角速度采样数量
  // double dt_{0.1};// 前向仿真时间步长
  double dvx = (max_vx - min_vx) / (vx_samples_ - 1);
  double dvy = (max_vy - min_vy) / (vy_samples_ - 1);
  double dvtheta = (max_vtheta - min_vtheta) / (vtheta_samples_ - 1);
  for(int i=0; i < vx_samples_ ; i++)
  {
    double vx = min_vx + dvx * i;
    for(int j =0; j< vy_samples_ ;j++)
    {
      double vy = min_vy + dvy * j;
      for(int k=0;k<vtheta_samples_ ;k++)
      {
        double vtheta = min_vtheta + dvtheta * k;
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
