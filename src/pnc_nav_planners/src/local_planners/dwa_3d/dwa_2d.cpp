
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
  node_->declare_parameter(name_ + ".heading_distance_bias", 15.0);

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
  heading_distance_bias_ = node_->get_parameter(name_ + ".heading_distance_bias").as_double();

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
  rebuildPathSamples();
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
            RCLCPP_DEBUG(node_->get_logger(), "与目标距离差达到");
            if(std::abs(dtheta) <= angle_tolerance)
            {
                RCLCPP_DEBUG(node_->get_logger(), "与目标角度差达到");
                return true;
            }
            RCLCPP_DEBUG(node_->get_logger(), "与目标角度差未达到");
            return false;
        }
        RCLCPP_DEBUG(node_->get_logger(), "与目标距离差未达到");
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
  const double invalid_cost = std::numeric_limits<double>::max();
  if (traj.poses.empty() || !costmap_) {
    return invalid_cost;
  }

  const double path_cost = pathDistanceCost(traj);
  const double goal_cost = goalDistanceCost(traj);
  const double obstacle_cost = obstacleCost(traj);
  const double heading_cost = headingCost(traj);
  if (path_cost == invalid_cost || goal_cost == invalid_cost || obstacle_cost == invalid_cost) {
    return invalid_cost;
  }

  double total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ *
    goal_cost + obstacle_cost_bias_ * obstacle_cost +
    heading_distance_bias_ * heading_cost;
  // 调试：记录最优分量
  debug_last_path_cost_ = path_cost;
  debug_last_goal_cost_ = goal_cost;
  debug_last_obstacle_cost_ = obstacle_cost;
  debug_last_heading_cost_ = heading_cost;
  debug_last_total_ = total_cost;
  return total_cost;
}

void DWA2D::rebuildPathSamples()
{
  path_s_.assign(global_path_.poses.size(), 0.0);
  path_length_ = 0.0;
  for (size_t i = 1; i < global_path_.poses.size(); ++i) {
    const auto & a = global_path_.poses[i - 1].pose.position;
    const auto & b = global_path_.poses[i].pose.position;
    path_length_ += std::hypot(b.x - a.x, b.y - a.y);
    path_s_[i] = path_length_;
  }
}

bool DWA2D::projectOnPath(
  double x, double y, double & s, double & lateral,
  double & dir_x, double & dir_y) const
{
  if (global_path_.poses.empty()) {
    return false;
  }
  if (global_path_.poses.size() == 1) {
    const auto & p = global_path_.poses.front().pose.position;
    lateral = std::hypot(x - p.x, y - p.y);
    s = 0.0;
    dir_x = 1.0;
    dir_y = 0.0;
    return true;
  }

  double best = std::numeric_limits<double>::max();
  bool found = false;
  for (size_t i = 0; i + 1 < global_path_.poses.size(); ++i) {
    const auto & a = global_path_.poses[i].pose.position;
    const auto & b = global_path_.poses[i + 1].pose.position;
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len = std::hypot(vx, vy);
    if (len < 1e-6) {
      continue;
    }
    const double t = std::clamp(((x - a.x) * vx + (y - a.y) * vy) / (len * len), 0.0, 1.0);
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double d = std::hypot(x - px, y - py);
    if (d < best) {
      best = d;
      lateral = d;
      s = path_s_[i] + t * len;
      dir_x = vx / len;
      dir_y = vy / len;
      found = true;
    }
  }
  return found;
}

bool DWA2D::pointAtArcLength(double s, geometry_msgs::msg::Point & out) const
{
  if (global_path_.poses.empty()) {
    return false;
  }
  if (global_path_.poses.size() == 1) {
    out = global_path_.poses.front().pose.position;
    return true;
  }
  const double target = std::clamp(s, 0.0, path_length_);
  for (size_t i = 0; i + 1 < global_path_.poses.size(); ++i) {
    if (path_s_[i + 1] >= target) {
      const auto & a = global_path_.poses[i].pose.position;
      const auto & b = global_path_.poses[i + 1].pose.position;
      const double seg_len = path_s_[i + 1] - path_s_[i];
      const double t = seg_len > 1e-6 ? (target - path_s_[i]) / seg_len : 0.0;
      out.x = a.x + t * (b.x - a.x);
      out.y = a.y + t * (b.y - a.y);
      out.z = 0.0;
      return true;
    }
  }
  out = global_path_.poses.back().pose.position;
  return true;
}

double DWA2D::pathDistanceCost(const Trajectory & traj) const
{
  if (traj.poses.empty() || global_path_.poses.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto & endpoint = traj.poses.back().pose.position;
  double s = 0.0;
  double lateral = 0.0;
  double dx = 1.0;
  double dy = 0.0;
  if (!projectOnPath(endpoint.x, endpoint.y, s, lateral, dx, dy)) {
    return std::numeric_limits<double>::max();
  }
  // 关键修复（KI-013）：代价用"到路径折线的横向偏差"，而不是"到最近离散路径点的距离"。
  // 量到离散点时，轨迹终点落在两点之间会凭空多出最多半个点距的代价，
  // 乘上 path_distance_bias 后足以让"原地不动"比"沿路径前进"更划算。
  return lateral;
}
  double DWA2D::goalDistanceCost(const Trajectory & traj) const
  {
    if(traj.poses.empty())return std::numeric_limits<double>::max();
    const auto & e = traj.poses.back().pose.position;

    // 有路径时用"沿路径的剩余里程"，这样沿路径前进一定单调下降；
    // 直接用"到目标的直线距离"会在路径绕行时和 path_cost 打架（KI-013）。
    double s = 0.0;
    double lateral = 0.0;
    double dx_dir = 1.0;
    double dy_dir = 0.0;
    if (path_length_ > 1e-6 &&
      projectOnPath(e.x, e.y, s, lateral, dx_dir, dy_dir))
    {
      return std::max(0.0, path_length_ - s);
    }

    double dx = e.x - current_goal_.pose.position.x;
    double dy = e.y - current_goal_.pose.position.y;
    return std::hypot(dx,dy);
  }
  double DWA2D::obstacleCost(const Trajectory & traj) const
  { double max_cost = 0.0;
    if(!costmap_ || traj.poses.empty())return std::numeric_limits<double>::max();
    for(auto & obs :traj.poses)
    {
      if(costmap_->isOccupied(obs.pose.position.x,obs.pose.position.y,0.0))
      {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 3000,
          "DWA2D: traj blocked at (%.2f, %.2f) cost=%.0f",
          obs.pose.position.x, obs.pose.position.y,
          costmap_->getCost(obs.pose.position.x, obs.pose.position.y, 0.0));
        return std::numeric_limits<double>::max();
      }

      double cost = costmap_->getCost(obs.pose.position.x,obs.pose.position.y,0.0);
      max_cost = std::max(max_cost,cost);
    }
    return max_cost;
  }
  // 目标朝向代价：远处用"到目标的方向角"（驱动机器人朝目标走），
  // 近处（<0.3m）切换为"goal yaw 角度差"（驱动转正朝向，满足 isGoalReached）
  double DWA2D::headingCost(const Trajectory & traj) const
  {
    if (traj.poses.empty()) {
      return std::numeric_limits<double>::max();
    }
    const auto & end = traj.poses.back().pose.position;
    const double traj_yaw = tf2::getYaw(traj.poses.back().pose.orientation);

    // 朝向代价跟"路径前视点"，而不是"目标点"：
    // 全局路径绕障时，朝目标点会与路径方向打架（KI-013 的另一半）。
    double s = 0.0;
    double lateral = 0.0;
    double dx_dir = 1.0;
    double dy_dir = 0.0;
    geometry_msgs::msg::Point ahead;
    if (path_length_ > 1e-6 &&
      projectOnPath(end.x, end.y, s, lateral, dx_dir, dy_dir) &&
      pointAtArcLength(s + lookahead_, ahead))
    {
      const double desired = std::atan2(ahead.y - end.y, ahead.x - end.x);
      const double dyaw = desired - traj_yaw;
      return std::abs(std::atan2(std::sin(dyaw), std::cos(dyaw)));
    }

    // 退化：没有路径时按目标方向
    double dx = current_goal_.pose.position.x - end.x;
    double dy = current_goal_.pose.position.y - end.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 0.3) {
      double goal_yaw = tf2::getYaw(current_goal_.pose.orientation);
      double dyaw = goal_yaw - traj_yaw;
      return std::abs(std::atan2(std::sin(dyaw), std::cos(dyaw)));
    }
    double goal_dir = std::atan2(dy, dx);
    double dyaw = goal_dir - traj_yaw;
    return std::abs(std::atan2(std::sin(dyaw), std::cos(dyaw)));
  }
  // 计算速度命令
  geometry_msgs::msg::TwistStamped DWA2D::computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel)
  {
    geometry_msgs::msg::TwistStamped best_cmd;
    best_cmd.header = current_pose.header;

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "DWA2D: pose=(%.2f,%.2f,yaw=%.2f) cost=%.0f occ=%d goal=(%.1f,%.1f) dist=%.2f",
      current_pose.pose.position.x, current_pose.pose.position.y,
      tf2::getYaw(current_pose.pose.orientation),
      costmap_->getCost(current_pose.pose.position.x, current_pose.pose.position.y, 0.0),
      costmap_->isOccupied(current_pose.pose.position.x, current_pose.pose.position.y, 0.0),
      current_goal_.pose.position.x, current_goal_.pose.position.y,
      std::hypot(current_goal_.pose.position.x - current_pose.pose.position.x,
                 current_goal_.pose.position.y - current_pose.pose.position.y));

    double best_cost = std::numeric_limits<double>::max();
    // 诊断：分别记录"停车/原地转"与"前进"两类候选的最优代价（排查只转不走用）
    double best_stop_cost = std::numeric_limits<double>::max();
    double best_move_cost = std::numeric_limits<double>::max();
    // 最优候选的分项代价（用于诊断）
    double best_path_c = 0.0, best_goal_c = 0.0, best_obs_c = 0.0, best_head_c = 0.0;
    bool found_valid_trajectory = false;
    double min_motion_cost = std::numeric_limits<double>::max();
    double min_motion_vx = 0.0, min_motion_vz = 0.0;
    double static_cost = std::numeric_limits<double>::max();
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
          if (vx == 0.0) {
            best_stop_cost = std::min(best_stop_cost, total_cost);
          } else {
            best_move_cost = std::min(best_move_cost, total_cost);
          }
          // 调试：独立统计运动/静止轨迹 cost
          if (vx == 0.0 && vtheta == 0.0) {
            static_cost = total_cost;
          } else if (total_cost < min_motion_cost) {
            min_motion_cost = total_cost;
            min_motion_vx = vx;
            min_motion_vz = vtheta;
          }
          if(total_cost < best_cost)
          {
            best_path_c = debug_last_path_cost_;
            best_goal_c = debug_last_goal_cost_;
            best_obs_c = debug_last_obstacle_cost_;
            best_head_c = debug_last_heading_cost_;
            found_valid_trajectory = true;
            best_cost = total_cost;
            best_cmd.twist.linear.x = vx;
            best_cmd.twist.linear.y = vy;
            best_cmd.twist.angular.z = vtheta;
            best_trajectory_ = traj;
            // 调试：记录最佳运动轨迹（非静止）
            if (vx != 0.0 || vtheta != 0.0) {
              debug_motion_vx_ = vx;
              debug_motion_vz_ = vtheta;
              debug_motion_cost_ = total_cost;
              debug_motion_path_ = debug_last_path_cost_;
              debug_motion_goal_ = debug_last_goal_cost_;
              debug_motion_obs_ = debug_last_obstacle_cost_;
              debug_motion_head_ = debug_last_heading_cost_;
            }
          }
        }
      }
    }

    if (!found_valid_trajectory) {
      best_trajectory_.poses.clear();
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "DWA2D: no valid trajectory found, publishing zero velocity");
    } else {
      // stop/move 的初值是 double::max（表示"本轮没采到该类候选"），打印成 -1 避免误读
      const auto as_dbg = [](double v) {
        return (v == std::numeric_limits<double>::max()) ? -1.0 : v;
      };
      const double static_cost_dbg = as_dbg(static_cost);
      const double stop_cost_dbg = as_dbg(best_stop_cost);
      const double move_cost_dbg = as_dbg(best_move_cost);
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "DWA2D: cmd=(%.4f, %.4f) best=%.3f stop=%.3f move=%.3f "
        "path=%.2f goal=%.2f obs=%.1f head=%.2f yaw=%.2f",
        best_cmd.twist.linear.x, best_cmd.twist.angular.z, best_cost,
        stop_cost_dbg, move_cost_dbg,
        best_path_c, best_goal_c, best_obs_c, best_head_c,
        tf2::getYaw(current_pose.pose.orientation));
    }

    return best_cmd;
  }
}


PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::DWA2D, pnc_nav_core::LocalPlannerBase)
