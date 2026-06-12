#include "pnc_nav_planners/path_trackers/pure_pursuit_2d.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace pnc_nav_planners
{


  void PurePursuit2D::configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name)
    {
        node_ = node;
        name_ = name;

        node_->declare_parameter(name_ + ".lookahead_distance", 0.6);
        node_->declare_parameter(name_ + ".min_lookahead", 0.6);
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
        RCLCPP_INFO(node_->get_logger(), "PurePursuit2D configured: Ld=%.2f", lookahead_distance_);
    }
  void PurePursuit2D::activate()
  {
    RCLCPP_INFO(node_->get_logger(), "PurePursuit2D activated");
  }
  void PurePursuit2D::deactivate()
  {
    RCLCPP_INFO(node_->get_logger(), "PurePursuit2D deactivated");
  }
  void PurePursuit2D::cleanup()
  {
    path_.poses.clear();
    node_.reset();
  }

// 设置路径
  bool PurePursuit2D::setPath(const nav_msgs::msg::Path & path) 
  {
    if(path.poses.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Received empty path");
        return false;
    }
    path_ = path;
    current_waypoint_idx_ = 0;
    return true;
  }

// 判断路径是否完成
  bool PurePursuit2D::isPathCompleted(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double dist_tolerance)
    {
        double dx = path_.poses.back().pose.position.x - current_pose.pose.position.x;
        double dy = path_.poses.back().pose.position.y - current_pose.pose.position.y;
        double dist_to_goal = std::hypot(dx, dy);
       return current_waypoint_idx_ >= path_.poses.size() ||
             dist_to_goal <= dist_tolerance;
    }


  // 计算自适应前视距离
  double PurePursuit2D::computeAdaptiveLookahead(double current_speed) const
  {
        return std::clamp(min_lookahead_ + lookahead_gain_ * std::abs(current_speed), min_lookahead_, max_lookahead_);
  }

  // 找前视点
  geometry_msgs::msg::PoseStamped PurePursuit2D::findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double lookahead_dist)
    {
        for(size_t i = current_waypoint_idx_; i < path_.poses.size();i++)
        {
            double dx = path_.poses[i].pose.position.x - current_pose.pose.position.x;
            double dy = path_.poses[i].pose.position.y - current_pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if(dist >= lookahead_dist)
            {
                current_waypoint_idx_ = i;
                return path_.poses[i];
            }
        }
        RCLCPP_WARN(node_->get_logger(), "No lookahead point found");
        return path_.poses.back();
    }
// 计算速度指令
  geometry_msgs::msg::TwistStamped PurePursuit2D::computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) 
    {
        //步骤是: 1. 计算自适应前视距离 2. 找前视点 3. 计算曲率 4. 生成速度指令
        double adaptive_lookahead = computeAdaptiveLookahead(current_vel.linear.x);
        geometry_msgs::msg::PoseStamped lookahead_point = findLookaheadPoint(current_pose, adaptive_lookahead);
        double curvature = computeCurvature(current_pose, lookahead_point);
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = node_->now();
        cmd.header.frame_id = current_pose.header.frame_id;
        cmd.twist.linear.x = std::clamp(current_vel.linear.x, min_linear_vel_, max_linear_vel_);
        cmd.twist.angular.z = std::clamp(curvature * cmd.twist.linear.x, -max_angular_vel_, max_angular_vel_);
        return cmd;


    }
  // 计算曲率
  double PurePursuit2D::computeCurvature(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point)
    {//有当前位子,前视点,前瞻距离,然后计算曲率
        double dx = lookahead_point.pose.position.x - current_pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose.pose.position.y;
        double lookahead_dist = std::hypot(dx, dy);
        if(lookahead_dist < 1e-6)
        {
            return 0.0;
        }
        double path_yaw = std::atan2(dy, dx);
        double robot_yaw = tf2::getYaw(current_pose.pose.orientation);
        double delta = path_yaw - robot_yaw;
        while (delta > M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        heading_error_ = delta;
        cross_track_error_ = std::sin(heading_error_) * lookahead_dist;
        return 2.0 * cross_track_error_ / (lookahead_dist * lookahead_dist);
    }

}