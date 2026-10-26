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
        node_->declare_parameter(name_ + ".min_lookahead", 0.3);
        node_->declare_parameter(name_ + ".max_lookahead", 1.5);
        node_->declare_parameter(name_ + ".lookahead_gain", 0.5);
        node_->declare_parameter(name_ + ".max_linear_vel", 0.5);
        node_->declare_parameter(name_ + ".max_angular_vel", 1.0);
        node_->declare_parameter(name_ + ".min_linear_vel", 0.05);
        // 插件参数一律挂在 <实例名>. 下（KI-009）
        if (!node_->has_parameter(name_ + ".debug")) {
            node_->declare_parameter(name_ + ".debug", false);
        }

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
      if(path_.poses.empty())return true;

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
        auto dist_to = [&](size_t i) {
              const auto & p = path_.poses[i].pose.position;
              return std::hypot(
                p.x - current_pose.pose.position.x,
                p.y - current_pose.pose.position.y);
            };

        // 单调推进索引：下一个点比当前点更近 → 说明机器人已经越过当前点。
        // 不加这一步，短距离折返时索引会停在路径起点/终点，机器人会追着身后的点跑（KI-020）。
        while (current_waypoint_idx_ + 1 < path_.poses.size() &&
          dist_to(current_waypoint_idx_ + 1) <= dist_to(current_waypoint_idx_))
        {
            ++current_waypoint_idx_;
        }

        for(size_t i = current_waypoint_idx_; i < path_.poses.size();i++)
        {
            if(dist_to(i) >= lookahead_dist)
            {
                current_waypoint_idx_ = i;
                return path_.poses[i];
            }
        }
        // 机器人已经越过整条路径（或路径比前视距离短）：用最后一个点
        current_waypoint_idx_ = path_.poses.size() - 1;
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

        // 大角度偏差（含反向）：先原地转，不要用曲率公式——
        // 前视点在正后方时 sin(heading_error)≈0，会算出"曲率 0"然后直着开走（KI-020 根因）。
        if (std::abs(heading_error_) > 2.0) {
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = (heading_error_ > 0.0 ? 1.0 : -1.0) * max_angular_vel_;
            if (node_->get_parameter(name_ + ".debug").as_bool()) {
                RCLCPP_INFO(
                    node_->get_logger(), "PurePursuit2D: rotate in place (heading_error=%.2f rad)",
                    heading_error_);
            }
            return cmd;
        }
        double  target_speed = max_linear_vel_ / (1.0 + std::abs(curvature) * 2.0); // 根据曲率调整速度
        double dist_to_goal = std::hypot(path_.poses.back().pose.position.x - current_pose.pose.position.x,
                                     path_.poses.back().pose.position.y - current_pose.pose.position.y);
        if(dist_to_goal < lookahead_distance_)
        {
            target_speed = std::max(min_linear_vel_, target_speed * (dist_to_goal / lookahead_distance_)); // 接近目标时减速
        }
        cmd.twist.linear.x = std::clamp(target_speed, min_linear_vel_, max_linear_vel_);
        cmd.twist.angular.z = std::clamp(curvature * cmd.twist.linear.x, -max_angular_vel_, max_angular_vel_);
        if(node_->get_parameter(name_ + ".debug").as_bool())
        {
            RCLCPP_INFO(node_->get_logger(), "Current Waypoint Index: %zu, Lookahead Point: (%.2f, %.2f), Curvature: %.4f, Cross Track Error: %.4f, Heading Error: %.4f",
                        current_waypoint_idx_, lookahead_point.pose.position.x, lookahead_point.pose.position.y,
                        curvature, cross_track_error_, heading_error_);
            RCLCPP_INFO(node_->get_logger(), "Target Speed: %.2f, Angular Velocity: %.2f", cmd.twist.linear.x, cmd.twist.angular.z);
        }
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
        // double delta = path_yaw - robot_yaw;
        // while (delta > M_PI) delta -= 2.0 * M_PI;
        // while (delta < -M_PI) delta += 2.0 * M_PI;
        //double delta = atan2(dy, dx) - tf2::getYaw(current_pose.pose.orientation);
        double delta = std::atan2(std::sin(path_yaw - robot_yaw), std::cos(path_yaw - robot_yaw));
        heading_error_ = delta;// 航向误差
        cross_track_error_ = std::sin(heading_error_) * lookahead_dist;// 横向误差 = 路径与机器人连线的距离 * sin(路径与机器人航向的夹角)
        // return 2.0 * cross_track_error_ / (lookahead_dist * lookahead_dist);
        return std::clamp(2.0 * std::sin(heading_error_) / lookahead_dist, -5.0, 5.0); // 曲率 = 2 * 横向误差 / 前视距离^2
    }

}
PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::PurePursuit2D, pnc_nav_core::PathTrackerBase)
