// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_PLANNERS__PATH_TRACKERS__PURE_PURSUIT_3D_HPP_
#define PNC_NAV_PLANNERS__PATH_TRACKERS__PURE_PURSUIT_3D_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/path_tracker_base.hpp"

namespace pnc_nav_planners
{

/**
 * @class PurePursuit3D
 * @brief Pure Pursuit 三维路径跟踪器
 *
 * 经典几何路径跟踪算法的 3D 扩展。
 * 通过前视点 (lookahead point) 计算曲率，输出角速度指令。
 * 前视距离可根据当前速度自适应调整。
 */
class PurePursuit3D : public pnc_nav_core::PathTrackerBase
{
public:
  PurePursuit3D() = default;
  ~PurePursuit3D() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPath(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) override;

  bool isPathCompleted(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double dist_tolerance) override;

  size_t getCurrentWaypointIndex() const override { return current_waypoint_idx_; }
  double getCrossTrackError() const override { return cross_track_error_; }
  double getHeadingError() const override { return heading_error_; }

  std::string getName() const override { return "PurePursuit3D"; }

private:
  // 找前视点
  geometry_msgs::msg::PoseStamped findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double lookahead_dist) const;

  // 计算自适应前视距离
  double computeAdaptiveLookahead(double current_speed) const;

  // 计算曲率
  double computeCurvature(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point) const;

  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;
  std::string name_;

  // 路径
  nav_msgs::msg::Path path_;
  size_t current_waypoint_idx_{0};

  // 跟踪误差
  double cross_track_error_{0.0};
  double heading_error_{0.0};

  // 参数
  double lookahead_distance_{0.6};
  double min_lookahead_{0.3};
  double max_lookahead_{1.5};
  double lookahead_gain_{0.5};     // Ld = min_ld + gain * speed
  double max_linear_vel_{0.5};
  double max_angular_vel_{1.0};
  double min_linear_vel_{0.05};
};

}  // namespace pnc_nav_planners

#endif  // PNC_NAV_PLANNERS__PATH_TRACKERS__PURE_PURSUIT_3D_HPP_
