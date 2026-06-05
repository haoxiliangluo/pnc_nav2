// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_PLANNERS__PATH_TRACKERS__STANLEY_3D_HPP_
#define PNC_NAV_PLANNERS__PATH_TRACKERS__STANLEY_3D_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/path_tracker_base.hpp"

namespace pnc_nav_planners
{

/**
 * @class Stanley3D
 * @brief Stanley 三维路径跟踪器
 *
 * 基于前轮反馈的路径跟踪算法。
 * 同时修正航向误差和横向误差，适合中高速场景。
 */
class Stanley3D : public pnc_nav_core::PathTrackerBase
{
public:
  Stanley3D() = default;
  ~Stanley3D() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  bool setPath(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) override;

  bool isPathCompleted(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double dist_tolerance) override;

  size_t getCurrentWaypointIndex() const override { return current_waypoint_idx_; }
  double getCrossTrackError() const override { return cross_track_error_; }
  double getHeadingError() const override { return heading_error_; }

  std::string getName() const override { return "Stanley3D"; }

private:
  // 找最近路径点
  size_t findClosestPoint(const geometry_msgs::msg::PoseStamped & pose) const;

  rclcpp::Node::SharedPtr node_;
  std::string name_;

  nav_msgs::msg::Path path_;
  size_t current_waypoint_idx_{0};

  double cross_track_error_{0.0};
  double heading_error_{0.0};

  // 参数
  double k_cross_track_{1.0};    // 横向误差增益
  double k_heading_{1.0};        // 航向误差增益
  double k_soft_{1.0};           // 软化系数（防止低速除零）
  double max_steer_{0.8};        // 最大转向角 (rad)
  double max_linear_vel_{0.5};
  double min_linear_vel_{0.05};
};

}  // namespace pnc_nav_planners

#endif  // PNC_NAV_PLANNERS__PATH_TRACKERS__STANLEY_3D_HPP_
