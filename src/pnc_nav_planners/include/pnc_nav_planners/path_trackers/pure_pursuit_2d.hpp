#ifndef PNC_NAV_PLANNERS__PATH_TRACKERS__PURE_PURSUIT_3D_HPP_
#define PNC_NAV_PLANNERS__PATH_TRACKERS__PURE_PURSUIT_3D_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/path_tracker_base.hpp"

namespace pnc_nav_planners
{

/**
 * @class PurePursuit2D
 * @brief Pure Pursuit 2D路径跟踪器
 *
 * 经典几何路径跟踪算法的 2D 扩展。
 * 通过前视点 (lookahead point) 计算曲率，输出角速度指令。
 * 前视距离可根据当前速度自适应调整。
 */
class PurePursuit2D : public pnc_nav_core::PathTrackerBase
{
public:
  PurePursuit2D() = default;
  ~PurePursuit2D() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
// 设置路径
  bool setPath(const nav_msgs::msg::Path & path) override;
// 计算速度指令
  geometry_msgs::msg::TwistStamped computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) override;
// 判断路径是否完成
  bool isPathCompleted(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double dist_tolerance) override;
// 获取当前路径点索引和误差
  size_t getCurrentWaypointIndex() const override { return current_waypoint_idx_; }
  // 获取横向误差
  double getCrossTrackError() const override { return cross_track_error_; }
  // 计算航向误差
  double getHeadingError() const override { return heading_error_; }

  std::string getName() const override { return "PurePursuit2D"; }

private:
  // 找前视点
  geometry_msgs::msg::PoseStamped findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double lookahead_dist);

  // 计算自适应前视距离
  double computeAdaptiveLookahead(double current_speed) const;

  // 计算曲率
  double computeCurvature(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;
  std::string name_;

  // 路径
  nav_msgs::msg::Path path_;
  size_t current_waypoint_idx_{0};// 当前路径点索引

  // 跟踪误差
  double cross_track_error_{0.0};// 横向误差
  double heading_error_{0.0};// 航向误差

  // 参数
  double lookahead_distance_{0.6};// 固定前视距离
  double min_lookahead_{0.3};// 最小前视距离
  double max_lookahead_{1.5};// 最大前视距离
  double lookahead_gain_{0.5};     // Ld = min_ld + gain * speed  // 自适应前视距离增益
  double max_linear_vel_{0.5};// 最大线速度
  double max_angular_vel_{1.0};// 最大角速度
  double min_linear_vel_{0.05};// 最小线速度
};

}  // namespace pnc_nav_planners

#endif  // PNC_NAV_PLANNERS__PATH_TRACKERS__PURE_PURSUIT_3D_HPP_
