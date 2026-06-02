// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_PLANNERS__LOCAL_PLANNERS__DWA_3D_HPP_
#define PNC_NAV_PLANNERS__LOCAL_PLANNERS__DWA_3D_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pnc_nav_core/local_planner_base.hpp"

namespace pnc_nav_planners
{

/**
 * @struct Trajectory
 * @brief 候选轨迹
 */
struct Trajectory
{
  double vx{0.0};
  double vy{0.0};
  double vtheta{0.0};
  double cost{0.0};
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

/**
 * @class DWA3D
 * @brief Dynamic Window Approach 三维局部规划器
 *
 * 在速度空间中采样，前向仿真生成候选轨迹，
 * 通过多目标代价函数选择最优轨迹。
 * 3D 扩展：考虑地形坡度和高度变化。
 */
class DWA3D : public pnc_nav_core::LocalPlannerBase
{
public:
  DWA3D() = default;
  ~DWA3D() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPath(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) override;

  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal,
    double dist_tolerance,
    double angle_tolerance) override;

  nav_msgs::msg::Path getLocalPlan() const override;

  std::string getName() const override { return "DWA3D"; }

private:
  // 计算动态窗口
  void computeDynamicWindow(
    const geometry_msgs::msg::Twist & current_vel,
    double & min_vx, double & max_vx,
    double & min_vy, double & max_vy,
    double & min_vtheta, double & max_vtheta) const;

  // 前向仿真生成轨迹
  Trajectory simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double vx, double vy, double vtheta) const;

  // 代价函数
  double pathDistanceCost(const Trajectory & traj) const;
  double goalDistanceCost(const Trajectory & traj) const;
  double obstacleCost(const Trajectory & traj) const;

  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<pnc_nav_core::CostmapInterface> costmap_;

  // 当前全局路径和目标
  nav_msgs::msg::Path global_path_;
  geometry_msgs::msg::PoseStamped current_goal_;

  // 最优轨迹（用于可视化）
  Trajectory best_trajectory_;

  // 速度限制
  double max_vel_x_{0.5};
  double max_vel_y_{0.3};
  double max_vel_theta_{1.0};
  double min_vel_x_{-0.1};

  // 加速度限制
  double acc_lim_x_{1.0};
  double acc_lim_y_{0.8};
  double acc_lim_theta_{2.0};

  // 采样参数
  double sim_time_{1.5};
  int vx_samples_{10};
  int vy_samples_{5};
  int vtheta_samples_{20};
  double dt_{0.1};

  // 代价权重
  double path_distance_bias_{32.0};
  double goal_distance_bias_{24.0};
  double obstacle_cost_bias_{0.01};
};

}  // namespace pnc_nav_planners

#endif  // PNC_NAV_PLANNERS__LOCAL_PLANNERS__DWA_3D_HPP_
