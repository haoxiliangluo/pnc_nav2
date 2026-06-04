// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__LOCAL_PLANNER_BASE_HPP_
#define PNC_NAV_CORE__LOCAL_PLANNER_BASE_HPP_

#include <memory>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pnc_nav_core/costmap_interface.hpp"

namespace pnc_nav_core
{

/**
 * @class LocalPlannerBase
 * @brief 局部规划器插件基类
 *
 * 负责根据全局路径和当前状态计算局部速度指令，实现避障。
 * 可实现 DWA、TEB、MPC、DRL 等算法。
 */
class LocalPlannerBase
{
public:
  using SharedPtr = std::shared_ptr<LocalPlannerBase>;

  virtual ~LocalPlannerBase() = default;

  /**
   * @brief 配置局部规划器
   * @param node ROS节点指针
   * @param name 规划器实例名称
   * @param costmap 代价地图接口
   */
  virtual void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<CostmapInterface> & costmap) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() = 0;

  /**
   * @brief 设置全局路径（由全局规划器提供）
   * @param path 全局路径
   */
  virtual void setPath(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief 计算速度指令
   * @param current_pose 当前位姿
   * @param current_vel 当前速度
   * @return 速度指令（带时间戳）
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) = 0;

  /**
   * @brief 判断是否到达目标
   * @param current_pose 当前位姿
   * @param goal 目标位姿
   * @param dist_tolerance 距离容差 (m)
   * @param angle_tolerance 角度容差 (rad)
   */
  virtual bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal,
    double dist_tolerance = 0.1,
    double angle_tolerance = 0.1) = 0;

  /**
   * @brief 获取局部规划的轨迹（用于可视化）
   */
  virtual nav_msgs::msg::Path getLocalPlan() const { return nav_msgs::msg::Path(); }

  virtual std::string getName() const = 0;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__LOCAL_PLANNER_BASE_HPP_
