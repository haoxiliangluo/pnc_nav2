// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__PATH_TRACKER_BASE_HPP_
#define PNC_NAV_CORE__PATH_TRACKER_BASE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace pnc_nav_core
{

/**
 * @class PathTrackerBase
 * @brief 路径跟踪控制器插件基类
 *
 * 负责跟踪局部/全局路径，输出速度指令。
 * 与 LocalPlanner 的区别：PathTracker 不做避障，只做路径跟踪控制。
 * 可实现 Pure Pursuit、Stanley、MPC Tracker 等算法。
 */
class PathTrackerBase
{
public:
  using SharedPtr = std::shared_ptr<PathTrackerBase>;

  virtual ~PathTrackerBase() = default;

  /**
   * @brief 配置路径跟踪器
   * @param node ROS节点指针
   * @param name 跟踪器实例名称
   */
  virtual void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() = 0;

  /**
   * @brief 设置要跟踪的路径
   * @param path 目标路径
   */
  virtual bool setPath(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief 计算跟踪速度指令
   * @param current_pose 当前位姿
   * @param current_vel 当前速度
   * @return 速度指令
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::Twist & current_vel) = 0;

  /**
   * @brief 判断是否完成路径跟踪
   * @param current_pose 当前位姿
   * @param dist_tolerance 距离容差 (m)
   */
  virtual bool isPathCompleted(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double dist_tolerance = 0.1) = 0;

  /**
   * @brief 获取当前跟踪的路径点索引
   */
  virtual size_t getCurrentWaypointIndex() const = 0;

  /**
   * @brief 获取横向跟踪误差 (cross-track error)
   */
  virtual double getCrossTrackError() const { return 0.0; }

  /**
   * @brief 获取航向误差
   */
  virtual double getHeadingError() const { return 0.0; }

  virtual std::string getName() const = 0;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__PATH_TRACKER_BASE_HPP_
