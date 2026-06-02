// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__NAV_SERVER_HPP_
#define PNC_NAV_CORE__NAV_SERVER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "pnc_nav_core/global_planner_base.hpp"
#include "pnc_nav_core/local_planner_base.hpp"
#include "pnc_nav_core/path_tracker_base.hpp"
#include "pnc_nav_core/costmap_interface.hpp"

namespace pnc_nav_core
{

/**
 * @enum NavState
 * @brief 导航状态机
 */
enum class NavState
{
  IDLE,           // 空闲
  PLANNING,       // 全局规划中
  FOLLOWING,      // 路径跟踪中
  RECOVERING,     // 恢复行为中
  SUCCEEDED,      // 导航成功
  FAILED          // 导航失败
};

/**
 * @class NavServer
 * @brief 导航服务器 — 协调全局规划、局部规划、路径跟踪
 *
 * 核心职责：
 * 1. 管理导航状态机
 * 2. 通过 pluginlib 加载/切换规划器插件
 * 3. 协调规划-控制循环
 * 4. 提供 Action 接口供上层调用
 */
class NavServer : public rclcpp::Node
{
public:
  explicit NavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NavServer() override;

  /// 初始化（加载插件、订阅话题）
  void initialize();

  /// 获取当前导航状态
  NavState getState() const { return state_; }

private:
  // --- 状态机 ---
  NavState state_{NavState::IDLE};
  void transitionTo(NavState new_state);

  // --- 插件管理 ---
  pluginlib::ClassLoader<GlobalPlannerBase> global_planner_loader_;
  pluginlib::ClassLoader<LocalPlannerBase> local_planner_loader_;
  pluginlib::ClassLoader<PathTrackerBase> path_tracker_loader_;

  GlobalPlannerBase::SharedPtr global_planner_;
  LocalPlannerBase::SharedPtr local_planner_;
  PathTrackerBase::SharedPtr path_tracker_;

  void loadPlugins();
  bool switchGlobalPlanner(const std::string & plugin_name);
  bool switchLocalPlanner(const std::string & plugin_name);
  bool switchPathTracker(const std::string & plugin_name);

  // --- TF ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped getCurrentPose();

  // --- 代价地图 ---
  CostmapInterface::SharedPtr costmap_;

  // --- 控制循环 ---
  rclcpp::TimerBase::SharedPtr control_timer_;
  void controlLoop();

  // --- 发布者 ---
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // --- 订阅者 ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // --- 当前状态 ---
  geometry_msgs::msg::PoseStamped current_goal_;
  nav_msgs::msg::Path current_global_path_;
  geometry_msgs::msg::Twist current_velocity_;

  // --- 参数 ---
  double control_frequency_{20.0};
  double goal_tolerance_dist_{0.15};
  double goal_tolerance_angle_{0.1};
  int max_planning_retries_{3};
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__NAV_SERVER_HPP_
