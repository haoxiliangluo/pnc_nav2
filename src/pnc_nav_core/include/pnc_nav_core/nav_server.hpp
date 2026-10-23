// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__NAV_SERVER_HPP_
#define PNC_NAV_CORE__NAV_SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "pnc_nav_interfaces/action/navigate_to3_d_pose.hpp"

#include "pnc_nav_core/global_planner_base.hpp"
#include "pnc_nav_core/local_planner_base.hpp"
#include "pnc_nav_core/path_tracker_base.hpp"
#include "pnc_nav_core/costmap_interface.hpp"

namespace pnc_nav_core
{

class Nav2CostmapAdapter;
class RollingCostmap;

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
 * @enum NavErrorCode
 * @brief 导航失败原因，直接映射到 NavigateTo3DPose.result.error_code
 */
enum class NavErrorCode : uint16_t
{
  OK = 0,                // 成功
  NO_PATH = 1,           // 全局规划无解
  PLANNING_TIMEOUT = 2,  // 单次规划超时
  MAX_RETRIES = 3,       // 恢复重试次数耗尽
  CANCELLED = 4,         // 被客户端取消 / 被新目标抢占
  TF_FAILURE = 5,        // 长时间拿不到 TF
  INVALID_GOAL = 6,      // 目标非法
  NO_PLANNER = 7         // 插件未加载
};

/// 错误码转字符串（日志/result.message 使用）
const char * toString(NavErrorCode code);

/// 状态转字符串（feedback.current_state 使用，小写）
const char * toString(NavState state);

/**
 * @class NavServer
 * @brief 导航服务器 — 协调全局规划、局部规划、路径跟踪
 *
 * 核心职责：
 * 1. 管理导航状态机（IDLE / PLANNING / FOLLOWING / RECOVERING / SUCCEEDED / FAILED）
 * 2. 通过 pluginlib 加载/切换规划器插件（支持 action.goal.planner_id 热插拔）
 * 3. 对外提供 NavigateTo3DPose action 会话（feedback / cancel / 抢占 / 错误码）
 * 4. 协调规划-控制循环
 *
 * 目标入口：
 * - `navigate_to_3d_pose` action（主入口，带 feedback/result/cancel）
 * - `goal_pose` 话题（兼容入口，等价于发一个不带 planner_id 的 action goal）
 *
 * 注：节点使用单线程执行器（rclcpp::spin），所有回调串行执行，
 *     因此状态机成员不需要额外加锁。
 */
class NavServer : public rclcpp::Node
{
public:
  using NavigateTo3DPose = pnc_nav_interfaces::action::NavigateTo3DPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateTo3DPose>;

  explicit NavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NavServer() override;

  /// 初始化（加载插件、订阅话题、创建 action server）
  void initialize();

  /// 获取当前导航状态
  NavState getState() const { return state_; }

private:
  // --- 状态机 ---
  NavState state_{NavState::IDLE};
  void transitionTo(NavState new_state);// 状态转换，负责日志记录和状态更新

  // --- 插件管理 ---
  pluginlib::ClassLoader<GlobalPlannerBase> global_planner_loader_;
  pluginlib::ClassLoader<LocalPlannerBase> local_planner_loader_;
  pluginlib::ClassLoader<PathTrackerBase> path_tracker_loader_;

  GlobalPlannerBase::SharedPtr global_planner_;
  LocalPlannerBase::SharedPtr local_planner_;
  PathTrackerBase::SharedPtr path_tracker_;
  std::string current_global_plugin_;// 已加载的全局插件类名（switch 时用于短路）

  void loadPlugins();
  bool switchGlobalPlanner(const std::string & plugin_name);
  bool switchLocalPlanner(const std::string & plugin_name);
  bool switchPathTracker(const std::string & plugin_name);

  // --- TF ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;// TF缓存
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;// TF监听器
  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose);// 获取机器人当前位姿
  bool tf_ok_{false};       // 本周期是否成功拿到 TF
  uint16_t tf_fail_count_{0};  // 连续 TF 失败次数

  // --- 代价地图 ---
  std::shared_ptr<Nav2CostmapAdapter> static_costmap_;// 全局：静态 /map（给全局规划器）
  std::shared_ptr<RollingCostmap> local_costmap_;// 局部：滚动窗口 + 障碍层（给局部规划器）
  void updateLocalCostmap();// 每周期：滚动窗口 + 激光障碍更新

  // --- 控制循环 ---
  rclcpp::TimerBase::SharedPtr control_timer_;
  void controlLoop();

  // --- 发布者 ---
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_pub_;
  uint32_t costmap_frame_count_{0};
  void publishCostmap();
  void stopRobot();// 发布零速，保证任何退出路径都先停车

  // --- 订阅者（兼容入口） ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // --- Action 服务（主入口） ---
  rclcpp_action::Server<NavigateTo3DPose>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> goal_handle_;// 当前活动会话（单目标，新目标抢占旧目标）

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateTo3DPose::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  /// 启动一次导航会话（action 与 /goal_pose 共用）
  bool startGoal(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id,
    double max_planning_time);

  /// 结束当前会话：填 result → succeed/abort/canceled → 进入 SUCCEEDED/FAILED
  void finishGoal(NavErrorCode code, const std::string & message);

  /// 发布 feedback（距离/ETA/恢复次数/当前状态）
  void publishFeedback(const geometry_msgs::msg::PoseStamped & pose);

  /// 进入恢复行为（计数 +1，记录开始时间）；重试耗尽时直接结束会话并返回 false
  bool enterRecovery(const std::string & reason);

  // --- 会话状态 ---
  geometry_msgs::msg::PoseStamped current_goal_;
  nav_msgs::msg::Path current_global_path_;
  geometry_msgs::msg::Twist current_velocity_;
  bool goal_active_{false};
  bool cancel_requested_{false};
  uint16_t recovery_count_{0};
  double distance_traveled_{0.0};
  double goal_max_planning_time_{0.0};
  std::string goal_planner_id_;

  // --- 计时（用 steady_clock，避免 sim time 跳变影响超时判断） ---
  std::chrono::steady_clock::time_point goal_start_time_;
  std::chrono::steady_clock::time_point plan_start_time_;
  std::chrono::steady_clock::time_point recovery_start_time_;

  // --- 卡住检测 ---
  geometry_msgs::msg::Point last_progress_pose_;
  std::chrono::steady_clock::time_point last_progress_time_;
  bool has_last_progress_{false};
  geometry_msgs::msg::Point last_position_;
  bool has_last_position_{false};

  // --- 参数 ---
  double control_frequency_{20.0};// 控制循环频率 (Hz)
  double goal_tolerance_dist_{0.15};// 到达目标的距离容差 (m)
  double goal_tolerance_angle_{0.1};// 到达目标的角度容差 (rad)
  int max_planning_retries_{3};// 最大恢复/重规划次数
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};

  double recovery_spin_duration_{1.0};// 恢复时原地旋转时长 (s)
  double recovery_angular_vel_{0.6};// 恢复时旋转角速度 (rad/s)
  double feedback_eta_speed_{0.25};// 计算 ETA 用的参考速度 (m/s)
  double progress_timeout_{8.0};// 卡住判定：多久没有位移算卡住 (s)，<=0 关闭
  double progress_epsilon_{0.05};// 卡住判定：位移阈值 (m)
  int tf_timeout_ticks_{50};// 连续 TF 失败多少周期后判失败
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__NAV_SERVER_HPP_
