// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_core/nav_server.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "pnc_nav_core/nav2_costmap_adapter.hpp"
#include "pnc_nav_core/rolling_costmap.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pnc_nav_core
{

namespace
{

/// 两点水平距离
double distance2D(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(
    a.pose.position.x - b.pose.position.x,
    a.pose.position.y - b.pose.position.y);
}

/// 两个 position 的水平距离
double distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double secondsSince(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

const char * toString(NavErrorCode code)
{
  switch (code) {
    case NavErrorCode::OK: return "OK";
    case NavErrorCode::NO_PATH: return "NO_PATH";
    case NavErrorCode::PLANNING_TIMEOUT: return "PLANNING_TIMEOUT";
    case NavErrorCode::MAX_RETRIES: return "MAX_RETRIES";
    case NavErrorCode::CANCELLED: return "CANCELLED";
    case NavErrorCode::TF_FAILURE: return "TF_FAILURE";
    case NavErrorCode::INVALID_GOAL: return "INVALID_GOAL";
    case NavErrorCode::NO_PLANNER: return "NO_PLANNER";
  }
  return "UNKNOWN";
}

const char * toString(NavState state)
{
  switch (state) {
    case NavState::IDLE: return "idle";
    case NavState::PLANNING: return "planning";
    case NavState::FOLLOWING: return "following";
    case NavState::RECOVERING: return "recovering";
    case NavState::SUCCEEDED: return "succeeded";
    case NavState::FAILED: return "failed";
  }
  return "unknown";
}

NavServer::NavServer(const rclcpp::NodeOptions & options)
: Node("nav_server", options),
  global_planner_loader_("pnc_nav_core", "pnc_nav_core::GlobalPlannerBase"),
  local_planner_loader_("pnc_nav_core", "pnc_nav_core::LocalPlannerBase"),
  path_tracker_loader_("pnc_nav_core", "pnc_nav_core::PathTrackerBase")
{
  // 声明参数
  declare_parameter("control_frequency", 20.0);
  declare_parameter("goal_tolerance_dist", 0.15);
  declare_parameter("goal_tolerance_angle", 0.1);
  declare_parameter("max_planning_retries", 3);
  declare_parameter("global_frame", "map");
  declare_parameter("robot_frame", "base_link");

  declare_parameter("global_planner_plugin", "pnc_nav_planners::AStar3D");
  declare_parameter("local_planner_plugin", "");
  declare_parameter("path_tracker_plugin", "pnc_nav_planners::PurePursuit3D");

  // 恢复行为 / 反馈 / 看门狗参数
  declare_parameter("recovery_spin_duration", 1.0);
  declare_parameter("recovery_angular_vel", 0.6);
  declare_parameter("feedback_eta_speed", 0.25);
  declare_parameter("progress_timeout", 8.0);
  declare_parameter("progress_epsilon", 0.05);
  declare_parameter("tf_timeout_ticks", 50);

  // 获取参数
  control_frequency_ = get_parameter("control_frequency").as_double();
  goal_tolerance_dist_ = get_parameter("goal_tolerance_dist").as_double();
  goal_tolerance_angle_ = get_parameter("goal_tolerance_angle").as_double();
  max_planning_retries_ = get_parameter("max_planning_retries").as_int();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_frame").as_string();

  recovery_spin_duration_ = get_parameter("recovery_spin_duration").as_double();
  recovery_angular_vel_ = get_parameter("recovery_angular_vel").as_double();
  feedback_eta_speed_ = get_parameter("feedback_eta_speed").as_double();
  progress_timeout_ = get_parameter("progress_timeout").as_double();
  progress_epsilon_ = get_parameter("progress_epsilon").as_double();
  tf_timeout_ticks_ = get_parameter("tf_timeout_ticks").as_int();

  RCLCPP_INFO(get_logger(), "NavServer created");
}

NavServer::~NavServer()
{
  if (global_planner_) { global_planner_->cleanup(); }
  if (local_planner_) { local_planner_->cleanup(); }
  if (path_tracker_) { path_tracker_->cleanup(); }
}

void NavServer::initialize()
{
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 代价地图分两层（对齐 Nav2 的 global/local 概念）：
  // - 全局：静态 /map（给全局规划器，保证能看到整个地图）
  // - 局部：滚动窗口 + 激光障碍层（给局部规划器做避障）
  static_costmap_ = std::make_shared<Nav2CostmapAdapter>(shared_from_this());
  local_costmap_ = std::make_shared<RollingCostmap>(
    shared_from_this(), tf_buffer_, static_costmap_);

  // 发布者
  global_plan_pub_ = create_publisher<nav_msgs::msg::Path>("global_plan", 10);
  local_plan_pub_ = create_publisher<nav_msgs::msg::Path>("local_plan", 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1);
  obstacle_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_layer", 1);

  // 订阅目标点（兼容入口）
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    std::bind(&NavServer::goalCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 20,
    std::bind(&NavServer::odomCallback, this, std::placeholders::_1));

  // 加载插件
  loadPlugins();

  // Action 服务（主入口）：navigate_to_3d_pose
  action_server_ = rclcpp_action::create_server<NavigateTo3DPose>(
    shared_from_this(),
    "navigate_to_3d_pose",
    std::bind(&NavServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavServer::handleCancel, this, std::placeholders::_1),
    std::bind(&NavServer::handleAccepted, this, std::placeholders::_1));

  // 控制循环定时器
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&NavServer::controlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "NavServer initialized, control freq: %.1f Hz, action: navigate_to_3d_pose, "
    "recovery retries: %d, progress timeout: %.1f s",
    control_frequency_, max_planning_retries_, progress_timeout_);
}

void NavServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_ = msg->twist.twist;
}

void NavServer::loadPlugins()
{
  auto global_plugin_name = get_parameter("global_planner_plugin").as_string();
  auto local_plugin_name = get_parameter("local_planner_plugin").as_string();
  auto tracker_plugin_name = get_parameter("path_tracker_plugin").as_string();

  try {
    global_planner_ = global_planner_loader_.createSharedInstance(global_plugin_name);
    global_planner_->configure(shared_from_this(), "global_planner", static_costmap_);
    global_planner_->activate();
    current_global_plugin_ = global_plugin_name;
    RCLCPP_INFO(get_logger(), "Loaded global planner: %s", global_plugin_name.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load global planner '%s': %s",
      global_plugin_name.c_str(), ex.what());
  }

  if (!local_plugin_name.empty()) {
    try {
      local_planner_ = local_planner_loader_.createSharedInstance(local_plugin_name);
      local_planner_->configure(shared_from_this(), "local_planner", local_costmap_);
      local_planner_->activate();
      RCLCPP_INFO(get_logger(), "Loaded local planner: %s", local_plugin_name.c_str());
      return;
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load local planner '%s': %s",
        local_plugin_name.c_str(), ex.what());
    }
  }

  if (!tracker_plugin_name.empty()) {
    try {
      path_tracker_ = path_tracker_loader_.createSharedInstance(tracker_plugin_name);
      path_tracker_->configure(shared_from_this(), "path_tracker");
      path_tracker_->activate();
      RCLCPP_INFO(get_logger(), "Loaded path tracker: %s", tracker_plugin_name.c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load path tracker '%s': %s",
        tracker_plugin_name.c_str(), ex.what());
    }
  }
}

bool NavServer::switchGlobalPlanner(const std::string & plugin_name)
{
  try {
    if (global_planner_ && current_global_plugin_ == plugin_name) {
      return true;  // 已经是目标插件，无需切换
    }
    if (global_planner_) {
      global_planner_->deactivate();
      global_planner_->cleanup();
      global_planner_.reset();
    }
    global_planner_ = global_planner_loader_.createSharedInstance(plugin_name);
    global_planner_->configure(shared_from_this(), "global_planner", static_costmap_);
    global_planner_->activate();
    current_global_plugin_ = plugin_name;
    RCLCPP_INFO(get_logger(), "Switched global planner to: %s", plugin_name.c_str());
    return true;
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to switch global planner: %s", ex.what());
    return false;
  }
}

bool NavServer::switchLocalPlanner(const std::string & plugin_name)
{
  try {
    if (local_planner_) {
      local_planner_->deactivate();
      local_planner_->cleanup();
    }
    local_planner_ = local_planner_loader_.createSharedInstance(plugin_name);
    local_planner_->configure(shared_from_this(), "local_planner", local_costmap_);
    local_planner_->activate();
    path_tracker_.reset();
    RCLCPP_INFO(get_logger(), "Switched local planner to: %s", plugin_name.c_str());
    return true;
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to switch local planner: %s", ex.what());
    return false;
  }
}

bool NavServer::switchPathTracker(const std::string & plugin_name)
{
  try {
    if (local_planner_) {
      local_planner_->deactivate();
      local_planner_->cleanup();
      local_planner_.reset();
    }
    if (path_tracker_) {
      path_tracker_->deactivate();
      path_tracker_->cleanup();
    }
    path_tracker_ = path_tracker_loader_.createSharedInstance(plugin_name);
    path_tracker_->configure(shared_from_this(), "path_tracker");
    path_tracker_->activate();
    RCLCPP_INFO(get_logger(), "Switched path tracker to: %s", plugin_name.c_str());
    return true;
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to switch path tracker: %s", ex.what());
    return false;
  }
}

// ---------------------------------------------------------------------------
// Action 服务
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse NavServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const NavigateTo3DPose::Goal> goal)
{
  const auto & position = goal->goal.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: non-finite position");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted goal request: (%.2f, %.2f, %.2f) planner_id='%s' max_planning_time=%.2f s",
    position.x, position.y, position.z,
    goal->planner_id.c_str(), goal->max_planning_time);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavServer::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received cancel request");
  cancel_requested_ = true;  // 实际 cancelled() 在控制循环里完成，保证先停车再回 result
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  // 抢占：单目标模型，新目标直接结束旧目标
  if (goal_handle_ && goal_handle_->is_active()) {
    RCLCPP_WARN(get_logger(), "New goal preempts the previous one");
    finishGoal(NavErrorCode::CANCELLED, "preempted by a new goal");
  }

  goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();
  if (!startGoal(goal->goal, goal->planner_id, goal->max_planning_time)) {
    finishGoal(NavErrorCode::INVALID_GOAL, "failed to start goal (invalid pose or planner_id)");
  }
}

bool NavServer::startGoal(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  double max_planning_time)
{
  const auto & position = goal.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
    return false;
  }

  // 插件热插拔：goal.planner_id 非空时切换到指定全局规划器
  if (!planner_id.empty() && !switchGlobalPlanner(planner_id)) {
    return false;
  }

  current_goal_ = goal;
  if (current_goal_.header.frame_id.empty()) {
    current_goal_.header.frame_id = global_frame_;
  }

  goal_planner_id_ = planner_id;
  goal_max_planning_time_ = max_planning_time > 0.0 ? max_planning_time : 0.0;
  recovery_count_ = 0;
  distance_traveled_ = 0.0;
  cancel_requested_ = false;
  goal_active_ = true;
  tf_fail_count_ = 0;
  has_last_progress_ = false;
  has_last_position_ = false;
  goal_start_time_ = std::chrono::steady_clock::now();
  plan_start_time_ = goal_start_time_;

  RCLCPP_INFO(
    get_logger(), "Start goal: (%.2f, %.2f) frame=%s planner=%s max_planning_time=%.2f s",
    position.x, position.y, current_goal_.header.frame_id.c_str(),
    planner_id.empty() ? "<default>" : planner_id.c_str(), goal_max_planning_time_);

  transitionTo(NavState::PLANNING);
  return true;
}

void NavServer::finishGoal(NavErrorCode code, const std::string & message)
{
  const bool success = (code == NavErrorCode::OK);
  stopRobot();

  const double total_time = goal_active_ ? secondsSince(goal_start_time_) : 0.0;

  if (goal_handle_ && goal_handle_->is_active()) {
    auto result = std::make_shared<NavigateTo3DPose::Result>();
    result->success = success;
    result->error_code = static_cast<uint16_t>(code);
    result->total_time = total_time;
    result->distance_traveled = distance_traveled_;
    result->message = message;

    if (goal_handle_->is_canceling()) {
      goal_handle_->canceled(result);
    } else if (success) {
      goal_handle_->succeed(result);
    } else {
      goal_handle_->abort(result);
    }
    RCLCPP_INFO(
      get_logger(), "Goal finished: success=%s error_code=%u(%s) time=%.1fs distance=%.2fm msg=%s",
      success ? "true" : "false", static_cast<unsigned>(code), toString(code),
      result->total_time, result->distance_traveled, message.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "Navigation %s: %s (%s)",
      success ? "succeeded" : "failed", toString(code), message.c_str());
  }

  goal_handle_.reset();
  goal_active_ = false;
  cancel_requested_ = false;
  has_last_progress_ = false;
  has_last_position_ = false;
  transitionTo(success ? NavState::SUCCEEDED : NavState::FAILED);
}

void NavServer::publishFeedback(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!goal_handle_ || !goal_handle_->is_active()) {
    return;
  }
  auto feedback = std::make_shared<NavigateTo3DPose::Feedback>();
  feedback->current_pose = pose;
  feedback->distance_remaining = distance2D(pose, current_goal_);
  feedback->eta_seconds = feedback_eta_speed_ > 1e-3
    ? feedback->distance_remaining / feedback_eta_speed_
    : 0.0;
  feedback->number_of_recoveries = recovery_count_;
  feedback->current_state = toString(state_);
  goal_handle_->publish_feedback(feedback);
}

bool NavServer::enterRecovery(const std::string & reason)
{
  ++recovery_count_;

  // 恢复次数用尽：结束会话（规划失败 / 反复卡住都走这里，保证有出口）
  if (max_planning_retries_ >= 0 &&
    recovery_count_ > static_cast<uint16_t>(max_planning_retries_))
  {
    finishGoal(
      NavErrorCode::MAX_RETRIES,
      "recovery attempts exhausted after " + std::to_string(recovery_count_) +
      " tries (last: " + reason + ")");
    return false;
  }

  RCLCPP_WARN(
    get_logger(), "Entering recovery (%s), attempt %u/%d",
    reason.c_str(), recovery_count_, max_planning_retries_);
  if (local_costmap_) {
    local_costmap_->clear();  // 清掉可能残留的激光障碍点，避免"顶着幻影障碍"反复卡住
  }
  recovery_start_time_ = std::chrono::steady_clock::now();
  has_last_progress_ = false;
  transitionTo(NavState::RECOVERING);
  return true;
}

void NavServer::stopRobot()
{
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }
}

// ---------------------------------------------------------------------------
// 兼容入口：/goal_pose
// ---------------------------------------------------------------------------

void NavServer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received /goal_pose (compat): (%.2f, %.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  if (goal_handle_ && goal_handle_->is_active()) {
    finishGoal(NavErrorCode::CANCELLED, "preempted by /goal_pose");
  }
  goal_handle_.reset();

  if (!startGoal(*msg, "", 0.0)) {
    RCLCPP_ERROR(get_logger(), "Failed to start goal from /goal_pose");
  }
}

// ---------------------------------------------------------------------------
// 位姿 / 代价地图
// ---------------------------------------------------------------------------

bool NavServer::getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    tf_fail_count_ = 0;
    return true;
  } catch (const tf2::TransformException & ex) {
    if (tf_fail_count_ < UINT16_MAX) {
      ++tf_fail_count_;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Could not get robot pose: %s", ex.what());
  }
  return false;
}

void NavServer::publishCostmap()
{
  if (!local_costmap_ || !costmap_pub_ || !local_costmap_->enabled()) {
    return;
  }

  // 合并窗口（静态障碍 + 激光障碍 + 膨胀）
  auto merged = local_costmap_->toOccupancyGrid(false);
  merged.header.stamp = now();
  costmap_pub_->publish(merged);

  // 只有激光障碍的窗口（调试/教学：确认障碍层真的在工作）
  if (obstacle_pub_) {
    auto obstacles = local_costmap_->toOccupancyGrid(true);
    obstacles.header.stamp = now();
    obstacle_pub_->publish(obstacles);
  }
}

void NavServer::updateLocalCostmap()
{
  if (!local_costmap_ || !local_costmap_->enabled()) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (getCurrentPose(pose)) {
    local_costmap_->update(pose);
  }
}

void NavServer::transitionTo(NavState new_state)
{
  RCLCPP_INFO(get_logger(), "State transition: %s -> %s", toString(state_), toString(new_state));
  state_ = new_state;
}

// ---------------------------------------------------------------------------
// 控制循环
// ---------------------------------------------------------------------------

void NavServer::controlLoop()
{
  // 局部代价地图（滚动窗口 + 障碍层）：每周期更新，代价是"有新扫描/机器人移动超过阈值"才重建
  updateLocalCostmap();

  // 代价地图可视化：每 10 帧（约 0.5s）发布一次（任意状态）
  if (costmap_frame_count_++ % 10 == 0) {
    publishCostmap();
  }

  // 取消请求：任意状态优先处理，先停车再回 result
  if (cancel_requested_) {
    RCLCPP_INFO(get_logger(), "Cancel accepted, stopping robot");
    finishGoal(NavErrorCode::CANCELLED, "goal cancelled by client");
    return;
  }

  // TF 长时间不可用 → 直接判失败（否则会一直卡在 PLANNING）
  if (goal_active_ && tf_timeout_ticks_ > 0 &&
    tf_fail_count_ > static_cast<uint16_t>(tf_timeout_ticks_))
  {
    finishGoal(NavErrorCode::TF_FAILURE, "TF unavailable, check map->odom->base_link");
    return;
  }

  switch (state_) {
    case NavState::IDLE:
      break;

    case NavState::PLANNING:
    {
      if (!global_planner_) {
        finishGoal(NavErrorCode::NO_PLANNER, "no global planner loaded");
        break;
      }

      geometry_msgs::msg::PoseStamped current_pose;
      if (!getCurrentPose(current_pose)) {
        break;
      }

      // 单次规划超时（action.goal.max_planning_time，0 = 不限）
      if (goal_max_planning_time_ > 0.0 && secondsSince(plan_start_time_) > goal_max_planning_time_) {
        finishGoal(
          NavErrorCode::PLANNING_TIMEOUT,
          "planning exceeded " + std::to_string(goal_max_planning_time_) + " s");
        break;
      }

      auto path = global_planner_->createPlan(current_pose, current_goal_);

      if (path.poses.empty()) {
        RCLCPP_WARN(
          get_logger(), "Global planner failed to find a path (recovery %u/%d)",
          recovery_count_, max_planning_retries_);
        enterRecovery("planning failed");
        break;
      }

      current_global_path_ = path;
      global_plan_pub_->publish(path);

      if (local_planner_) {
        if (!local_planner_->setPath(path)) {
          finishGoal(NavErrorCode::NO_PATH, "local planner rejected the path");
          break;
        }
        RCLCPP_INFO(get_logger(), "Local planner set with new path (%zu poses)", path.poses.size());
      } else if (path_tracker_) {
        if (!path_tracker_->setPath(path)) {
          finishGoal(NavErrorCode::NO_PATH, "path tracker rejected the path");
          break;
        }
        RCLCPP_INFO(get_logger(), "Path tracker set with new path (%zu poses)", path.poses.size());
      } else {
        finishGoal(NavErrorCode::NO_PLANNER, "no local planner or path tracker loaded");
        break;
      }

      // 重置卡住检测
      last_progress_pose_ = current_pose.pose.position;
      last_progress_time_ = std::chrono::steady_clock::now();
      has_last_progress_ = true;

      publishFeedback(current_pose);
      transitionTo(NavState::FOLLOWING);
      RCLCPP_INFO(get_logger(), "Global path found with %zu waypoints", path.poses.size());
      break;
    }

    case NavState::FOLLOWING:
    {
      geometry_msgs::msg::PoseStamped current_pose;
      if (!getCurrentPose(current_pose)) {
        break;
      }

      // 里程累加
      if (has_last_position_) {
        distance_traveled_ += distance2D(last_position_, current_pose.pose.position);
      }
      last_position_ = current_pose.pose.position;
      has_last_position_ = true;

      // 卡住检测：窗口内位移不足 → 恢复
      if (progress_timeout_ > 0.0) {
        if (!has_last_progress_) {
          last_progress_pose_ = current_pose.pose.position;
          last_progress_time_ = std::chrono::steady_clock::now();
          has_last_progress_ = true;
        } else if (distance2D(last_progress_pose_, current_pose.pose.position) > progress_epsilon_) {
          last_progress_pose_ = current_pose.pose.position;
          last_progress_time_ = std::chrono::steady_clock::now();
        } else if (secondsSince(last_progress_time_) > progress_timeout_) {
          RCLCPP_WARN(
            get_logger(), "No progress for %.1f s (moved < %.2f m), entering recovery",
            progress_timeout_, progress_epsilon_);
          enterRecovery("stuck");
          break;
        }
      }

      geometry_msgs::msg::TwistStamped cmd;
      bool goal_reached = false;

      if (local_planner_) {
        cmd = local_planner_->computeVelocityCommand(current_pose, current_velocity_);
        goal_reached = local_planner_->isGoalReached(
          current_pose, current_goal_, goal_tolerance_dist_, goal_tolerance_angle_);
        auto local_plan = local_planner_->getLocalPlan();
        if (!local_plan.poses.empty()) {
          local_plan_pub_->publish(local_plan);
        }
      } else if (path_tracker_) {
        cmd = path_tracker_->computeVelocityCommand(current_pose, current_velocity_);
        goal_reached = path_tracker_->isPathCompleted(current_pose, goal_tolerance_dist_);
      } else {
        finishGoal(NavErrorCode::NO_PLANNER, "no local planner or path tracker loaded");
        break;
      }

      if (goal_reached) {
        finishGoal(NavErrorCode::OK, "goal reached");
      } else {
        cmd_vel_pub_->publish(cmd.twist);
        publishFeedback(current_pose);
      }
      break;
    }

    case NavState::RECOVERING:
    {
      // 最小恢复动作：原地旋转，换个视角/脱离卡死姿态，然后重新规划。
      // 注：动态障碍层（obstacle layer）加入后，这里应先清局部代价地图再旋转。
      if (secondsSince(recovery_start_time_) < recovery_spin_duration_) {
        geometry_msgs::msg::Twist spin;
        spin.angular.z = recovery_angular_vel_;
        cmd_vel_pub_->publish(spin);
        break;
      }

      stopRobot();
      RCLCPP_INFO(get_logger(), "Recovery spin done, replanning");
      plan_start_time_ = std::chrono::steady_clock::now();
      transitionTo(NavState::PLANNING);
      break;
    }

    case NavState::SUCCEEDED:
      RCLCPP_INFO_ONCE(get_logger(), "Navigation succeeded, waiting for new goal");
      transitionTo(NavState::IDLE);
      break;

    case NavState::FAILED:
      RCLCPP_WARN_ONCE(get_logger(), "Navigation failed, waiting for new goal");
      transitionTo(NavState::IDLE);
      break;
  }
}

}  // namespace pnc_nav_core
