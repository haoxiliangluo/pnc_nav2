// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_core/nav_server.hpp"

#include <chrono>

#include "pnc_nav_core/simple_costmap_2d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pnc_nav_core
{

NavServer::NavServer(const rclcpp::NodeOptions & options)
: Node("nav_server", options),
  global_planner_loader_("pnc_nav_core", "pnc_nav_core::GlobalPlannerBase"),
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
  declare_parameter("path_tracker_plugin", "pnc_nav_planners::PurePursuit3D");

  // 获取参数
  control_frequency_ = get_parameter("control_frequency").as_double();
  goal_tolerance_dist_ = get_parameter("goal_tolerance_dist").as_double();
  goal_tolerance_angle_ = get_parameter("goal_tolerance_angle").as_double();
  max_planning_retries_ = get_parameter("max_planning_retries").as_int();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_frame").as_string();

  RCLCPP_INFO(get_logger(), "NavServer created");
}

NavServer::~NavServer()
{
  if (global_planner_) { global_planner_->cleanup(); }
  if (path_tracker_) { path_tracker_->cleanup(); }
}

void NavServer::initialize()
{
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Phase 1: 手写轻量 2D costmap, 用于验证 A* + Pure Pursuit 闭环。
  costmap_ = std::make_shared<SimpleCostmap2D>(shared_from_this());

  // 发布者
  global_plan_pub_ = create_publisher<nav_msgs::msg::Path>("global_plan", 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 订阅目标点
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    std::bind(&NavServer::goalCallback, this, std::placeholders::_1));

  // 加载插件
  loadPlugins();

  // 控制循环定时器
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&NavServer::controlLoop, this));

  RCLCPP_INFO(get_logger(), "NavServer initialized, control freq: %.1f Hz", control_frequency_);
}

void NavServer::loadPlugins()
{
  auto global_plugin_name = get_parameter("global_planner_plugin").as_string();
  auto tracker_plugin_name = get_parameter("path_tracker_plugin").as_string();

  try {
    global_planner_ = global_planner_loader_.createSharedInstance(global_plugin_name);
    global_planner_->configure(shared_from_this(), "global_planner", costmap_);
    global_planner_->activate();
    RCLCPP_INFO(get_logger(), "Loaded global planner: %s", global_plugin_name.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load global planner '%s': %s",
      global_plugin_name.c_str(), ex.what());
  }

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

bool NavServer::switchGlobalPlanner(const std::string & plugin_name)
{
  try {
    if (global_planner_) {
      global_planner_->deactivate();
      global_planner_->cleanup();
    }
    global_planner_ = global_planner_loader_.createSharedInstance(plugin_name);
    global_planner_->configure(shared_from_this(), "global_planner", costmap_);
    global_planner_->activate();
    RCLCPP_INFO(get_logger(), "Switched global planner to: %s", plugin_name.c_str());
    return true;
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to switch global planner: %s", ex.what());
    return false;
  }
}

bool NavServer::switchPathTracker(const std::string & plugin_name)
{
  try {
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

void NavServer::transitionTo(NavState new_state)
{
  RCLCPP_INFO(get_logger(), "State transition: %d -> %d",
    static_cast<int>(state_), static_cast<int>(new_state));
  state_ = new_state;
}

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
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Could not get robot pose: %s", ex.what());
  }
  return false;
}

void NavServer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received goal: (%.2f, %.2f, %.2f)",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  current_goal_ = *msg;
  if (current_goal_.header.frame_id.empty()) {
    current_goal_.header.frame_id = global_frame_;
  }
  transitionTo(NavState::PLANNING);
}

void NavServer::controlLoop()
{
  switch (state_) {
    case NavState::IDLE:
      break;

    case NavState::PLANNING:
    {
      if (!global_planner_) {
        RCLCPP_ERROR(get_logger(), "No global planner loaded");
        transitionTo(NavState::FAILED);
        break;
      }

      geometry_msgs::msg::PoseStamped current_pose;
      if (!getCurrentPose(current_pose)) {
        break;
      }

      auto path = global_planner_->createPlan(current_pose, current_goal_);

      if (path.poses.empty()) {
        RCLCPP_WARN(get_logger(), "Global planner failed to find a path");
        transitionTo(NavState::RECOVERING);
      } else {
        current_global_path_ = path;
        global_plan_pub_->publish(path);

        if (path_tracker_) {
          path_tracker_->setPath(path);
        }

        transitionTo(NavState::FOLLOWING);
        RCLCPP_INFO(get_logger(), "Global path found with %zu waypoints", path.poses.size());
      }
      break;
    }

    case NavState::FOLLOWING:
    {
      geometry_msgs::msg::PoseStamped current_pose;
      if (!getCurrentPose(current_pose)) {
        break;
      }

      geometry_msgs::msg::TwistStamped cmd;
      bool goal_reached = false;

      if (path_tracker_) {
        cmd = path_tracker_->computeVelocityCommand(current_pose, current_velocity_);
        goal_reached = path_tracker_->isPathCompleted(current_pose, goal_tolerance_dist_);
      } else {
        RCLCPP_ERROR(get_logger(), "No path tracker loaded");
        transitionTo(NavState::FAILED);
        break;
      }

      if (goal_reached) {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        transitionTo(NavState::SUCCEEDED);
        RCLCPP_INFO(get_logger(), "Goal reached!");
      } else {
        cmd_vel_pub_->publish(cmd.twist);
        current_velocity_ = cmd.twist;
      }
      break;
    }

    case NavState::RECOVERING:
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "In recovery state");
      transitionTo(NavState::FAILED);
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
