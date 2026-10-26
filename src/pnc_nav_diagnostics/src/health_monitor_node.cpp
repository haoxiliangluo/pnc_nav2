// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License
//
// health_monitor_node：导航链路健康自检
// 检查项：/map 是否收到、/odom 是否在更新、传感器话题（可选）、map→base_link TF、
//        navigate_to_3d_pose action server 是否存在
// 输出：diagnostic_msgs/DiagnosticArray（标准话题 /diagnostics）+ 状态变化日志

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "pnc_nav_interfaces/action/navigate_to3_d_pose.hpp"

namespace pnc_nav_diagnostics
{

using steady = std::chrono::steady_clock;

class HealthMonitorNode : public rclcpp::Node
{
public:
  using NavigateTo3DPose = pnc_nav_interfaces::action::NavigateTo3DPose;

  HealthMonitorNode() : Node("health_monitor")
  {
    period_ = declare_parameter<double>("health.period", 2.0);
    odom_timeout_ = declare_parameter<double>("health.odom_timeout", 2.0);
    scan_timeout_ = declare_parameter<double>("health.scan_timeout", 3.0);
    map_timeout_ = declare_parameter<double>("health.map_timeout", 10.0);
    tf_timeout_ = declare_parameter<double>("health.tf_timeout", 2.0);
    check_action_ = declare_parameter<bool>("health.check_action_server", true);
    scan_topic_ = declare_parameter<std::string>("health.scan_topic", "");
    cloud_topic_ = declare_parameter<std::string>("health.cloud_topic", "");
    require_map_ = declare_parameter<bool>("health.require_map", true);
    map_frame_ = declare_parameter<std::string>("health.map_frame", "map");
    base_frame_ = declare_parameter<std::string>("health.base_frame", "base_link");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr) { last_map_ = steady::now(); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr) { last_odom_ = steady::now(); });

    if (!scan_topic_.empty()) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr) { last_scan_ = steady::now(); });
    }
    if (!cloud_topic_.empty()) {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr) { last_scan_ = steady::now(); });
    }

    if (check_action_) {
      action_client_ = rclcpp_action::create_client<NavigateTo3DPose>(
        this, "navigate_to_3d_pose");
    }

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&HealthMonitorNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "health_monitor started: period=%.1fs scan='%s' cloud='%s' action_check=%s",
      period_, scan_topic_.c_str(), cloud_topic_.c_str(), check_action_ ? "true" : "false");
  }

private:
  struct Check
  {
    std::string name;
    bool ok{false};
    bool warn{false};
    std::string message;
    bool reported{false};
  };

  static double ageSec(const steady::time_point & t)
  {
    if (t.time_since_epoch().count() == 0) {
      return 1e9;
    }
    return std::chrono::duration<double>(steady::now() - t).count();
  }

  void tick()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.header.frame_id = map_frame_;

    // 1) /map
    {
      Check c{"map", !require_map_ || last_map_.time_since_epoch().count() != 0};
      if (require_map_ && !c.ok) {
        c.message = "waiting for /map";
      } else {
        c.ok = true;
        c.message = "ok";
      }
      pushCheck(array, c);
    }

    // 2) /odom
    {
      const double age = ageSec(last_odom_);
      Check c{"odom", age <= odom_timeout_};
      c.message = c.ok ? "ok" : ("no /odom for " + std::to_string(age).substr(0, 4) + " s");
      pushCheck(array, c);
    }

    // 3) 传感器（可选）
    if (!scan_topic_.empty() || !cloud_topic_.empty()) {
      const double age = ageSec(last_scan_);
      Check c{"sensor", age <= scan_timeout_};
      c.message = c.ok ? "ok" : ("no sensor data for " + std::to_string(age).substr(0, 4) + " s");
      pushCheck(array, c);
    }

    // 4) TF map -> base_link
    {
      bool ok = false;
      std::string msg = "lookup failed";
      try {
        const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        (void)tf;
        ok = true;
        msg = "ok";
      } catch (const std::exception & ex) {
        msg = std::string("TF unavailable: ") + ex.what();
        // 用消息判断是否只是"还没数据"，避免刷屏
        if (msg.size() > 120) {
          msg = msg.substr(0, 120);
        }
      }
      Check c{"tf", ok, false, msg};
      pushCheck(array, c);
    }

    // 5) action server
    if (check_action_ && action_client_) {
      const bool ready = action_client_->action_server_is_ready();
      Check c{"navigate_to_3d_pose", ready, false, ready ? "ok" : "action server not available"};
      pushCheck(array, c);
    }

    diagnostics_pub_->publish(array);
    (void)tf_timeout_;
  }

  void pushCheck(
    diagnostic_msgs::msg::DiagnosticArray & array,
    const Check & check)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pnc_nav/" + check.name;
    status.hardware_id = "pnc_nav2";
    status.level = check.ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
      : diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "message";
    kv.value = check.message;
    status.values.push_back(kv);
    array.status.push_back(status);

    // 状态变化时打日志（避免每周期刷）
    auto it = last_state_.find(check.name);
    const bool previous = (it == last_state_.end()) ? true : it->second;
    if (previous != check.ok) {
      if (check.ok) {
        RCLCPP_INFO(get_logger(), "[health] %s: recovered", check.name.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "[health] %s: %s", check.name.c_str(), check.message.c_str());
      }
      last_state_[check.name] = check.ok;
    }
  }

  double period_{2.0};
  double odom_timeout_{2.0};
  double scan_timeout_{3.0};
  double map_timeout_{10.0};
  double tf_timeout_{2.0};
  bool check_action_{true};
  bool require_map_{true};
  std::string scan_topic_;
  std::string cloud_topic_;
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};

  steady::time_point last_map_{};
  steady::time_point last_odom_{};
  steady::time_point last_scan_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_action::Client<NavigateTo3DPose>::SharedPtr action_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, bool> last_state_;
};

}  // namespace pnc_nav_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_nav_diagnostics::HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
