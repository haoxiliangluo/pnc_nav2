// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License
//
// elevation_map_node：/points → 2.5D 高程图，发布
//   /elevation_map      (nav_msgs/OccupancyGrid，代价 0-100，未知 -1)
//   /traversability_map (pnc_nav_interfaces/TraversabilityMap，高程/坡度/粗糙度/可通行性)

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "pnc_nav_interfaces/msg/traversability_map.hpp"
#include "pnc_nav_map/elevation_builder.hpp"

namespace pnc_nav_map
{

class ElevationMapNode : public rclcpp::Node
{
public:
  ElevationMapNode() : Node("elevation_map_node")
  {
    ElevationBuilderParams params;
    params.cloud_topic = declare_parameter<std::string>("elevation.cloud_topic", params.cloud_topic);
    params.frame_id = declare_parameter<std::string>("elevation.frame_id", params.frame_id);
    params.robot_frame = declare_parameter<std::string>("elevation.robot_frame", params.robot_frame);
    params.window_size = declare_parameter<double>("elevation.window_size", params.window_size);
    params.resolution = declare_parameter<double>("elevation.resolution", params.resolution);
    params.stride = declare_parameter<int>("elevation.stride", params.stride);
    params.min_range = declare_parameter<double>("elevation.min_range", params.min_range);
    params.max_range = declare_parameter<double>("elevation.max_range", params.max_range);
    params.min_z = declare_parameter<double>("elevation.min_z", params.min_z);
    params.max_z = declare_parameter<double>("elevation.max_z", params.max_z);
    params.recenter_threshold = declare_parameter<double>(
      "elevation.recenter_threshold", params.recenter_threshold);
    params.fill_radius = declare_parameter<double>("elevation.fill_radius", params.fill_radius);
    params.robot_clear_radius = declare_parameter<double>(
      "elevation.robot_clear_radius", params.robot_clear_radius);
    params.slope_baseline_cells = declare_parameter<int>(
      "elevation.slope_baseline_cells", params.slope_baseline_cells);
    params.fill_max_height_spread = declare_parameter<double>(
      "elevation.fill_max_height_spread", params.fill_max_height_spread);
    params.trav.slope_max_deg = declare_parameter<double>(
      "elevation.slope_max_deg", params.trav.slope_max_deg);
    params.trav.step_max_m = declare_parameter<double>(
      "elevation.step_max_m", params.trav.step_max_m);
    params.trav.roughness_max_m = declare_parameter<double>(
      "elevation.roughness_max_m", params.trav.roughness_max_m);
    const double publish_rate = declare_parameter<double>("elevation.publish_rate", 2.0);

    params_ = params;
    publish_rate_ = publish_rate;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /// 必须在 make_shared 之后调用（builder 需要 shared_from_this()）
  void initialize()
  {
    builder_ = std::make_unique<ElevationBuilder>(shared_from_this(), tf_buffer_, params_);

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("elevation_map", 1);
    trav_pub_ = create_publisher<pnc_nav_interfaces::msg::TraversabilityMap>(
      "traversability_map", 1);

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.5, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ElevationMapNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "elevation_map_node ready (publish %.1f Hz): /elevation_map, /traversability_map",
      publish_rate_);
  }

private:
  void onTimer()
  {
    updateRobotPose();
    grid_pub_->publish(builder_->occupancyGrid());
    trav_pub_->publish(builder_->traversabilityMap());
  }

  void updateRobotPose()
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        builder_->params().frame_id, builder_->params().robot_frame, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      builder_->update(pose);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "elevation_map_node: cannot get robot pose: %s", ex.what());
    }
  }

  ElevationBuilderParams params_;
  double publish_rate_{2.0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<ElevationBuilder> builder_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<pnc_nav_interfaces::msg::TraversabilityMap>::SharedPtr trav_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pnc_nav_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pnc_nav_map::ElevationMapNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
