// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_map/elevation_builder.hpp"

#include <algorithm>
#include <cmath>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/utils.h"

namespace pnc_nav_map
{

ElevationBuilder::ElevationBuilder(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const ElevationBuilderParams & params)
: node_(node), tf_buffer_(std::move(tf_buffer)), params_(params)
{
  const int cells = std::max(1, static_cast<int>(std::ceil(params_.window_size / params_.resolution)));
  // 初始窗口放在原点；第一次 update() 会按机器人位置重新居中
  map_ = std::make_shared<ElevationMap>(
    params_.resolution, -0.5 * cells * params_.resolution, -0.5 * cells * params_.resolution,
    cells, cells);
  map_->setSlopeBaselineCells(params_.slope_baseline_cells);

  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.cloud_topic, rclcpp::SensorDataQoS(),
    std::bind(&ElevationBuilder::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "ElevationBuilder: cloud=%s window=%.1fm res=%.2fm stride=%d z=[%.2f, %.2f] "
    "range=[%.1f, %.1f] recenter=%.1fm",
    params_.cloud_topic.c_str(), params_.window_size, params_.resolution, params_.stride,
    params_.min_z, params_.max_z, params_.min_range, params_.max_range,
    params_.recenter_threshold);
}

void ElevationBuilder::recenter(double x, double y)
{
  const int cells = std::max(1, static_cast<int>(std::ceil(params_.window_size / params_.resolution)));
  const double origin_x = x - 0.5 * cells * params_.resolution;
  const double origin_y = y - 0.5 * cells * params_.resolution;
  map_->setBounds(origin_x, origin_y, cells, cells);
  has_center_ = true;
  center_x_ = x;
  center_y_ = y;
}

void ElevationBuilder::update(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  const double x = robot_pose.pose.position.x;
  const double y = robot_pose.pose.position.y;
  if (!has_center_ ||
    std::hypot(x - center_x_, y - center_y_) > params_.recenter_threshold)
  {
    recenter(x, y);
  }
  // 补洞：16 线激光的地面点呈同心圆环，环间无点，不填充会导致大量"未知"
  map_->fillUnknown(params_.fill_radius);
}

void ElevationBuilder::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!tf_buffer_) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      params_.frame_id, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ElevationBuilder: cannot transform %s -> %s: %s",
      msg->header.frame_id.c_str(), params_.frame_id.c_str(), ex.what());
    return;
  }

  const double tx = tf.transform.translation.x;
  const double ty = tf.transform.translation.y;
  const double tz = tf.transform.translation.z;
  const double yaw = tf2::getYaw(tf.transform.rotation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  size_t index = 0;
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++index) {
    if (params_.stride > 1 && (index % static_cast<size_t>(params_.stride)) != 0) {
      continue;
    }
    const double px = *it_x;
    const double py = *it_y;
    const double pz = *it_z;
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    const double range = std::hypot(px, py);
    if (range < params_.min_range || range > params_.max_range) {
      continue;
    }

    const double mx = tx + cos_yaw * px - sin_yaw * py;
    const double my = ty + sin_yaw * px + cos_yaw * py;
    const double mz = tz + pz;

    // footprint 清除：机器人自身的点（车体/轮子）不能被当成障碍，
    // 否则测量到的"当前位置"可通行性会变成 0，A* 直接判 "Start is occupied"（KI-017）。
    if (has_center_ &&
      std::hypot(mx - center_x_, my - center_y_) < params_.robot_clear_radius)
    {
      continue;
    }

    if (mz < params_.min_z || mz > params_.max_z) {
      continue;
    }

    if (map_->addPoint(mx, my, mz)) {
      ++point_count_;
    }
  }
}

nav_msgs::msg::OccupancyGrid ElevationBuilder::occupancyGrid() const
{
  auto grid = map_->toOccupancyGrid(params_.trav);
  grid.header.stamp = node_->now();
  return grid;
}

pnc_nav_interfaces::msg::TraversabilityMap ElevationBuilder::traversabilityMap() const
{
  auto msg = map_->toTraversabilityMap(params_.trav);
  msg.header.stamp = node_->now();
  return msg;
}

}  // namespace pnc_nav_map
