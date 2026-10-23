// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_core/rolling_costmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pnc_nav_core
{

namespace
{
constexpr double kMaxObstaclePoints = 20000.0;   // 障碍点上限（防止长时间运行内存膨胀）
constexpr double kRebuildMoveThreshold = 0.25;    // 机器人移动超过这个距离才重建 (m)
}  // namespace

RollingCostmap::RollingCostmap(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<Nav2CostmapAdapter> static_layer)
: node_(node), tf_buffer_(std::move(tf_buffer)), static_layer_(std::move(static_layer))
{
  enabled_ = node_->declare_parameter<bool>("local_costmap.enable", enabled_);
  window_size_ = node_->declare_parameter<double>("local_costmap.window_size", window_size_);
  resolution_ = node_->declare_parameter<double>("local_costmap.resolution", resolution_);
  obstacle_ttl_ = node_->declare_parameter<double>("local_costmap.obstacle_ttl", obstacle_ttl_);
  max_range_ = node_->declare_parameter<double>("local_costmap.max_range", max_range_);
  inflation_radius_ = node_->declare_parameter<double>(
    "local_costmap.inflation_radius", inflation_radius_);
  robot_radius_ = node_->declare_parameter<double>("local_costmap.robot_radius", robot_radius_);
  cost_scaling_factor_ = node_->declare_parameter<double>(
    "local_costmap.cost_scaling_factor", cost_scaling_factor_);
  include_static_ = node_->declare_parameter<bool>(
    "local_costmap.include_static", include_static_);
  unknown_as_occupied_ = node_->declare_parameter<bool>(
    "local_costmap.unknown_as_occupied", unknown_as_occupied_);
  const std::string scan_topic = node_->declare_parameter<std::string>(
    "local_costmap.scan_topic", "nav_scan");

  // 参数清洗
  window_size_ = std::max(1.0, window_size_);
  resolution_ = std::max(0.01, resolution_);
  obstacle_ttl_ = std::max(0.0, obstacle_ttl_);
  max_range_ = std::max(0.5, max_range_);
  robot_radius_ = std::max(0.0, robot_radius_);
  inflation_radius_ = std::max(robot_radius_, inflation_radius_);
  cost_scaling_factor_ = std::max(0.01, cost_scaling_factor_);

  if (static_layer_) {
    frame_id_ = static_layer_->getFrameId();
  }

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&RollingCostmap::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "RollingCostmap: scan=%s window=%.1fm res=%.3f ttl=%.1fs max_range=%.1fm "
    "inflation=%.2f include_static=%s",
    scan_topic.c_str(), window_size_, resolution_, obstacle_ttl_, max_range_,
    inflation_radius_, include_static_ ? "true" : "false");
}

// ---------------------------------------------------------------------------
// 激光 → 障碍点（map 系）
// ---------------------------------------------------------------------------

void RollingCostmap::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!enabled_ || !tf_buffer_) {
    return;
  }

  // 只用"最新可用"变换（tf2::TimePointZero）：
  // 不要用 lookupTransform(..., stamp, rclcpp::Duration) 这种带超时的版本——
  // 仿真刚启动时 /clock 可能还是 0，超时计时基于 ROS 时间会永远等不到，把单线程执行器卡死。
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      frame_id_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "RollingCostmap: cannot transform %s -> %s: %s",
      msg->header.frame_id.c_str(), frame_id_.c_str(), ex.what());
    return;
  }

  const double tx = tf.transform.translation.x;
  const double ty = tf.transform.translation.y;
  const double yaw = tf2::getYaw(tf.transform.rotation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double range_limit = std::min<double>(msg->range_max, max_range_);
  const auto now = std::chrono::steady_clock::now();
  size_t added = 0;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > range_limit) {
      continue;  // 无效/超量程：不当作障碍
    }
    if (r < 0.1) {
      continue;  // 贴脸噪声（可能是机器人自身），跳过
    }

    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const double lx = r * std::cos(angle);
    const double ly = r * std::sin(angle);

    ObstaclePoint point;
    point.x = tx + cos_yaw * lx - sin_yaw * ly;
    point.y = ty + sin_yaw * lx + cos_yaw * ly;
    point.stamp = now;
    points_.push_back(point);
    ++added;
  }

  // 上限保护：丢最老的
  while (points_.size() > static_cast<size_t>(kMaxObstaclePoints)) {
    points_.pop_front();
  }

  if (added > 0) {
    dirty_ = true;
  }
}

void RollingCostmap::expirePoints()
{
  if (obstacle_ttl_ <= 0.0) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  while (!points_.empty()) {
    const double age = std::chrono::duration<double>(now - points_.front().stamp).count();
    if (age <= obstacle_ttl_) {
      break;
    }
    points_.pop_front();
    dirty_ = true;
  }
}

// ---------------------------------------------------------------------------
// 网格重建（窗口随机器人滚动）
// ---------------------------------------------------------------------------

void RollingCostmap::rebuildGrid(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  const double cx = robot_pose.pose.position.x;
  const double cy = robot_pose.pose.position.y;

  width_ = std::max(1, static_cast<int>(std::ceil(window_size_ / resolution_)));
  height_ = width_;
  origin_x_ = cx - 0.5 * width_ * resolution_;
  origin_y_ = cy - 0.5 * height_ * resolution_;

  const size_t cell_count = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  std::vector<uint8_t> static_mask(cell_count, 0);
  std::vector<uint8_t> laser_mask(cell_count, 0);

  // 1) 静态障碍（直接读 /map 栅格，O(1)/cell）
  if (include_static_) {
    auto map = static_layer_ ? static_layer_->mapGrid() : nullptr;
    if (map && map->info.resolution > 0.0) {
      const int threshold = static_layer_->occupiedThreshold();
      const double map_res = map->info.resolution;
      const double map_ox = map->info.origin.position.x;
      const double map_oy = map->info.origin.position.y;
      const int map_w = static_cast<int>(map->info.width);
      const int map_h = static_cast<int>(map->info.height);

      for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
          const double wx = origin_x_ + (gx + 0.5) * resolution_;
          const double wy = origin_y_ + (gy + 0.5) * resolution_;
          const int mx = static_cast<int>(std::floor((wx - map_ox) / map_res));
          const int my = static_cast<int>(std::floor((wy - map_oy) / map_res));
          if (mx < 0 || my < 0 || mx >= map_w || my >= map_h) {
            continue;
          }
          const int8_t occ = map->data[static_cast<size_t>(my) * map_w + mx];
          if (occ >= 0 && occ >= threshold) {
            static_mask[static_cast<size_t>(gy) * width_ + gx] = 1;
          }
        }
      }
    }
  }

  // 2) 激光障碍点 → 栅格
  const double inflate_margin = inflation_radius_ + resolution_;
  for (const auto & point : points_) {
    int mx = 0;
    int my = 0;
    if (!worldToCellPrecomputed(point.x, point.y, mx, my)) {
      continue;
    }
    const double dx = point.x - (origin_x_ + (mx + 0.5) * resolution_);
    const double dy = point.y - (origin_y_ + (my + 0.5) * resolution_);
    if (std::hypot(dx, dy) > inflate_margin) {
      continue;
    }
    laser_mask[static_cast<size_t>(my) * width_ + mx] = 1;
  }

  // 3) 膨胀（一次预计算，查询时 O(1)）
  obstacle_cost_.assign(cell_count, cost_values::FREE_SPACE);
  merged_cost_.assign(cell_count, cost_values::FREE_SPACE);
  inflateMask(laser_mask, obstacle_cost_);

  std::vector<uint8_t> static_cost(cell_count, cost_values::FREE_SPACE);
  inflateMask(static_mask, static_cost);
  for (size_t i = 0; i < cell_count; ++i) {
    merged_cost_[i] = std::max(obstacle_cost_[i], static_cost[i]);
  }

  dirty_ = false;
  has_center_ = true;
  last_center_x_ = cx;
  last_center_y_ = cy;
}

bool RollingCostmap::worldToCell(double wx, double wy, int & mx, int & my) const
{
  return worldToCellPrecomputed(wx, wy, mx, my);
}

bool RollingCostmap::worldToCellPrecomputed(double wx, double wy, int & mx, int & my) const
{
  if (resolution_ <= 0.0 || width_ <= 0 || height_ <= 0) {
    return false;
  }
  mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return mx >= 0 && my >= 0 && mx < width_ && my < height_;
}

void RollingCostmap::inflateMask(
  const std::vector<uint8_t> & mask, std::vector<uint8_t> & out) const
{
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  const double robot_cells = robot_radius_ / resolution_;
  const double max_soft_cost = static_cast<double>(cost_values::INSCRIBED - 1);

  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      const size_t idx = static_cast<size_t>(gy) * width_ + gx;
      if (mask[idx] == 0) {
        continue;
      }
      // 障碍本体
      out[idx] = cost_values::LETHAL;

      // 邻域膨胀
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const int ny = gy + dy;
        if (ny < 0 || ny >= height_) {
          continue;
        }
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int nx = gx + dx;
          if (nx < 0 || nx >= width_) {
            continue;
          }
          const double dist_cells = std::hypot(dx, dy);
          const double dist = dist_cells * resolution_;
          if (dist > inflation_radius_) {
            continue;
          }

          uint8_t cost = cost_values::LETHAL;
          if (dist_cells > robot_cells) {
            const double decay = std::exp(
              -cost_scaling_factor_ * (dist - robot_radius_));
            cost = static_cast<uint8_t>(
              std::clamp(max_soft_cost * decay, 1.0, max_soft_cost));
          }
          const size_t nidx = static_cast<size_t>(ny) * width_ + nx;
          out[nidx] = std::max(out[nidx], cost);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 每周期更新
// ---------------------------------------------------------------------------

void RollingCostmap::update(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (!enabled_) {
    return;
  }

  expirePoints();

  const double cx = robot_pose.pose.position.x;
  const double cy = robot_pose.pose.position.y;
  const bool moved = !has_center_ ||
    std::hypot(cx - last_center_x_, cy - last_center_y_) > kRebuildMoveThreshold;

  if (!dirty_ && !moved) {
    return;
  }
  rebuildGrid(robot_pose);
}

void RollingCostmap::clear()
{
  points_.clear();
  std::fill(obstacle_cost_.begin(), obstacle_cost_.end(), cost_values::FREE_SPACE);
  std::fill(merged_cost_.begin(), merged_cost_.end(), cost_values::FREE_SPACE);
  dirty_ = true;
  RCLCPP_INFO(node_->get_logger(), "RollingCostmap: obstacle layer cleared");
}

// ---------------------------------------------------------------------------
// CostmapInterface
// ---------------------------------------------------------------------------

uint8_t RollingCostmap::getCost(double x, double y, double z) const
{
  (void)z;

  if (!enabled_) {
    return static_layer_ ? static_layer_->getCost(x, y, z) : cost_values::UNKNOWN;
  }

  int mx = 0;
  int my = 0;
  if (!worldToCell(x, y, mx, my) || merged_cost_.empty()) {
    return unknown_as_occupied_ ? cost_values::LETHAL : cost_values::UNKNOWN;
  }
  return merged_cost_[static_cast<size_t>(my) * width_ + mx];
}

bool RollingCostmap::isOccupied(double x, double y, double z) const
{
  const uint8_t cost = getCost(x, y, z);
  if (cost == cost_values::UNKNOWN) {
    return unknown_as_occupied_;
  }
  return cost >= cost_values::INSCRIBED;
}

bool RollingCostmap::isInBounds(double x, double y, double z) const
{
  (void)z;
  if (!enabled_) {
    return static_layer_ ? static_layer_->isInBounds(x, y, z) : false;
  }
  int mx = 0;
  int my = 0;
  return worldToCell(x, y, mx, my);
}

void RollingCostmap::getBounds(
  double & min_x, double & min_y, double & min_z,
  double & max_x, double & max_y, double & max_z) const
{
  if (!enabled_ && static_layer_) {
    static_layer_->getBounds(min_x, min_y, min_z, max_x, max_y, max_z);
    return;
  }
  min_x = origin_x_;
  min_y = origin_y_;
  min_z = 0.0;
  max_x = origin_x_ + width_ * resolution_;
  max_y = origin_y_ + height_ * resolution_;
  max_z = 0.0;
}

nav_msgs::msg::OccupancyGrid RollingCostmap::toOccupancyGrid(bool obstacles_only) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id_;
  msg.info.resolution = resolution_;
  msg.info.width = static_cast<uint32_t>(std::max(0, width_));
  msg.info.height = static_cast<uint32_t>(std::max(0, height_));
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<size_t>(width_) * height_, -1);

  const auto & src = obstacles_only ? obstacle_cost_ : merged_cost_;
  if (src.empty()) {
    return msg;
  }

  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      const size_t idx = static_cast<size_t>(gy) * width_ + gx;
      const uint8_t cost = src[idx];
      int8_t value = 0;
      if (cost >= cost_values::LETHAL) {
        value = 100;
      } else if (cost >= cost_values::INSCRIBED) {
        value = 99;
      } else {
        value = static_cast<int8_t>(cost * 98 / cost_values::INSCRIBED);
      }
      msg.data[idx] = value;
    }
  }
  return msg;
}

}  // namespace pnc_nav_core
