#include "pnc_nav_core/nav2_costmap_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_nav_core
{

Nav2CostmapAdapter::Nav2CostmapAdapter(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  occupied_threshold_ = node->declare_parameter<int>(
    "map_adapter.occupied_threshold", occupied_threshold_);
  robot_radius_ = node->declare_parameter<double>(
    "map_adapter.robot_radius", robot_radius_);
  inflation_radius_ = node->declare_parameter<double>(
    "map_adapter.inflation_radius", inflation_radius_);
  cost_scaling_factor_ = node->declare_parameter<double>(
    "map_adapter.cost_scaling_factor", cost_scaling_factor_);
  unknown_as_occupied_ = node->declare_parameter<bool>(
    "map_adapter.unknown_as_occupied", unknown_as_occupied_);

  occupied_threshold_ = std::clamp(occupied_threshold_, 0, 100);
  robot_radius_ = std::max(0.0, robot_radius_);
  inflation_radius_ = std::max(robot_radius_, inflation_radius_);
  cost_scaling_factor_ = std::max(0.01, cost_scaling_factor_);

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(10).transient_local(),
    std::bind(&Nav2CostmapAdapter::mapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "Nav2CostmapAdapter: subscribed to /map, occupied_threshold=%d, robot_radius=%.2f, "
    "inflation_radius=%.2f, cost_scaling_factor=%.2f, unknown_as_occupied=%s",
    occupied_threshold_, robot_radius_, inflation_radius_, cost_scaling_factor_,
    unknown_as_occupied_ ? "true" : "false");
}

void Nav2CostmapAdapter::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  RCLCPP_INFO(node_->get_logger(), "Received map: %dx%d, resolution=%.3f",
    msg->info.width, msg->info.height, msg->info.resolution);
}

uint8_t Nav2CostmapAdapter::getCost(double x, double y, double z) const
{
  (void)z;

  if (!current_map_) return cost_values::UNKNOWN;

  int mx = 0;
  int my = 0;
  if (!worldToMap(x, y, mx, my)) {
    return unknown_as_occupied_ ? cost_values::LETHAL : cost_values::UNKNOWN;
  }

  if (isObstacleCell(mx, my)) {
    return cost_values::LETHAL;
  }

  const uint8_t raw_cost = rawCostAtCell(mx, my);
  if (inflation_radius_ <= 0.0 || current_map_->info.resolution <= 0.0) {
    return raw_cost;
  }

  const double resolution = current_map_->info.resolution;
  const int cell_radius = static_cast<int>(std::ceil(inflation_radius_ / resolution));
  double nearest_obstacle_dist = std::numeric_limits<double>::infinity();

  for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      const double dist = std::hypot(dx, dy) * resolution;
      if (dist > inflation_radius_) {
        continue;
      }

      const int nx = mx + dx;
      const int ny = my + dy;
      if (isObstacleCell(nx, ny)) {
        nearest_obstacle_dist = std::min(nearest_obstacle_dist, dist);
      }
    }
  }

  if (!std::isfinite(nearest_obstacle_dist)) {
    return raw_cost;
  }

  if (nearest_obstacle_dist <= robot_radius_) {
    return cost_values::LETHAL;
  }

  const double decay = std::exp(
    -cost_scaling_factor_ * (nearest_obstacle_dist - robot_radius_));
  const double max_soft_cost = static_cast<double>(cost_values::INSCRIBED - 1);
  const auto inflated_cost = static_cast<uint8_t>(
    std::clamp(max_soft_cost * decay, 1.0, max_soft_cost));

  return std::max(raw_cost, inflated_cost);
}

bool Nav2CostmapAdapter::isOccupied(double x, double y, double z) const
{
  const uint8_t cost = getCost(x, y, z);
  if (cost == cost_values::UNKNOWN) {
    return unknown_as_occupied_;
  }
  return cost >= cost_values::INSCRIBED;
}

bool Nav2CostmapAdapter::isInBounds(double x, double y, double z) const
{
  (void)z;

  int mx = 0;
  int my = 0;
  return worldToMap(x, y, mx, my);
}

double Nav2CostmapAdapter::getResolution() const
{
  return current_map_ ? current_map_->info.resolution : 0.05;
}

void Nav2CostmapAdapter::getBounds(double &min_x, double &min_y, double &min_z, double &max_x, double &max_y, double &max_z) const
{
  if (!current_map_) {
    min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    return;
  }
  min_x = current_map_->info.origin.position.x;
  min_y = current_map_->info.origin.position.y;
  min_z = 0.0;
  max_x = min_x + current_map_->info.width * current_map_->info.resolution;
  max_y = min_y + current_map_->info.height * current_map_->info.resolution;
  max_z = 0.0;
}

std::string Nav2CostmapAdapter::getFrameId() const
{
  return current_map_ ? current_map_->header.frame_id : "map";
}

bool Nav2CostmapAdapter::worldToMap(double wx, double wy, int & mx, int & my) const
{
  if (!current_map_ || current_map_->info.resolution <= 0.0) {
    return false;
  }

  mx = static_cast<int>(
    std::floor((wx - current_map_->info.origin.position.x) / current_map_->info.resolution));
  my = static_cast<int>(
    std::floor((wy - current_map_->info.origin.position.y) / current_map_->info.resolution));

  return isCellInBounds(mx, my);
}

bool Nav2CostmapAdapter::isCellInBounds(int mx, int my) const
{
  if (!current_map_) {
    return false;
  }

  return mx >= 0 && my >= 0 &&
    mx < static_cast<int>(current_map_->info.width) &&
    my < static_cast<int>(current_map_->info.height);
}

bool Nav2CostmapAdapter::isObstacleCell(int mx, int my) const
{
  if (!isCellInBounds(mx, my)) {
    return unknown_as_occupied_;
  }

  const int idx = my * static_cast<int>(current_map_->info.width) + mx;
  const int8_t occ = current_map_->data[idx];

  if (occ < 0) {
    return unknown_as_occupied_;
  }

  return occ >= occupied_threshold_;
}

uint8_t Nav2CostmapAdapter::rawCostAtCell(int mx, int my) const
{
  if (!isCellInBounds(mx, my)) {
    return unknown_as_occupied_ ? cost_values::LETHAL : cost_values::UNKNOWN;
  }

  const int idx = my * static_cast<int>(current_map_->info.width) + mx;
  const int8_t occ = current_map_->data[idx];

  if (occ < 0) {
    return unknown_as_occupied_ ? cost_values::LETHAL : cost_values::UNKNOWN;
  }
  if (occ >= occupied_threshold_) {
    return cost_values::LETHAL;
  }
  if (occ > 0) {
    const int scaled = occ * static_cast<int>(cost_values::INSCRIBED - 1) / 100;
    return static_cast<uint8_t>(
      std::clamp(scaled, 1, static_cast<int>(cost_values::INSCRIBED - 1)));
  }

  return cost_values::FREE_SPACE;
}

}
