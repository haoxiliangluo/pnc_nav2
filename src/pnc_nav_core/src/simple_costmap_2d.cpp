// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_core/simple_costmap_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_nav_core
{

SimpleCostmap2D::SimpleCostmap2D(const rclcpp::Node::SharedPtr & node)
{
  const std::vector<double> default_obstacles = {
    0.5, 1.95, 3.5, 2.05,
    -1.05, -2.0, -0.95, 2.0,
    1.25, -2.75, 2.75, -1.25,
    -4.0, -3.05, -2.0, -2.95,
    -3.05, 0.5, -2.95, 3.5
  };

  node->declare_parameter("costmap.frame_id", frame_id_);
  node->declare_parameter("costmap.origin_x", origin_x_);
  node->declare_parameter("costmap.origin_y", origin_y_);
  node->declare_parameter("costmap.width", width_);
  node->declare_parameter("costmap.height", height_);
  node->declare_parameter("costmap.resolution", resolution_);
  node->declare_parameter("costmap.robot_radius", lethal_radius_);
  node->declare_parameter("costmap.inflation_radius", inflation_radius_);
  node->declare_parameter("costmap.obstacles", default_obstacles);

  frame_id_ = node->get_parameter("costmap.frame_id").as_string();
  origin_x_ = node->get_parameter("costmap.origin_x").as_double();
  origin_y_ = node->get_parameter("costmap.origin_y").as_double();
  width_ = node->get_parameter("costmap.width").as_double();
  height_ = node->get_parameter("costmap.height").as_double();
  resolution_ = node->get_parameter("costmap.resolution").as_double();
  lethal_radius_ = node->get_parameter("costmap.robot_radius").as_double();
  inflation_radius_ = node->get_parameter("costmap.inflation_radius").as_double();
  loadObstacleBoxes(node->get_parameter("costmap.obstacles").as_double_array());

  RCLCPP_INFO(
    node->get_logger(),
    "SimpleCostmap2D: %.1fm x %.1fm, res=%.2f, obstacles=%zu",
    width_, height_, resolution_, obstacles_.size());
}

uint8_t SimpleCostmap2D::getCost(double x, double y, double z) const
{
  (void)z;

  if (!isInBounds(x, y, z)) {
    return cost_values::UNKNOWN;
  }

  const double distance = distanceToNearestObstacle(x, y);
  if (distance <= lethal_radius_) {
    return cost_values::LETHAL;
  }
  if (distance <= inflation_radius_) {
    const double ratio = 1.0 - (distance - lethal_radius_) /
      std::max(1e-6, inflation_radius_ - lethal_radius_);
    return static_cast<uint8_t>(std::clamp(180.0 * ratio, 1.0, 180.0));
  }

  return cost_values::FREE_SPACE;
}

bool SimpleCostmap2D::isOccupied(double x, double y, double z) const
{
  return getCost(x, y, z) >= cost_values::LETHAL;
}

bool SimpleCostmap2D::isInBounds(double x, double y, double z) const
{
  (void)z;
  return x >= origin_x_ && x <= origin_x_ + width_ &&
         y >= origin_y_ && y <= origin_y_ + height_;
}

double SimpleCostmap2D::getResolution() const
{
  return resolution_;
}

void SimpleCostmap2D::getBounds(
  double & min_x, double & min_y, double & min_z,
  double & max_x, double & max_y, double & max_z) const
{
  min_x = origin_x_;
  min_y = origin_y_;
  min_z = 0.0;
  max_x = origin_x_ + width_;
  max_y = origin_y_ + height_;
  max_z = 0.0;
}

std::string SimpleCostmap2D::getFrameId() const
{
  return frame_id_;
}

void SimpleCostmap2D::loadObstacleBoxes(const std::vector<double> & values)
{
  obstacles_.clear();
  for (size_t i = 0; i + 3 < values.size(); i += 4) {
    ObstacleBox box;
    box.min_x = std::min(values[i], values[i + 2]);
    box.min_y = std::min(values[i + 1], values[i + 3]);
    box.max_x = std::max(values[i], values[i + 2]);
    box.max_y = std::max(values[i + 1], values[i + 3]);
    obstacles_.push_back(box);
  }
}

double SimpleCostmap2D::distanceToBox(double x, double y, const ObstacleBox & box) const
{
  const double dx = std::max({box.min_x - x, 0.0, x - box.max_x});
  const double dy = std::max({box.min_y - y, 0.0, y - box.max_y});
  return std::hypot(dx, dy);
}

double SimpleCostmap2D::distanceToNearestObstacle(double x, double y) const
{
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & obstacle : obstacles_) {
    min_distance = std::min(min_distance, distanceToBox(x, y, obstacle));
  }
  return min_distance;
}

}  // namespace pnc_nav_core
