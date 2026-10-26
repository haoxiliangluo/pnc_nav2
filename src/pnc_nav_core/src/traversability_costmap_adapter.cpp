// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_core/traversability_costmap_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_nav_core
{

TraversabilityCostmapAdapter::TraversabilityCostmapAdapter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & topic,
  double lethal_score)
: node_(node), lethal_score_(std::clamp(lethal_score, 0.0, 1.0))
{
  sub_ = node_->create_subscription<pnc_nav_interfaces::msg::TraversabilityMap>(
    topic, rclcpp::QoS(1),
    std::bind(&TraversabilityCostmapAdapter::msgCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "TraversabilityCostmapAdapter: subscribed to /%s, lethal_score=%.2f",
    topic.c_str(), lethal_score_);
}

void TraversabilityCostmapAdapter::msgCallback(
  const pnc_nav_interfaces::msg::TraversabilityMap::SharedPtr msg)
{
  if (msg->width == 0 || msg->height == 0 || msg->resolution <= 0.0) {
    return;
  }
  grid_ = msg;
}

bool TraversabilityCostmapAdapter::worldToIndex(double x, double y, size_t & index) const
{
  if (!grid_) {
    return false;
  }
  const int mx = static_cast<int>(std::floor((x - grid_->origin_x) / grid_->resolution));
  const int my = static_cast<int>(std::floor((y - grid_->origin_y) / grid_->resolution));
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(grid_->width) || my >= static_cast<int>(grid_->height))
  {
    return false;
  }
  index = static_cast<size_t>(my) * grid_->width + mx;
  return index < grid_->traversability.size();
}

uint8_t TraversabilityCostmapAdapter::getCost(double x, double y, double z) const
{
  (void)z;
  size_t index = 0;
  if (!worldToIndex(x, y, index)) {
    return cost_values::UNKNOWN;
  }
  const double score = grid_->traversability[index];
  if (!(score >= 0.0)) {   // 同时过滤 NaN
    return cost_values::UNKNOWN;
  }
  return static_cast<uint8_t>(
    std::clamp((1.0 - score) * static_cast<double>(cost_values::LETHAL),
    0.0, static_cast<double>(cost_values::LETHAL)));
}

bool TraversabilityCostmapAdapter::isOccupied(double x, double y, double z) const
{
  (void)z;
  size_t index = 0;
  if (!worldToIndex(x, y, index)) {
    return false;
  }
  const double score = grid_->traversability[index];
  return score >= 0.0 && score < lethal_score_;
}

bool TraversabilityCostmapAdapter::isInBounds(double x, double y, double z) const
{
  (void)z;
  size_t index = 0;
  return worldToIndex(x, y, index);
}

double TraversabilityCostmapAdapter::getResolution() const
{
  return grid_ ? grid_->resolution : 0.1;
}

void TraversabilityCostmapAdapter::getBounds(
  double & min_x, double & min_y, double & min_z,
  double & max_x, double & max_y, double & max_z) const
{
  if (!grid_) {
    min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    return;
  }
  min_x = grid_->origin_x;
  min_y = grid_->origin_y;
  min_z = 0.0;
  max_x = min_x + grid_->width * grid_->resolution;
  max_y = min_y + grid_->height * grid_->resolution;
  max_z = 0.0;
}

std::string TraversabilityCostmapAdapter::getFrameId() const
{
  return grid_ ? grid_->header.frame_id : "map";
}

void TraversabilityCostmapAdapter::clear()
{
  grid_.reset();
}

double TraversabilityCostmapAdapter::getTraversability(double x, double y, double z) const
{
  (void)z;
  size_t index = 0;
  if (!worldToIndex(x, y, index)) {
    return -1.0;
  }
  return grid_->traversability[index];
}

double TraversabilityCostmapAdapter::getGroundHeight(double x, double y) const
{
  size_t index = 0;
  if (!worldToIndex(x, y, index) || index >= grid_->elevation.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return grid_->elevation[index];
}

}  // namespace pnc_nav_core
