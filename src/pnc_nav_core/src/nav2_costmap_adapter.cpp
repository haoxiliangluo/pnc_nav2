#include "pnc_nav_core/nav2_costmap_adapter.hpp"

namespace pnc_nav_core
{

Nav2CostmapAdapter::Nav2CostmapAdapter(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(10).transient_local(),
    std::bind(&Nav2CostmapAdapter::mapCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Nav2CostmapAdapter: subscribed to /map");
}

void Nav2CostmapAdapter::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  RCLCPP_INFO(node_->get_logger(), "Received map: %dx%d, resolution=%.3f",
    msg->info.width, msg->info.height, msg->info.resolution);
}

uint8_t Nav2CostmapAdapter::getCost(double x, double y, double z) const
{
  if (!current_map_) return cost_values::UNKNOWN;

  int mx = (x - current_map_->info.origin.position.x) / current_map_->info.resolution;
  int my = (y - current_map_->info.origin.position.y) / current_map_->info.resolution;

  if (mx < 0 || my < 0 || mx >= (int)current_map_->info.width || my >= (int)current_map_->info.height)
    return cost_values::UNKNOWN;

  int idx = my * current_map_->info.width + mx;
  int8_t occ = current_map_->data[idx];

  if (occ < 0) return cost_values::UNKNOWN;
  if (occ > 65) return cost_values::LETHAL;
  if (occ > 0) return static_cast<uint8_t>(occ * 254 / 100);
  return cost_values::FREE_SPACE;
}

bool Nav2CostmapAdapter::isOccupied(double x, double y, double z) const
{
  return getCost(x, y, z) >= cost_values::INSCRIBED;
}

bool Nav2CostmapAdapter::isInBounds(double x, double y, double z) const
{
  if (!current_map_) return false;

  int mx = (x - current_map_->info.origin.position.x) / current_map_->info.resolution;
  int my = (y - current_map_->info.origin.position.y) / current_map_->info.resolution;

  return mx >= 0 && my >= 0 && mx < (int)current_map_->info.width && my < (int)current_map_->info.height;
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

}
