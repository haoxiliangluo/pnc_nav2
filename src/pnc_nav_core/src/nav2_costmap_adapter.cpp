// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_core/nav2_costmap_adapter.hpp"

namespace pnc_nav_core
{

Nav2CostmapAdapter::Nav2CostmapAdapter(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: costmap_ros_(costmap_ros)
{
  costmap_ = costmap_ros_->getCostmap();
}

uint8_t Nav2CostmapAdapter::getCost(double x, double y, double z) const
{
  (void)z;  // 2D costmap 忽略 z

  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return cost_values::UNKNOWN;
  }

  return costmap_->getCost(mx, my);
}

bool Nav2CostmapAdapter::isOccupied(double x, double y, double z) const
{
  uint8_t cost = getCost(x, y, z);
  return cost >= cost_values::INSCRIBED;
}

bool Nav2CostmapAdapter::isInBounds(double x, double y, double z) const
{
  (void)z;

  unsigned int mx, my;
  return costmap_->worldToMap(x, y, mx, my);
}

double Nav2CostmapAdapter::getResolution() const
{
  return costmap_->getResolution();
}

void Nav2CostmapAdapter::getBounds(
  double & min_x, double & min_y, double & min_z,
  double & max_x, double & max_y, double & max_z) const
{
  min_x = costmap_->getOriginX();
  min_y = costmap_->getOriginY();
  min_z = 0.0;  // 2D costmap

  max_x = min_x + costmap_->getSizeInMetersX();
  max_y = min_y + costmap_->getSizeInMetersY();
  max_z = 0.0;
}

std::string Nav2CostmapAdapter::getFrameId() const
{
  return costmap_ros_->getGlobalFrameID();
}

}  // namespace pnc_nav_core
