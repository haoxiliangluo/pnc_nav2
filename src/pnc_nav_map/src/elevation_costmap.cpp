// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_map/elevation_costmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_nav_map
{

ElevationCostmap::ElevationCostmap(
  std::shared_ptr<ElevationMap> map,
  TraversabilityParams params,
  double lethal_score)
: map_(std::move(map)), params_(params), lethal_score_(std::clamp(lethal_score, 0.0, 1.0))
{
}

uint8_t ElevationCostmap::getCost(double x, double y, double z) const
{
  (void)z;
  if (!map_) {
    return pnc_nav_core::cost_values::UNKNOWN;
  }
  const auto trav = map_->traversabilityAt(x, y, params_);
  if (trav.score < 0.0) {
    return pnc_nav_core::cost_values::UNKNOWN;
  }
  return traversabilityToCost(trav.score, pnc_nav_core::cost_values::LETHAL);
}

bool ElevationCostmap::isOccupied(double x, double y, double z) const
{
  (void)z;
  if (!map_) {
    return false;
  }
  const auto trav = map_->traversabilityAt(x, y, params_);
  if (trav.score < 0.0) {
    return false;  // 未知区域不当作障碍（与 RollingCostmap 的 unknown_as_occupied=false 一致）
  }
  return trav.score < lethal_score_;
}

bool ElevationCostmap::isInBounds(double x, double y, double z) const
{
  (void)z;
  return map_ && map_->inBounds(x, y);
}

double ElevationCostmap::getResolution() const
{
  return map_ ? map_->resolution() : 0.1;
}

void ElevationCostmap::getBounds(
  double & min_x, double & min_y, double & min_z,
  double & max_x, double & max_y, double & max_z) const
{
  if (!map_) {
    min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    return;
  }
  const double res = map_->resolution();
  min_x = map_->originX();
  min_y = map_->originY();
  min_z = 0.0;
  max_x = min_x + map_->width() * res;
  max_y = min_y + map_->height() * res;
  max_z = 0.0;
}

std::string ElevationCostmap::getFrameId() const
{
  return "map";
}

void ElevationCostmap::clear()
{
  if (map_) {
    map_->reset();
  }
}

double ElevationCostmap::getTraversability(double x, double y, double z) const
{
  (void)z;
  if (!map_) {
    return -1.0;
  }
  const auto trav = map_->traversabilityAt(x, y, params_);
  return trav.score;
}

double ElevationCostmap::getGroundHeight(double x, double y) const
{
  double z = std::numeric_limits<double>::quiet_NaN();
  if (map_ && !map_->getGroundHeight(x, y, z)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return z;
}

}  // namespace pnc_nav_map
