// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_map/elevation_map.hpp"

#include <algorithm>
#include <cmath>

namespace pnc_nav_map
{

ElevationMap::ElevationMap(double resolution, double origin_x, double origin_y, int width, int height)
: resolution_(std::max(0.01, resolution)), origin_x_(origin_x), origin_y_(origin_y),
  width_(std::max(0, width)), height_(std::max(0, height))
{
  reset();
}

void ElevationMap::reset()
{
  cells_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), Cell{});
}

void ElevationMap::setBounds(double origin_x, double origin_y, int width, int height)
{
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  width_ = std::max(0, width);
  height_ = std::max(0, height);
  reset();
}

bool ElevationMap::addPoint(double x, double y, double z)
{
  int mx = 0;
  int my = 0;
  if (!worldToCell(x, y, mx, my) || !std::isfinite(z)) {
    return false;
  }
  Cell & cell = cells_[static_cast<size_t>(indexOf(mx, my))];
  cell.z_sum += z;
  cell.z_min = std::min(cell.z_min, z);
  cell.z_max = std::max(cell.z_max, z);
  ++cell.count;
  return true;
}

void ElevationMap::fillUnknown(double radius_m)
{
  if (radius_m <= 0.0 || resolution_ <= 0.0 || cells_.empty()) {
    return;
  }
  const int rc = std::max(1, static_cast<int>(std::ceil(radius_m / resolution_)));
  std::vector<Cell> out = cells_;   // 先写副本，避免"边填边用"

  for (int my = 0; my < height_; ++my) {
    for (int mx = 0; mx < width_; ++mx) {
      const size_t idx = static_cast<size_t>(indexOf(mx, my));
      if (cells_[idx].hasData()) {
        continue;
      }
      double lowest = std::numeric_limits<double>::infinity();
      for (int dy = -rc; dy <= rc; ++dy) {
        const int ny = my + dy;
        if (ny < 0 || ny >= height_) {
          continue;
        }
        for (int dx = -rc; dx <= rc; ++dx) {
          const int nx = mx + dx;
          if (nx < 0 || nx >= width_) {
            continue;
          }
          const Cell & c = cells_[static_cast<size_t>(indexOf(nx, ny))];
          if (c.hasData() && std::hypot(dx, dy) <= rc) {
            // 用格内最低点（而不是均值）：地面是最低的表面，墙/箱子的高点不会污染填充
            lowest = std::min(lowest, c.z_min);
          }
        }
      }
      if (std::isfinite(lowest)) {
        out[idx].z_sum = lowest;
        out[idx].z_min = lowest;
        out[idx].z_max = lowest;
        out[idx].count = 1;
      }
    }
  }
  cells_.swap(out);
}

bool ElevationMap::worldToCell(double x, double y, int & mx, int & my) const
{
  if (resolution_ <= 0.0 || width_ <= 0 || height_ <= 0) {
    return false;
  }
  mx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return mx >= 0 && my >= 0 && mx < width_ && my < height_;
}

bool ElevationMap::inBounds(double x, double y) const
{
  int mx = 0;
  int my = 0;
  return worldToCell(x, y, mx, my);
}

bool ElevationMap::hasData(int mx, int my) const
{
  if (mx < 0 || my < 0 || mx >= width_ || my >= height_) {
    return false;
  }
  return cells_[static_cast<size_t>(indexOf(mx, my))].hasData();
}

bool ElevationMap::cellMean(int mx, int my, double & z) const
{
  if (mx < 0 || my < 0 || mx >= width_ || my >= height_) {
    return false;
  }
  const Cell & cell = cells_[static_cast<size_t>(indexOf(mx, my))];
  if (!cell.hasData()) {
    return false;
  }
  z = cell.mean();
  return true;
}

bool ElevationMap::getGroundHeight(double x, double y, double & z) const
{
  int mx = 0;
  int my = 0;
  if (!worldToCell(x, y, mx, my)) {
    return false;
  }
  return cellMean(mx, my, z);
}

Traversability ElevationMap::traversabilityAt(
  double x, double y, const TraversabilityParams & params) const
{
  int mx = 0;
  int my = 0;
  double center_z = 0.0;
  if (!worldToCell(x, y, mx, my) || !cellMean(mx, my, center_z)) {
    Traversability none;
    none.score = -1.0;
    return none;
  }

  // 8 邻域，顺序见 NeighborIndex；缺数据的格填 NaN
  static const int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> neighbors(8, nan);
  for (int i = 0; i < 8; ++i) {
    double z = 0.0;
    if (cellMean(mx + kDx[i], my + kDy[i], z)) {
      neighbors[i] = z;
    }
  }

  return evaluateTraversability(center_z, neighbors, resolution_, params);
}

nav_msgs::msg::OccupancyGrid ElevationMap::toOccupancyGrid(
  const TraversabilityParams & params) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.resolution = static_cast<float>(resolution_);
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<size_t>(width_) * height_, -1);

  for (int my = 0; my < height_; ++my) {
    for (int mx = 0; mx < width_; ++mx) {
      if (!hasData(mx, my)) {
        continue;
      }
      const double x = origin_x_ + (mx + 0.5) * resolution_;
      const double y = origin_y_ + (my + 0.5) * resolution_;
      const auto trav = traversabilityAt(x, y, params);
      if (trav.score < 0.0) {
        continue;
      }
      const uint8_t cost = traversabilityToCost(trav.score, 100);
      msg.data[static_cast<size_t>(indexOf(mx, my))] = static_cast<int8_t>(cost);
    }
  }
  return msg;
}

pnc_nav_interfaces::msg::TraversabilityMap ElevationMap::toTraversabilityMap(
  const TraversabilityParams & params) const
{
  pnc_nav_interfaces::msg::TraversabilityMap msg;
  msg.header.frame_id = "map";
  msg.resolution = resolution_;
  msg.origin_x = origin_x_;
  msg.origin_y = origin_y_;
  msg.width = static_cast<uint32_t>(width_);
  msg.height = static_cast<uint32_t>(height_);

  const size_t n = static_cast<size_t>(width_) * height_;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  msg.elevation.assign(n, nan);
  msg.slope.assign(n, nan);
  msg.roughness.assign(n, nan);
  msg.traversability.assign(n, -1.0f);

  for (int my = 0; my < height_; ++my) {
    for (int mx = 0; mx < width_; ++mx) {
      const size_t idx = static_cast<size_t>(indexOf(mx, my));
      double z = 0.0;
      if (!cellMean(mx, my, z)) {
        continue;
      }
      const double x = origin_x_ + (mx + 0.5) * resolution_;
      const double y = origin_y_ + (my + 0.5) * resolution_;
      const auto trav = traversabilityAt(x, y, params);
      msg.elevation[idx] = static_cast<float>(z);
      msg.slope[idx] = static_cast<float>(trav.slope_rad);
      msg.roughness[idx] = static_cast<float>(trav.roughness_m);
      msg.traversability[idx] = static_cast<float>(trav.score);
    }
  }
  return msg;
}

}  // namespace pnc_nav_map
