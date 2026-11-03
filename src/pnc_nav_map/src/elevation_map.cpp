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

      // 收集邻域已知格（相对坐标 + 高程），用于平面拟合
      double sx = 0.0, sy = 0.0, sz = 0.0;
      double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
      int n = 0;
      double lowest = std::numeric_limits<double>::infinity();
      double highest = -std::numeric_limits<double>::infinity();
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
          if (std::hypot(dx, dy) > rc) {
            continue;
          }
          const Cell & c = cells_[static_cast<size_t>(indexOf(nx, ny))];
          if (!c.hasData()) {
            continue;
          }
          const double px = dx * resolution_;
          const double py = dy * resolution_;
          const double pz = c.z_min;
          lowest = std::min(lowest, pz);
          highest = std::max(highest, pz);
          sx += px; sy += py; sz += pz;
          sxx += px * px; sxy += px * py; syy += py * py;
          sxz += px * pz; syz += py * pz;
          ++n;
        }
      }

      if (n == 0) {
        continue;
      }

      // 邻域高度跨度太大（跨台阶/崖边，或远处被遮挡只看到一部分表面）：不填，
      // 保持未知让上层回退静态图，避免把平台填成地面 → 被判不可通行
      if (std::isfinite(lowest) && std::isfinite(highest) &&
        (highest - lowest) > fill_max_height_spread_)
      {
        continue;
      }

      double filled = lowest;
      if (n >= 3) {
        // 解 3x3 正规方程：z = a*x + b*y + c
        const double m00 = sxx, m01 = sxy, m02 = sx;
        const double m11 = syy, m12 = sy, m22 = static_cast<double>(n);
        const double det =
          m00 * (m11 * m22 - m12 * m12) -
          m01 * (m01 * m22 - m12 * m02) +
          m02 * (m01 * m12 - m11 * m02);
        if (std::abs(det) > 1e-12) {
          const double b0 = sxz, b1 = syz, b2 = sz;
          const double a =
            (b0 * (m11 * m22 - m12 * m12) - m01 * (b1 * m22 - m12 * b2) +
            m02 * (b1 * m12 - m11 * b2)) / det;
          const double b =
            (m00 * (b1 * m22 - m12 * b2) - b0 * (m01 * m22 - m12 * m02) +
            m02 * (m01 * b2 - b1 * m02)) / det;
          const double c =
            (m00 * (m11 * b2 - b1 * m12) - m01 * (m01 * b2 - b1 * m02) +
            b0 * (m01 * m12 - m11 * m02)) / det;

          // 残差
          double rms = 0.0;
          for (int dy = -rc; dy <= rc; ++dy) {
            const int ny = my + dy;
            if (ny < 0 || ny >= height_) {
              continue;
            }
            for (int dx = -rc; dx <= rc; ++dx) {
              const int nx = mx + dx;
              if (nx < 0 || nx >= width_ || std::hypot(dx, dy) > rc) {
                continue;
              }
              const Cell & cell = cells_[static_cast<size_t>(indexOf(nx, ny))];
              if (!cell.hasData()) {
                continue;
              }
              const double pred = a * dx * resolution_ + b * dy * resolution_ + c;
              const double e = cell.z_min - pred;
              rms += e * e;
            }
          }
          rms = std::sqrt(rms / static_cast<double>(n));
          if (rms <= fill_planar_max_rms_) {
            // 平面在中心处的值；限制在邻域 [lowest, highest] 内，避免外推出离谱高度
            filled = std::clamp(c, lowest, highest);
          }
        }
      }

      if (std::isfinite(filled)) {
        out[idx].z_sum = filled;
        out[idx].z_min = filled;
        out[idx].z_max = filled;
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
  //
  // KI-018：16 线激光的高程本身有噪声，直接拿相邻格做中心差分会把坡度放大 2–3 倍
  // （实测 15° 斜坡被算成 27–34°，可通行性直接判 0）。这里先用 3×3 均值平滑一次再求梯度。
  static const int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  auto boxMean = [this](int cx, int cy, double & out) {
      double sum = 0.0;
      int n = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          double z = 0.0;
          if (cellMean(cx + dx, cy + dy, z)) {
            sum += z;
            ++n;
          }
        }
      }
      if (n == 0) {
        return false;
      }
      out = sum / n;
      return true;
    };

  double smoothed_center = center_z;
  boxMean(mx, my, smoothed_center);

  std::vector<double> neighbors(8, nan);
  for (int i = 0; i < 8; ++i) {
    double z = 0.0;
    if (boxMean(mx + kDx[i], my + kDy[i], z)) {
      neighbors[i] = z;
    }
  }

  // 宽基线求坡度：用 ±N 格（默认 ±0.2m）的平滑高程做中心差分。
  // 相邻格差分对 16 线激光的噪声太敏感（15° 坡会被放大到 25–34°，KI-018）。
  double slope_rad = 0.0;
  const int base = std::max(1, slope_baseline_cells_);
  double east = nan;
  double west = nan;
  double north = nan;
  double south = nan;
  if (!boxMean(mx + base, my, east)) {east = nan;}
  if (!boxMean(mx - base, my, west)) {west = nan;}
  if (!boxMean(mx, my + base, north)) {north = nan;}
  if (!boxMean(mx, my - base, south)) {south = nan;}

  const double span = 2.0 * base * resolution_;
  double dzdx = 0.0;
  double dzdy = 0.0;
  bool have_dx = false;
  bool have_dy = false;
  if (std::isfinite(east) && std::isfinite(west) && span > 1e-6) {
    dzdx = (east - west) / span;
    have_dx = true;
  }
  if (std::isfinite(north) && std::isfinite(south) && span > 1e-6) {
    dzdy = (north - south) / span;
    have_dy = true;
  }
  if (have_dx || have_dy) {
    slope_rad = slopeRadFromGradient(dzdx, dzdy);
  }

  return evaluateTraversabilityFromSlope(smoothed_center, neighbors, slope_rad, params);
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
