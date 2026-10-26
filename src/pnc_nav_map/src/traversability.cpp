// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_map/traversability.hpp"

#include <algorithm>
#include <cmath>

namespace pnc_nav_map
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp01(double v)
{
  return std::isfinite(v) ? std::clamp(v, 0.0, 1.0) : 0.0;
}
}  // namespace

double slopeRadFromGradient(double dz_dx, double dz_dy)
{
  return std::atan(std::hypot(dz_dx, dz_dy));
}

double stepFromNeighbors(double center_z, const std::vector<double> & neighbor_zs)
{
  double max_step = 0.0;
  for (double z : neighbor_zs) {
    if (!std::isfinite(z)) {
      continue;
    }
    max_step = std::max(max_step, std::abs(z - center_z));
  }
  return max_step;
}

double roughnessFromSamples(const std::vector<double> & zs)
{
  double sum = 0.0;
  size_t n = 0;
  for (double z : zs) {
    if (!std::isfinite(z)) {
      continue;
    }
    sum += z;
    ++n;
  }
  if (n < 2) {
    return 0.0;
  }
  const double mean = sum / static_cast<double>(n);
  double var = 0.0;
  for (double z : zs) {
    if (!std::isfinite(z)) {
      continue;
    }
    const double d = z - mean;
    var += d * d;
  }
  return std::sqrt(var / static_cast<double>(n - 1));
}

Traversability evaluateTraversability(
  double center_z,
  const std::vector<double> & neighbors,
  double cell_size,
  const TraversabilityParams & params)
{
  Traversability out;
  if (!std::isfinite(center_z) || neighbors.size() < 8 || cell_size <= 1e-6) {
    out.score = -1.0;
    return out;
  }

  // 坡度：中心差分（只在与中心相对的格都有效时计算）
  double dzdx = 0.0;
  double dzdy = 0.0;
  bool have_dx = false;
  bool have_dy = false;
  if (std::isfinite(neighbors[kEast]) && std::isfinite(neighbors[kWest])) {
    dzdx = (neighbors[kEast] - neighbors[kWest]) / (2.0 * cell_size);
    have_dx = true;
  }
  if (std::isfinite(neighbors[kNorth]) && std::isfinite(neighbors[kSouth])) {
    dzdy = (neighbors[kNorth] - neighbors[kSouth]) / (2.0 * cell_size);
    have_dy = true;
  }
  if (have_dx || have_dy) {
    out.slope_rad = slopeRadFromGradient(dzdx, dzdy);
  }

  out.step_m = stepFromNeighbors(center_z, neighbors);
  out.roughness_m = roughnessFromSamples(neighbors);

  const double slope_max_rad = params.slope_max_deg * kPi / 180.0;
  const double slope_score = (slope_max_rad > 1e-6)
    ? clamp01(1.0 - out.slope_rad / slope_max_rad)
    : 1.0;
  const double step_score = (params.step_max_m > 1e-6)
    ? clamp01(1.0 - out.step_m / params.step_max_m)
    : 1.0;
  const double rough_score = (params.roughness_max_m > 1e-6)
    ? clamp01(1.0 - out.roughness_m / params.roughness_max_m)
    : 1.0;

  out.score = std::min({slope_score, step_score, rough_score});
  return out;
}

uint8_t traversabilityToCost(double score, uint8_t lethal_cost)
{
  if (score < 0.0) {
    return 255;
  }
  // score=1 → 0（完全可通行）；score=0 → lethal_cost（不可通行）
  const double cost = (1.0 - clamp01(score)) * static_cast<double>(lethal_cost);
  return static_cast<uint8_t>(std::clamp(cost, 0.0, static_cast<double>(lethal_cost)));
}

}  // namespace pnc_nav_map
