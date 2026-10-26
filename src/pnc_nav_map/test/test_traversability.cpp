// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "pnc_nav_map/traversability.hpp"

using pnc_nav_map::TraversabilityParams;
using pnc_nav_map::evaluateTraversability;
using pnc_nav_map::roughnessFromSamples;
using pnc_nav_map::traversabilityToCost;

namespace
{
constexpr double kDeg = M_PI / 180.0;
}

TEST(Traversability, FlatGroundIsFullyTraversable)
{
  TraversabilityParams params;
  const std::vector<double> neighbors(8, 0.0);
  const auto t = evaluateTraversability(0.0, neighbors, 0.05, params);
  EXPECT_NEAR(t.slope_rad, 0.0, 1e-9);
  EXPECT_NEAR(t.step_m, 0.0, 1e-9);
  EXPECT_NEAR(t.roughness_m, 0.0, 1e-9);
  EXPECT_NEAR(t.score, 1.0, 1e-9);
}

TEST(Traversability, SlopeFifteenDegreesIsSlopeLimited)
{
  TraversabilityParams params;  // slope_max = 20 deg
  const double dz = std::tan(15.0 * kDeg) * 0.05;   // 相邻格高差
  std::vector<double> n(8, 0.0);
  n[pnc_nav_map::kEast] = dz;
  n[pnc_nav_map::kWest] = -dz;

  const auto t = evaluateTraversability(0.0, n, 0.05, params);
  EXPECT_NEAR(t.slope_rad / kDeg, 15.0, 1.0);
  // 坡度分项 = 1 - 15/20 = 0.25，且是最小项
  EXPECT_NEAR(t.score, 0.25, 0.05);
}

TEST(Traversability, StepBeyondThresholdIsZero)
{
  TraversabilityParams params;
  params.slope_max_deg = 90.0;  // 隔离台阶项
  const std::vector<double> n(8, 0.15);
  const auto t = evaluateTraversability(0.0, n, 0.05, params);
  EXPECT_NEAR(t.step_m, 0.15, 1e-9);
  EXPECT_DOUBLE_EQ(t.score, 0.0);
}

TEST(Traversability, RoughnessIsSampleStdDev)
{
  const std::vector<double> zs{0.0, 0.1, 0.0, 0.1};
  EXPECT_NEAR(roughnessFromSamples(zs), 0.057735, 1e-5);
}

TEST(Traversability, MissingDataReturnsNegativeScore)
{
  TraversabilityParams params;
  const auto t = evaluateTraversability(
    std::numeric_limits<double>::quiet_NaN(), std::vector<double>(8, 0.0), 0.05, params);
  EXPECT_LT(t.score, 0.0);
}

TEST(Traversability, CostMapping)
{
  EXPECT_EQ(traversabilityToCost(1.0), 0);
  EXPECT_EQ(traversabilityToCost(0.0), 254);
  EXPECT_EQ(traversabilityToCost(-1.0), 255);
  EXPECT_NEAR(traversabilityToCost(0.5), 127, 2);
}
