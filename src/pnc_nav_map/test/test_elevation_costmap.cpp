// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "pnc_nav_core/costmap_interface.hpp"
#include "pnc_nav_map/elevation_costmap.hpp"
#include "pnc_nav_map/elevation_map.hpp"

using pnc_nav_map::ElevationCostmap;
using pnc_nav_map::ElevationMap;
using pnc_nav_map::TraversabilityParams;

namespace
{

std::shared_ptr<ElevationMap> makePlane(double slope)
{
  auto map = std::make_shared<ElevationMap>(0.1, 0.0, 0.0, 20, 20);
  for (int mx = 0; mx < map->width(); ++mx) {
    for (int my = 0; my < map->height(); ++my) {
      const double x = map->originX() + (mx + 0.5) * map->resolution();
      const double y = map->originY() + (my + 0.5) * map->resolution();
      map->addPoint(x, y, slope * x);
    }
  }
  return map;
}

}  // namespace

TEST(ElevationCostmap, FlatGroundIsFree)
{
  ElevationCostmap costmap(makePlane(0.0));
  EXPECT_EQ(costmap.getCost(0.55, 0.55, 0.0), pnc_nav_core::cost_values::FREE_SPACE);
  EXPECT_FALSE(costmap.isOccupied(0.55, 0.55, 0.0));
  EXPECT_NEAR(costmap.getTraversability(0.55, 0.55, 0.0), 1.0, 1e-6);
  EXPECT_NEAR(costmap.getGroundHeight(0.55, 0.55), 0.0, 1e-9);
}

TEST(ElevationCostmap, SteepSlopeIsLethal)
{
  // tan 40° ≈ 0.84 → 超过 slope_max(20°)，score 0
  ElevationCostmap costmap(makePlane(std::tan(40.0 * M_PI / 180.0)));
  EXPECT_GE(costmap.getCost(1.55, 0.55, 0.0), pnc_nav_core::cost_values::INSCRIBED);
  EXPECT_TRUE(costmap.isOccupied(1.55, 0.55, 0.0));
  EXPECT_LT(costmap.getTraversability(1.55, 0.55, 0.0), 0.3);
}

TEST(ElevationCostmap, UnknownAreaIsNotOccupiedButHasUnknownCost)
{
  ElevationCostmap costmap(std::make_shared<ElevationMap>(0.1, 0.0, 0.0, 10, 10));
  EXPECT_EQ(costmap.getCost(0.5, 0.5, 0.0), pnc_nav_core::cost_values::UNKNOWN);
  EXPECT_FALSE(costmap.isOccupied(0.5, 0.5, 0.0));
  EXPECT_LT(costmap.getTraversability(0.5, 0.5, 0.0), 0.0);
  EXPECT_TRUE(std::isnan(costmap.getGroundHeight(0.5, 0.5)));
}

TEST(ElevationCostmap, BoundsFollowElevationMap)
{
  auto map = std::make_shared<ElevationMap>(0.1, 1.0, 2.0, 10, 20);
  ElevationCostmap costmap(map);
  EXPECT_NEAR(costmap.getResolution(), 0.1, 1e-9);
  EXPECT_TRUE(costmap.isInBounds(1.5, 2.5, 0.0));
  EXPECT_FALSE(costmap.isInBounds(0.5, 2.5, 0.0));

  double min_x, min_y, min_z, max_x, max_y, max_z;
  costmap.getBounds(min_x, min_y, min_z, max_x, max_y, max_z);
  EXPECT_NEAR(min_x, 1.0, 1e-9);
  EXPECT_NEAR(min_y, 2.0, 1e-9);
  EXPECT_NEAR(max_x, 2.0, 1e-9);
  EXPECT_NEAR(max_y, 4.0, 1e-9);
}
