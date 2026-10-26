// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include <gtest/gtest.h>

#include <cmath>

#include "pnc_nav_map/elevation_map.hpp"

using pnc_nav_map::ElevationMap;
using pnc_nav_map::TraversabilityParams;

namespace
{
void fillPlane(ElevationMap & map, double slope)
{
  for (int mx = 0; mx < map.width(); ++mx) {
    for (int my = 0; my < map.height(); ++my) {
      const double x = map.originX() + (mx + 0.5) * map.resolution();
      const double y = map.originY() + (my + 0.5) * map.resolution();
      map.addPoint(x, y, slope * x);
    }
  }
}
}  // namespace

TEST(ElevationMap, FlatPlaneIsFullyTraversable)
{
  ElevationMap map(0.1, 0.0, 0.0, 20, 20);
  fillPlane(map, 0.0);

  double z = 123.0;
  ASSERT_TRUE(map.getGroundHeight(0.55, 0.55, z));
  EXPECT_NEAR(z, 0.0, 1e-9);

  const TraversabilityParams params;
  const auto t = map.traversabilityAt(0.55, 0.55, params);
  EXPECT_NEAR(t.score, 1.0, 1e-6);
}

TEST(ElevationMap, SlopedPlaneIsLessTraversable)
{
  ElevationMap map(0.1, 0.0, 0.0, 20, 20);
  fillPlane(map, 0.1);   // tan ≈ 0.1 → 约 5.7°

  double z = 0.0;
  ASSERT_TRUE(map.getGroundHeight(0.95, 0.55, z));
  EXPECT_NEAR(z, 0.095, 0.02);

  const TraversabilityParams params;
  const auto t = map.traversabilityAt(0.95, 0.55, params);
  EXPECT_GT(t.score, 0.0);
  EXPECT_LT(t.score, 0.95);   // 平地是 1.0，斜坡必须更低
  EXPECT_NEAR(t.slope_rad / M_PI * 180.0, 5.7, 1.5);   // 坡度角约 5.7°
}

TEST(ElevationMap, EmptyCellHasNoData)
{
  ElevationMap map(0.1, 0.0, 0.0, 10, 10);
  double z = 0.0;
  EXPECT_FALSE(map.getGroundHeight(0.5, 0.5, z));

  const auto t = map.traversabilityAt(0.5, 0.5, TraversabilityParams{});
  EXPECT_LT(t.score, 0.0);
}

TEST(ElevationMap, GridExportKeepsUnknownCells)
{
  ElevationMap map(0.1, 1.0, 2.0, 10, 10);
  map.addPoint(1.05, 2.05, 0.0);

  const auto grid = map.toOccupancyGrid(TraversabilityParams{});
  EXPECT_EQ(grid.info.width, 10u);
  EXPECT_EQ(grid.info.height, 10u);
  EXPECT_NEAR(grid.info.resolution, 0.1, 1e-6);
  EXPECT_NEAR(grid.info.origin.position.x, 1.0, 1e-9);
  EXPECT_EQ(grid.data.size(), 100u);
  EXPECT_EQ(grid.data[0], 0);        // 唯一有数据的格 → 可通行
  EXPECT_EQ(grid.data[50], -1);      // 其它格 → 未知
}

TEST(ElevationMap, FillUnknownUsesLowestNeighbor)
{
  ElevationMap map(0.1, 0.0, 0.0, 10, 10);
  map.addPoint(0.55, 0.55, -0.02);   // 一格地面
  map.addPoint(0.55, 0.55, 0.60);    // 同一格还有墙点（均值会被抬高）

  map.fillUnknown(0.25);
  double z = 0.0;
  ASSERT_TRUE(map.getGroundHeight(0.65, 0.55, z));
  EXPECT_NEAR(z, -0.02, 1e-6);       // 填充取邻域"最低 z"，不会被墙高污染
}
