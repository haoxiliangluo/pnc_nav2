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

TEST(ElevationMap, NoisyRampSlopeIsNotAmplified)
{
  // KI-018 回归：15° 斜坡 + 每格 ±0.02m 噪声，坡度估计不应被放大到 25°+。
  ElevationMap map(0.1, 0.0, 0.0, 40, 20);
  const double slope = std::tan(15.0 * M_PI / 180.0);
  for (int mx = 0; mx < map.width(); ++mx) {
    for (int my = 0; my < map.height(); ++my) {
      const double x = map.originX() + (mx + 0.5) * map.resolution();
      const double y = map.originY() + (my + 0.5) * map.resolution();
      const double noise = ((mx * 7 + my * 13) % 5 - 2) * 0.01;   // 确定性 ±0.02m
      map.addPoint(x, y, slope * x + noise);
    }
  }

  // 与 config/elevation.yaml 的 slope_max_deg=25 对齐：15° 坡的分项得分 = 1-15/25 = 0.4
  TraversabilityParams params;
  params.slope_max_deg = 25.0;
  const auto t = map.traversabilityAt(2.0, 1.0, params);
  EXPECT_NEAR(t.slope_rad * 180.0 / M_PI, 15.0, 4.0);
  EXPECT_GT(t.score, 0.35);
}

TEST(ElevationMap, HoleInRampIsFilledAlongSurface)   // KI-019 回归
{
  // 15° 斜坡上挖一个 0.6m×0.2m 的空洞，平面拟合填充应沿坡面补，而不是补成地面高度
  ElevationMap map(0.1, 0.0, 0.0, 40, 20);
  const double slope = std::tan(15.0 * M_PI / 180.0);
  for (int mx = 0; mx < map.width(); ++mx) {
    for (int my = 0; my < map.height(); ++my) {
      const double x = map.originX() + (mx + 0.5) * map.resolution();
      const double y = map.originY() + (my + 0.5) * map.resolution();
      if (x > 1.6 && x < 2.2 && y > 0.4 && y < 0.6) {
        continue;   // 空洞：不写入任何点
      }
      map.addPoint(x, y, slope * x);
    }
  }

  map.fillUnknown(0.4);

  double z = 0.0;
  ASSERT_TRUE(map.getGroundHeight(1.9, 0.5, z));
  EXPECT_NEAR(z, slope * 1.9, 0.06);   // 应贴着坡面，而不是 0

  TraversabilityParams params;
  params.slope_max_deg = 25.0;
  const auto t = map.traversabilityAt(1.9, 0.5, params);
  EXPECT_NEAR(t.slope_rad * 180.0 / M_PI, 15.0, 5.0);
  EXPECT_GT(t.score, 0.3);
}

TEST(ElevationMap, FillSkipsCellsAcrossBigHeightSpread)   // KI-019：跨台阶/崖边不填
{
  // 左半地面 z=0，右半平台 z=0.8：中间未知格邻域高度跨度 0.8 > 阈值 → 不填，保持未知
  ElevationMap map(0.1, 0.0, 0.0, 20, 10);
  for (int mx = 0; mx < map.width(); ++mx) {
    for (int my = 0; my < map.height(); ++my) {
      const double x = map.originX() + (mx + 0.5) * map.resolution();
      const double y = map.originY() + (my + 0.5) * map.resolution();
      if (x > 0.8 && x < 1.1) {
        continue;   // 中间留空（模拟台阶边缘/遮挡）
      }
      map.addPoint(x, y, x > 1.1 ? 0.8 : 0.0);
    }
  }

  map.fillUnknown(0.4);
  double z = 0.0;
  EXPECT_FALSE(map.getGroundHeight(0.95, 0.5, z));   // 跨崖边的格保持未知
  EXPECT_TRUE(map.getGroundHeight(0.15, 0.5, z));    // 平地内部照常
}
