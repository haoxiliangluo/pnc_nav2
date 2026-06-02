// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__COSTMAP_INTERFACE_HPP_
#define PNC_NAV_CORE__COSTMAP_INTERFACE_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point.hpp"

namespace pnc_nav_core
{

/// 代价值常量
namespace cost_values
{
constexpr uint8_t FREE_SPACE = 0;
constexpr uint8_t INSCRIBED = 253;
constexpr uint8_t LETHAL = 254;
constexpr uint8_t UNKNOWN = 255;
}  // namespace cost_values

/**
 * @class CostmapInterface
 * @brief 代价地图统一接口
 *
 * 抽象3D代价地图的访问方式，支持：
 * - OctoMap (3D占据栅格)
 * - 2.5D高程图 + 代价
 * - ESDF (欧氏距离场)
 * - 2D栅格 (兼容Nav2)
 *
 * 规划器通过此接口查询环境信息，不直接依赖具体地图实现。
 */
class CostmapInterface
{
public:
  using SharedPtr = std::shared_ptr<CostmapInterface>;

  virtual ~CostmapInterface() = default;

  /**
   * @brief 查询某点的代价值
   * @param x, y, z 世界坐标
   * @return 代价值 [0, 255]
   */
  virtual uint8_t getCost(double x, double y, double z) const = 0;

  /**
   * @brief 查询某点是否被占据
   */
  virtual bool isOccupied(double x, double y, double z) const = 0;

  /**
   * @brief 查询某点是否在地图范围内
   */
  virtual bool isInBounds(double x, double y, double z) const = 0;

  /**
   * @brief 查询某点到最近障碍物的距离
   * @return 距离 (m)，若无ESDF信息则返回 -1
   */
  virtual double getDistanceToObstacle(double x, double y, double z) const
  {
    (void)x; (void)y; (void)z;
    return -1.0;
  }

  /**
   * @brief 查询某点的可通行性
   * @return [0, 1]，1表示完全可通行，0表示不可通行，-1表示未知
   */
  virtual double getTraversability(double x, double y, double z) const
  {
    (void)x; (void)y; (void)z;
    return isOccupied(x, y, z) ? 0.0 : 1.0;
  }

  /**
   * @brief 查询某点的地面高度（2.5D）
   * @param x, y 水平坐标
   * @return 地面高度z，若未知返回 NaN
   */
  virtual double getGroundHeight(double x, double y) const
  {
    (void)x; (void)y;
    return std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief 获取地图分辨率
   */
  virtual double getResolution() const = 0;

  /**
   * @brief 获取地图边界
   */
  virtual void getBounds(
    double & min_x, double & min_y, double & min_z,
    double & max_x, double & max_y, double & max_z) const = 0;

  /**
   * @brief 获取地图坐标系
   */
  virtual std::string getFrameId() const = 0;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__COSTMAP_INTERFACE_HPP_
