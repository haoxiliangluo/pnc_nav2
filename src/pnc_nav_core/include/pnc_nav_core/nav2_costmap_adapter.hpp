#ifndef PNC_NAV_CORE__NAV2_COSTMAP_ADAPTER_HPP_
#define PNC_NAV_CORE__NAV2_COSTMAP_ADAPTER_HPP_

#include "pnc_nav_core/costmap_interface.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pnc_nav_core
{

class Nav2CostmapAdapter : public CostmapInterface
{
public:
  Nav2CostmapAdapter(const rclcpp::Node::SharedPtr & node);

  uint8_t getCost(double x, double y, double z) const override;
  bool isOccupied(double x, double y, double z) const override;
  bool isInBounds(double x, double y, double z) const override;
  double getResolution() const override;
  void getBounds(double &min_x, double &min_y, double &min_z, double &max_x, double &max_y, double &max_z) const override;
  std::string getFrameId() const override;
  void clear() override {}

  /// 底层 /map 栅格（可能为空）；滚动代价地图用它叠加静态障碍
  const nav_msgs::msg::OccupancyGrid::SharedPtr & mapGrid() const {return current_map_;}
  /// 判定障碍的占用阈值（0-100）
  int occupiedThreshold() const {return occupied_threshold_;}

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool worldToMap(double wx, double wy, int & mx, int & my) const;
  bool isCellInBounds(int mx, int my) const;
  bool isObstacleCell(int mx, int my) const;
  uint8_t rawCostAtCell(int mx, int my) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

  int occupied_threshold_ = 65;
  double robot_radius_ = 0.25;
  double inflation_radius_ = 0.35;
  double cost_scaling_factor_ = 3.0;
  bool unknown_as_occupied_ = true;
};

}
#endif
