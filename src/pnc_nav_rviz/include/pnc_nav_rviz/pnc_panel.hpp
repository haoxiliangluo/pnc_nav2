#ifndef PNC_NAV_RVIZ__PNC_PANEL_HPP_
#define PNC_NAV_RVIZ__PNC_PANEL_HPP_

#include <memory>
#include <string>

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"

namespace pnc_nav_rviz
{

class PncPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PncPanel(QWidget * parent = nullptr);
  ~PncPanel() override;

  void onInitialize() override;

protected Q_SLOTS:
  void onMoveForward();
  void onMoveBackward();
  void onTurnLeft();
  void onTurnRight();
  void onStop();
  void onSaveMap();
  void onRelocalize();
  void onExtButton1();
  void onExtButton2();
  void onExtButton3();

private:
  void publishCmd(double vx, double wz);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Client<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_client_;

  QLineEdit * x_edit_;
  QLineEdit * y_edit_;
  QLineEdit * yaw_edit_;
};

}  // namespace pnc_nav_rviz

#endif  // PNC_NAV_RVIZ__PNC_PANEL_HPP_
