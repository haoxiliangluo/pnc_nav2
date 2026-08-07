#include "pnc_nav_rviz/pnc_panel.hpp"

#include <cmath>
#include <string>

#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace pnc_nav_rviz
{

PncPanel::PncPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // ---- 移动控制 ----
  auto * move_box = new QGroupBox("移动控制");
  auto * move_grid = new QGridLayout(move_box);
  auto * fwd_btn = new QPushButton("↑ 前进", move_box);
  auto * back_btn = new QPushButton("↓ 后退", move_box);
  auto * left_btn = new QPushButton("← 左转", move_box);
  auto * right_btn = new QPushButton("→ 右转", move_box);
  auto * stop_btn = new QPushButton("■ 停止", move_box);
  stop_btn->setStyleSheet("QPushButton { background: #c33; color: white; }");
  move_grid->addWidget(fwd_btn, 0, 1);
  move_grid->addWidget(left_btn, 1, 0);
  move_grid->addWidget(stop_btn, 1, 1);
  move_grid->addWidget(right_btn, 1, 2);
  move_grid->addWidget(back_btn, 2, 1);
  connect(fwd_btn, &QPushButton::clicked, this, &PncPanel::onMoveForward);
  connect(back_btn, &QPushButton::clicked, this, &PncPanel::onMoveBackward);
  connect(left_btn, &QPushButton::clicked, this, &PncPanel::onTurnLeft);
  connect(right_btn, &QPushButton::clicked, this, &PncPanel::onTurnRight);
  connect(stop_btn, &QPushButton::clicked, this, &PncPanel::onStop);

  // ---- 建图 ----
  auto * map_box = new QGroupBox("建图");
  auto * map_layout = new QVBoxLayout(map_box);
  auto * save_btn = new QPushButton("💾 保存地图 (write_state)", map_box);
  map_layout->addWidget(save_btn);
  connect(save_btn, &QPushButton::clicked, this, &PncPanel::onSaveMap);

  // ---- 重定位 ----
  auto * reloc_box = new QGroupBox("重定位 (AMCL)");
  auto * reloc_layout = new QGridLayout(reloc_box);
  x_edit_ = new QLineEdit("0.0", reloc_box);
  y_edit_ = new QLineEdit("0.0", reloc_box);
  yaw_edit_ = new QLineEdit("0.0", reloc_box);
  auto * reloc_btn = new QPushButton("初始化位姿", reloc_box);
  reloc_layout->addWidget(new QLabel("x:", reloc_box), 0, 0);
  reloc_layout->addWidget(x_edit_, 0, 1);
  reloc_layout->addWidget(new QLabel("y:", reloc_box), 1, 0);
  reloc_layout->addWidget(y_edit_, 1, 1);
  reloc_layout->addWidget(new QLabel("yaw:", reloc_box), 2, 0);
  reloc_layout->addWidget(yaw_edit_, 2, 1);
  reloc_layout->addWidget(reloc_btn, 3, 0, 1, 2);
  connect(reloc_btn, &QPushButton::clicked, this, &PncPanel::onRelocalize);

  // ---- 待扩展 ----
  auto * ext_box = new QGroupBox("待扩展");
  auto * ext_layout = new QHBoxLayout(ext_box);
  auto * ext1 = new QPushButton("扩展1", ext_box);
  auto * ext2 = new QPushButton("扩展2", ext_box);
  auto * ext3 = new QPushButton("扩展3", ext_box);
  ext_layout->addWidget(ext1);
  ext_layout->addWidget(ext2);
  ext_layout->addWidget(ext3);
  connect(ext1, &QPushButton::clicked, this, &PncPanel::onExtButton1);
  connect(ext2, &QPushButton::clicked, this, &PncPanel::onExtButton2);
  connect(ext3, &QPushButton::clicked, this, &PncPanel::onExtButton3);

  auto * root = new QVBoxLayout(this);
  root->addWidget(move_box);
  root->addWidget(map_box);
  root->addWidget(reloc_box);
  root->addWidget(ext_box);
}

PncPanel::~PncPanel()
{
  if (node_) {
    rclcpp::shutdown();
  }
}

void PncPanel::onInitialize()
{
  fprintf(stderr, "PNC_PANEL: onInitialize called\n");
  node_ = std::make_shared<rclcpp::Node>("pnc_panel");
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  initialpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
  write_state_client_ = node_->create_client<cartographer_ros_msgs::srv::WriteState>(
    "write_state");
}

void PncPanel::publishCmd(double vx, double wz)
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = vx;
  t.angular.z = wz;
  cmd_vel_pub_->publish(t);
}

void PncPanel::onMoveForward() { publishCmd(0.2, 0.0); }
void PncPanel::onMoveBackward() { publishCmd(-0.2, 0.0); }
void PncPanel::onTurnLeft() { publishCmd(0.0, 0.4); }
void PncPanel::onTurnRight() { publishCmd(0.0, -0.4); }
void PncPanel::onStop() { publishCmd(0.0, 0.0); }

void PncPanel::onSaveMap()
{
  if (!write_state_client_ || !write_state_client_->wait_for_service(std::chrono::seconds(2))) {
    QMessageBox::warning(this, "保存地图", "找不到 /write_state 服务（建图节点未运行？）");
    return;
  }
  auto req = std::make_shared<cartographer_ros_msgs::srv::WriteState::Request>();
  req->filename = "/home/hao/pnc_nav2/maps/sc40/sc40.pbstream";
  auto future = write_state_client_->async_send_request(req);
  // 简单轮询结果（不阻塞 UI 太久）
  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto resp = future.get();
    if (resp->status.code == 0) {
      QMessageBox::information(this, "保存地图", "保存成功: " + QString::fromStdString(resp->status.message));
    } else {
      QMessageBox::warning(this, "保存地图", "保存失败: " + QString::fromStdString(resp->status.message));
    }
  } else {
    QMessageBox::warning(this, "保存地图", "保存超时");
  }
}

void PncPanel::onRelocalize()
{
  if (!initialpose_pub_) {
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.pose.pose.position.x = x_edit_->text().toDouble();
  msg.pose.pose.position.y = y_edit_->text().toDouble();
  double yaw = yaw_edit_->text().toDouble();
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.1;
  initialpose_pub_->publish(msg);
}

void PncPanel::onExtButton1()
{
  QMessageBox::information(this, "扩展1", "预留按键，待扩展功能");
}
void PncPanel::onExtButton2()
{
  QMessageBox::information(this, "扩展2", "预留按键，待扩展功能");
}
void PncPanel::onExtButton3()
{
  QMessageBox::information(this, "扩展3", "预留按键，待扩展功能");
}

}  // namespace pnc_nav_rviz

PLUGINLIB_EXPORT_CLASS(pnc_nav_rviz::PncPanel, rviz_common::Panel)
