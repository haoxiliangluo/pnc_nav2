// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "rclcpp/rclcpp.hpp"
#include "pnc_nav_core/nav_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<pnc_nav_core::NavServer>();
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
