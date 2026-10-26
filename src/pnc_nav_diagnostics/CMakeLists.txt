cmake_minimum_required(VERSION 3.8)
project(pnc_nav_diagnostics)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pnc_nav_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  diagnostic_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  pnc_nav_interfaces
)

add_executable(health_monitor_node src/health_monitor_node.cpp)
ament_target_dependencies(health_monitor_node ${dependencies})

install(TARGETS health_monitor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
