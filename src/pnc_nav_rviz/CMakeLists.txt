cmake_minimum_required(VERSION 3.8)
project(pnc_nav_rviz)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(pnc_panel SHARED
  src/pnc_panel.cpp
  include/pnc_nav_rviz/pnc_panel.hpp
)
target_include_directories(pnc_panel PUBLIC
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(pnc_panel
  Qt5::Widgets
  ${rviz_common_TARGETS}
)
ament_target_dependencies(pnc_panel
  rclcpp
  geometry_msgs
  std_msgs
  cartographer_ros_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS pnc_panel
  DESTINATION lib
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
