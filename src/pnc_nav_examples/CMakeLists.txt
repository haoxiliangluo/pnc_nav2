cmake_minimum_required(VERSION 3.8)
project(pnc_nav_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pnc_nav_core REQUIRED)

set(dependencies
  rclcpp
  pluginlib
  nav_msgs
  geometry_msgs
  pnc_nav_core
)

# ============================================================
# 教学示例插件
# ============================================================

add_library(straight_line_planner SHARED
  src/straight_line_planner.cpp
)
target_include_directories(straight_line_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(straight_line_planner ${dependencies})

# ============================================================
# Install
# ============================================================

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS straight_line_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 关键：把 plugins.xml 注册到基类所在包（pnc_nav_core）的插件索引里
pluginlib_export_plugin_description_file(pnc_nav_core plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(straight_line_planner)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
