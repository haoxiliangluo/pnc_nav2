cmake_minimum_required(VERSION 3.8)
project(pnc_nav_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pnc_nav_core REQUIRED)
find_package(pnc_nav_interfaces REQUIRED)

set(dependencies
  rclcpp
  pluginlib
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pnc_nav_core
  pnc_nav_interfaces
)

# ============================================================
# Global Planners
# ============================================================

# A* 3D
add_library(astar_3d_planner SHARED
  src/global_planners/astar_3d/astar_3d.cpp
)
target_include_directories(astar_3d_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(astar_3d_planner ${dependencies})

# RRT* 3D
add_library(rrt_star_3d_planner SHARED
  src/global_planners/rrt_star_3d/rrt_star_3d.cpp
)
target_include_directories(rrt_star_3d_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rrt_star_3d_planner ${dependencies})

# ============================================================
# Local Planners
# ============================================================

# DWA 3D
add_library(dwa_3d_planner SHARED
  src/local_planners/dwa_3d/dwa_3d.cpp
)
target_include_directories(dwa_3d_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dwa_3d_planner ${dependencies})

# ============================================================
# Path Trackers
# ============================================================

# Pure Pursuit 3D
add_library(pure_pursuit_3d_tracker SHARED
  src/path_trackers/pure_pursuit_3d/pure_pursuit_3d.cpp
)
target_include_directories(pure_pursuit_3d_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pure_pursuit_3d_tracker ${dependencies})

# Stanley 3D
add_library(stanley_3d_tracker SHARED
  src/path_trackers/stanley_3d/stanley_3d.cpp
)
target_include_directories(stanley_3d_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stanley_3d_tracker ${dependencies})

# ============================================================
# Install
# ============================================================

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  astar_3d_planner
  rrt_star_3d_planner
  dwa_3d_planner
  pure_pursuit_3d_tracker
  stanley_3d_tracker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin description file
pluginlib_export_plugin_description_file(pnc_nav_core plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(
  astar_3d_planner
  rrt_star_3d_planner
  dwa_3d_planner
  pure_pursuit_3d_tracker
  stanley_3d_tracker
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
