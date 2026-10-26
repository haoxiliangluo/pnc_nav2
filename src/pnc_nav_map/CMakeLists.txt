cmake_minimum_required(VERSION 3.8)
project(pnc_nav_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pnc_nav_core REQUIRED)
find_package(pnc_nav_interfaces REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pnc_nav_core
  pnc_nav_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/traversability.cpp
  src/elevation_map.cpp
  src/elevation_builder.cpp
  src/elevation_costmap.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(elevation_map_node src/elevation_map_node.cpp)
target_link_libraries(elevation_map_node ${PROJECT_NAME})
ament_target_dependencies(elevation_map_node ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS elevation_map_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_traversability test/test_traversability.cpp)
  target_link_libraries(test_traversability ${PROJECT_NAME})
  ament_add_gtest(test_elevation_map test/test_elevation_map.cpp)
  target_link_libraries(test_elevation_map ${PROJECT_NAME})
  ament_add_gtest(test_elevation_costmap test/test_elevation_costmap.cpp)
  target_link_libraries(test_elevation_costmap ${PROJECT_NAME})
endif()

ament_package()
