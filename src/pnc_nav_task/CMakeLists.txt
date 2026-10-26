cmake_minimum_required(VERSION 3.8)
project(pnc_nav_task)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pnc_nav_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  std_srvs
  pnc_nav_interfaces
)

add_library(${PROJECT_NAME} SHARED src/mission_runner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(mission_executor_node src/mission_executor_node.cpp)
target_link_libraries(mission_executor_node ${PROJECT_NAME})
ament_target_dependencies(mission_executor_node ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS mission_executor_node DESTINATION lib/${PROJECT_NAME})
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
  ament_add_gtest(test_mission_runner test/test_mission_runner.cpp)
  target_link_libraries(test_mission_runner ${PROJECT_NAME})
endif()

ament_package()
