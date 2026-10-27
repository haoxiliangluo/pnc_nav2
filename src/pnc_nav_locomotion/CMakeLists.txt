cmake_minimum_required(VERSION 3.8)
project(pnc_nav_locomotion)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pnc_nav_core REQUIRED)
find_package(pnc_nav_interfaces REQUIRED)

set(dependencies
  rclcpp pluginlib geometry_msgs std_srvs pnc_nav_core pnc_nav_interfaces
)

add_library(fake_trot_locomotion SHARED src/fake_trot_locomotion.cpp)
target_include_directories(fake_trot_locomotion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fake_trot_locomotion ${dependencies})

add_executable(locomotion_node src/locomotion_node.cpp)
ament_target_dependencies(locomotion_node ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS fake_trot_locomotion locomotion_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS locomotion_node DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(pnc_nav_core plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(fake_trot_locomotion)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fake_trot test/test_fake_trot.cpp)
  target_link_libraries(test_fake_trot fake_trot_locomotion)
  ament_target_dependencies(test_fake_trot ${dependencies})
endif()

ament_package()
