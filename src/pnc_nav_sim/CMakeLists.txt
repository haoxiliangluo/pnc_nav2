cmake_minimum_required(VERSION 3.8)
project(pnc_nav_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/points_time_adder.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY worlds models urdf params launch go2 go2w
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/mono_scan_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
