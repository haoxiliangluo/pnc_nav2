cmake_minimum_required(VERSION 3.8)
project(pnc_nav_sim)

find_package(ament_cmake REQUIRED)

# This is a resource-only package (worlds, URDF/xacro, launch). No targets to
# build — we only install shareable assets for the simulation layer.
install(DIRECTORY
  launch
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
