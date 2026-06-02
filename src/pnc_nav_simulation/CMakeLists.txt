cmake_minimum_required(VERSION 3.8)
project(pnc_nav_simulation)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
