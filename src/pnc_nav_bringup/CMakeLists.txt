cmake_minimum_required(VERSION 3.8)
project(pnc_nav_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
