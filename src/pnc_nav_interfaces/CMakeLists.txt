cmake_minimum_required(VERSION 3.8)
project(pnc_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TraversabilityMap.msg"
  "msg/RobotState.msg"
  "msg/JointCommands.msg"
  "msg/PlannerStatus.msg"
  "srv/SetMap.srv"
  "srv/SwitchPlanner.srv"
  "action/NavigateTo3DPose.action"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
