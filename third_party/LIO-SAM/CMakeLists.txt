cmake_minimum_required(VERSION 3.5)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()


# pnc_nav: GTSAM 用 /opt/gtsam_sys（源码编译，含 gtsam_unstable、系统 Eigen）
# 安装方式见 scripts/setup_lio_sam.sh
list(PREPEND CMAKE_PREFIX_PATH "/opt/gtsam_sys")
set(CMAKE_BUILD_RPATH "/opt/gtsam_sys/lib")
link_directories(/opt/gtsam_sys/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenMP REQUIRED)

include_directories(
  include/lio_sam
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)



add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM)
if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX)
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}")
endif()


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# pnc_nav: 链接 gtsam（含 unstable），保证裸 colcon build 也能通过
find_library(GTSAM_UNSTABLE_LIB gtsam_unstable PATHS /opt/gtsam_sys/lib REQUIRED)
find_library(GTSAM_LIB gtsam PATHS /opt/gtsam_sys/lib)
foreach(_t lio_sam_featureExtraction lio_sam_imageProjection lio_sam_imuPreintegration lio_sam_mapOptimization)
  if(TARGET ${_t})
    target_link_libraries(${_t} ${GTSAM_UNSTABLE_LIB} ${GTSAM_LIB})
    # 用 DT_RPATH（--disable-new-dtags）：RUNPATH 不会传给传递依赖（libgtsam -> libmetis-gtsam）
    set_target_properties(${_t} PROPERTIES
      LINK_FLAGS "-Wl,--disable-new-dtags -Wl,-rpath,/opt/gtsam_sys/lib"
      BUILD_RPATH "/opt/gtsam_sys/lib"
      INSTALL_RPATH "/opt/gtsam_sys/lib")
  endif()
endforeach()

install(TARGETS
  lio_sam_featureExtraction
  lio_sam_imageProjection
  lio_sam_imuPreintegration
  lio_sam_mapOptimization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
