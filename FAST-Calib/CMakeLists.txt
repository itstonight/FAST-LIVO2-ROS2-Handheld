cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# fast_calib 单场景标定节点
add_executable(fast_calib src/main.cpp)
ament_target_dependencies(fast_calib
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  cv_bridge
  livox_ros_driver2
  rosbag2_cpp
)
target_link_libraries(fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# multi_fast_calib 多场景标定节点
add_executable(multi_fast_calib src/multi_scene.cpp)
ament_target_dependencies(multi_fast_calib
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  livox_ros_driver2
  rosbag2_cpp
)
target_link_libraries(multi_fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  fast_calib
  multi_fast_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
