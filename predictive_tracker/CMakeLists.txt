cmake_minimum_required(VERSION 3.8)
project(predictive_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)
find_package(pcl_conversions REQUIRED)
find_package(predictive_navigation_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(dynamic_tracker_node src/dynamic_tracker_node.cpp)

ament_target_dependencies(dynamic_tracker_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  predictive_navigation_msgs
)

target_include_directories(dynamic_tracker_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(dynamic_tracker_node
  ${PCL_LIBRARIES}
)

install(TARGETS dynamic_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
