cmake_minimum_required(VERSION 3.5)
project(safety_feature)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(message_filters REQUIRED)

set(THIS_PACKAGE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  message_filters
)

include_directories(include)

add_executable(safety_node src/safety_node.cpp src/safety_feature.cpp)
ament_target_dependencies(safety_node ${THIS_PACKAGE_DEPENDS})


install(TARGETS
  safety_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()