cmake_minimum_required(VERSION 3.8)
project(lemniscate_executor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lemniscate_executor_node
  src/lemniscate_executor_node.cpp
  src/lemniscate_executor_ros.cpp
  src/lemniscate.cpp)
target_include_directories(lemniscate_executor_node PRIVATE include)
ament_target_dependencies(lemniscate_executor_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  ur_msgs
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs)
target_link_libraries(lemniscate_executor_node Eigen3::Eigen)

install(TARGETS lemniscate_executor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()