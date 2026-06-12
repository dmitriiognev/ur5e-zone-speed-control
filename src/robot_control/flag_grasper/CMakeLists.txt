cmake_minimum_required(VERSION 3.8)
project(flag_grasper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gripper_srv REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(flag_grasper_node
  src/flag_grasper_node.cpp
  src/flag_grasper_ros.cpp
  src/flag_grasper.cpp)
target_include_directories(flag_grasper_node PRIVATE include)
ament_target_dependencies(flag_grasper_node
  rclcpp
  std_srvs
  gripper_srv
  moveit_ros_planning_interface)

install(TARGETS flag_grasper_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()