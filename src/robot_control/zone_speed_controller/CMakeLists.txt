cmake_minimum_required(VERSION 3.8)
project(zone_speed_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

add_executable(speed_controller_node
  src/speed_controller_node.cpp
  src/speed_controller_ros.cpp
  src/speed_fsm.cpp)
target_include_directories(speed_controller_node PRIVATE include)
ament_target_dependencies(speed_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  ur_msgs)

install(TARGETS speed_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()