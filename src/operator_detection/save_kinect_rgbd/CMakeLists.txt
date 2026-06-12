cmake_minimum_required(VERSION 3.8)
project(save_kinect_rgbd)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(save_rgbd_node
  src/rgbd_saver_node.cpp
  src/rgbd_saver_ros.cpp
  src/rgbd_saver.cpp)
target_include_directories(save_rgbd_node PRIVATE include)
ament_target_dependencies(save_rgbd_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(save_rgbd_node ${OpenCV_LIBS})

install(TARGETS save_rgbd_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()