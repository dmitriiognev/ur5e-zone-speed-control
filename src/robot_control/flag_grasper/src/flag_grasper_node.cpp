#include "flag_grasper/flag_grasper_node.hpp"

#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto flag_grasper_node = std::make_shared<flag_grasper::FlagGrasperNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(flag_grasper_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  flag_grasper_node->init_moveit();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}