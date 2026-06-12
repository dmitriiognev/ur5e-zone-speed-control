#include "lemniscate_executor/lemniscate_executor_node.hpp"

#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto lemniscate_executor_node =
    std::make_shared<lemniscate_executor::LemniscateExecutorNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(lemniscate_executor_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  lemniscate_executor_node->run();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}