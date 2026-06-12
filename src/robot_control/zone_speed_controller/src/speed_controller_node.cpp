#include "zone_speed_controller/speed_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto speed_controller_node = std::make_shared<zone_speed_controller::SpeedControllerNode>();
  rclcpp::spin(speed_controller_node);
  rclcpp::shutdown();
  return 0;
}