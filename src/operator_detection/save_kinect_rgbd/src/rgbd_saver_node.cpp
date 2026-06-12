#include "save_kinect_rgbd/rgbd_saver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto save_rgbd_node = std::make_shared<save_kinect_rgbd::RgbdSaverNode>();
  rclcpp::spin(save_rgbd_node);
  rclcpp::shutdown();
  return 0;
}