#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "save_kinect_rgbd/rgbd_saver.hpp"

namespace save_kinect_rgbd
{

class RgbdSaverNode : public rclcpp::Node
{
public:
  explicit RgbdSaverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_rgb_received(sensor_msgs::msg::Image::SharedPtr msg);
  void on_depth_received(sensor_msgs::msg::Image::SharedPtr msg);
  void try_save_pair();

  RgbdSaver saver_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;

  sensor_msgs::msg::Image::SharedPtr last_rgb_msg_;
  sensor_msgs::msg::Image::SharedPtr last_depth_msg_;
};

}  // namespace save_kinect_rgbd