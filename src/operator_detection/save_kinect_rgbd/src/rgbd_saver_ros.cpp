#include "save_kinect_rgbd/rgbd_saver_node.hpp"

#include <functional>
#include <string>

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>

namespace save_kinect_rgbd
{

RgbdSaverNode::RgbdSaverNode(const rclcpp::NodeOptions & options)
: Node("save_kinect_rgbd", options),
  saver_(declare_parameter<std::string>("output_directory", "."))
{
  auto rgb_topic = declare_parameter<std::string>("rgb_topic", "/kinect2/sd/image_color_rect");
  auto depth_topic = declare_parameter<std::string>("depth_topic", "/kinect2/sd/image_depth_rect");

  rgb_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    rgb_topic, rclcpp::SensorDataQoS(),
    std::bind(&RgbdSaverNode::on_rgb_received, this, std::placeholders::_1));

  depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic, rclcpp::SensorDataQoS(),
    std::bind(&RgbdSaverNode::on_depth_received, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Waiting for RGB on %s and depth on %s ...",
    rgb_topic.c_str(), depth_topic.c_str());
  RCLCPP_INFO(get_logger(), "Will save as experiment #%d", saver_.experiment_number());
}

void RgbdSaverNode::on_rgb_received(sensor_msgs::msg::Image::SharedPtr msg)
{
  last_rgb_msg_ = msg;
  try_save_pair();
}

void RgbdSaverNode::on_depth_received(sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
    RCLCPP_ERROR(get_logger(), "Depth encoding is %s, expected 16UC1", msg->encoding.c_str());
    return;
  }
  last_depth_msg_ = msg;
  try_save_pair();
}

void RgbdSaverNode::try_save_pair()
{
  if (!last_rgb_msg_ || !last_depth_msg_) {
    return;
  }

  RCLCPP_INFO(get_logger(), "Got RGB+Depth pair, saving...");

  try {
    auto rgb_image = cv_bridge::toCvCopy(*last_rgb_msg_, sensor_msgs::image_encodings::BGR8);
    auto depth_image = cv_bridge::toCvCopy(
      *last_depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
    saver_.save_pair(rgb_image->image, depth_image->image);
    RCLCPP_INFO(get_logger(), "Saved RGB   -> %s", saver_.rgb_path().c_str());
    RCLCPP_INFO(get_logger(), "Saved depth -> %s", saver_.depth_path().c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error while saving: %s", e.what());
  }

  rclcpp::shutdown();
}

}  // namespace save_kinect_rgbd