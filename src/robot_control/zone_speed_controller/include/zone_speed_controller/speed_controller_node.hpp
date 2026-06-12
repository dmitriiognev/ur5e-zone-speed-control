#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ur_msgs/srv/set_speed_slider_fraction.hpp>

#include "zone_speed_controller/speed_fsm.hpp"

namespace zone_speed_controller
{

class SpeedControllerNode : public rclcpp::Node
{
public:
  explicit SpeedControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_zone_received(std_msgs::msg::Int32::SharedPtr msg);
  void on_collaborative_mode_received(std_msgs::msg::Bool::SharedPtr msg);
  void on_skeleton_received(geometry_msgs::msg::PoseArray::SharedPtr msg);
  void on_watchdog_tick();

  void apply_command(const SpeedFsm::Command & command);
  void set_speed(double speed);
  void publish_paused(bool paused);

  double skeleton_timeout_;
  SpeedFsm fsm_;
  std::string speed_slider_service_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr zone_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr skeleton_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collaborative_mode_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr paused_publisher_;
  rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace zone_speed_controller