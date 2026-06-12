#include "zone_speed_controller/speed_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace zone_speed_controller
{

// UR io_and_status_controller rejects speed_slider_fraction values outside this range.
static constexpr double SPEED_MIN = 0.01;
static constexpr double SPEED_MAX = 1.0;

SpeedControllerNode::SpeedControllerNode(const rclcpp::NodeOptions & options)
: Node("speed_controller", options),
  skeleton_timeout_(declare_parameter("skeleton_timeout", 1.0)),
  fsm_(static_cast<int>(declare_parameter("num_zones", 5)), skeleton_timeout_)
{
  const auto watchdog_rate = declare_parameter("watchdog_rate", 10.0);
  speed_slider_service_ = declare_parameter<std::string>(
    "speed_slider_service", "/io_and_status_controller/set_speed_slider");
  const auto zone_topic = declare_parameter<std::string>("zone_topic", "/operator/zone");
  const auto skeleton_topic =
    declare_parameter<std::string>("skeleton_topic", "/pose/operator_skeleton");
  const auto collaborative_mode_topic = declare_parameter<std::string>(
    "collaborative_mode_topic", "/operator/collaborative_mode");
  const auto paused_topic = declare_parameter<std::string>("paused_topic", "/motion/paused");

  zone_subscription_ = create_subscription<std_msgs::msg::Int32>(
    zone_topic, 10,
    std::bind(&SpeedControllerNode::on_zone_received, this, std::placeholders::_1));

  skeleton_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
    skeleton_topic, rclcpp::SensorDataQoS(),
    std::bind(&SpeedControllerNode::on_skeleton_received, this, std::placeholders::_1));

  auto latched_qos = rclcpp::QoS(1).transient_local();
  collaborative_mode_subscription_ = create_subscription<std_msgs::msg::Bool>(
    collaborative_mode_topic, latched_qos,
    std::bind(&SpeedControllerNode::on_collaborative_mode_received, this, std::placeholders::_1));
  paused_publisher_ = create_publisher<std_msgs::msg::Bool>(paused_topic, latched_qos);

  speed_slider_client_ = create_client<ur_msgs::srv::SetSpeedSliderFraction>(
    speed_slider_service_);

  const auto period = std::chrono::duration<double>(1.0 / watchdog_rate);
  watchdog_timer_ = create_wall_timer(
    period, std::bind(&SpeedControllerNode::on_watchdog_tick, this));

  publish_paused(false);

  RCLCPP_INFO(
    get_logger(),
    "SpeedControllerNode started | state=%s | %ld zones | skeleton_timeout=%.1fs",
    fsm_.state_name(), get_parameter("num_zones").as_int(), skeleton_timeout_);
}

void SpeedControllerNode::on_zone_received(std_msgs::msg::Int32::SharedPtr msg)
{
  const int zone = msg->data;
  if (!fsm_.is_valid_zone(zone)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Out-of-range zone %d - ignoring.", zone);
    return;
  }

  const auto state_before = fsm_.state();
  const char * state_before_name = fsm_.state_name();
  const auto command = fsm_.on_zone(zone);
  apply_command(command);

  if (fsm_.state() == SpeedFsm::State::PAUSED && state_before != SpeedFsm::State::PAUSED) {
    RCLCPP_WARN(get_logger(), "%s -> PAUSED: zone 0 - robot stopped.", state_before_name);
  } else if (command.speed.has_value()) {
    RCLCPP_INFO(
      get_logger(), "%s -> RUNNING: zone %d, speed=%.2f.",
      state_before_name, zone, *command.speed);
  }
}

void SpeedControllerNode::on_collaborative_mode_received(std_msgs::msg::Bool::SharedPtr msg)
{
  const bool collaborative = msg->data;
  const char * state_before_name = fsm_.state_name();
  const auto command = fsm_.on_collaborative_mode(collaborative);
  apply_command(command);

  if (!command.paused.has_value()) {
    return;
  }
  if (fsm_.state() == SpeedFsm::State::NONCOLLAB) {
    RCLCPP_WARN(
      get_logger(), "%s -> NONCOLLAB: collaborative mode off - zone processing frozen.",
      state_before_name);
  } else {
    RCLCPP_INFO(get_logger(), "NONCOLLAB -> IDLE: collaborative mode restored.");
  }
}

void SpeedControllerNode::on_skeleton_received(geometry_msgs::msg::PoseArray::SharedPtr)
{
  fsm_.on_skeleton(now().seconds());
}

void SpeedControllerNode::on_watchdog_tick()
{
  const char * state_before_name = fsm_.state_name();
  const auto command = fsm_.on_tick(now().seconds());
  if (!command.paused.has_value()) {
    return;
  }
  apply_command(command);
  RCLCPP_ERROR(
    get_logger(), "%s -> PAUSED: no skeleton for %.1f s - perception lost, robot stopped.",
    state_before_name, skeleton_timeout_);
}

void SpeedControllerNode::apply_command(const SpeedFsm::Command & command)
{
  if (command.speed.has_value()) {
    set_speed(*command.speed);
  }
  if (command.paused.has_value()) {
    publish_paused(*command.paused);
  }
}

void SpeedControllerNode::set_speed(double speed)
{
  if (!speed_slider_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Speed slider service '%s' not available.", speed_slider_service_.c_str());
    return;
  }
  auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
  request->speed_slider_fraction = std::clamp(speed, SPEED_MIN, SPEED_MAX);
  speed_slider_client_->async_send_request(request);
}

void SpeedControllerNode::publish_paused(bool paused)
{
  std_msgs::msg::Bool msg;
  msg.data = paused;
  paused_publisher_->publish(msg);
}

}  // namespace zone_speed_controller