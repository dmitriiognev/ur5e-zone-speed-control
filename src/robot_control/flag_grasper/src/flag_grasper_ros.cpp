#include "flag_grasper/flag_grasper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "flag_grasper/flag_grasper.hpp"

namespace flag_grasper
{

static constexpr double PLANNING_TIME_S = 5.0;
static constexpr std::chrono::seconds GRIPPER_SERVICE_WAIT_TIMEOUT{3};
static constexpr std::chrono::seconds GRIPPER_RESPONSE_TIMEOUT{10};

FlagGrasperNode::FlagGrasperNode(const rclcpp::NodeOptions & options)
: Node("flag_grasper_node", options)
{
  flag_thickness_mm_ = declare_parameter("flag_thickness_mm", 3.0);
  wrist_rotation_deg_ = declare_parameter("wrist_rotation_deg", 90.0);
  gripper_max_aperture_mm_ = declare_parameter("gripper_max_aperture_mm", 85.0);
  gripper_speed_ = static_cast<int>(declare_parameter("gripper_speed", 128));
  gripper_force_ = static_cast<int>(declare_parameter("gripper_force", 100));
  planning_group_ = declare_parameter<std::string>("planning_group", "ur5e_arm");
  wrist_joint_name_ = declare_parameter<std::string>("wrist_joint_name", "ur5e_wrist_3_joint");
  const auto gripper_service_name =
    declare_parameter<std::string>("gripper_service", "gripper_service");

  gripper_client_ = create_client<gripper_srv::srv::GripperService>(gripper_service_name);

  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  grasp_service_ = create_service<std_srvs::srv::Trigger>(
    "flag_grasp",
    std::bind(
      &FlagGrasperNode::on_grasp_requested, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    service_callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "FlagGrasperNode ready - thickness=%.1f mm, rotation=%.1f deg, gripper_service='%s'",
    flag_thickness_mm_, wrist_rotation_deg_, gripper_service_name.c_str());
}

void FlagGrasperNode::init_moveit()
{
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), planning_group_);
  move_group_interface_->setPlanningTime(PLANNING_TIME_S);
  RCLCPP_INFO(get_logger(), "MoveGroupInterface ready (group='%s')", planning_group_.c_str());
}

void FlagGrasperNode::on_grasp_requested(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!move_group_interface_) {
    response->success = false;
    response->message = "MoveGroupInterface not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Flag grasp started - rotating wrist %.1f deg", wrist_rotation_deg_);

  if (!rotate_wrist()) {
    response->success = false;
    response->message = "Wrist rotation failed";
    return;
  }

  RCLCPP_INFO(get_logger(), "Wrist rotated - closing gripper for %.1f mm flag", flag_thickness_mm_);

  if (!close_gripper()) {
    response->success = false;
    response->message = "Gripper close failed";
    return;
  }

  response->success = true;
  response->message = "Flag grasped";
  RCLCPP_INFO(get_logger(), "Flag grasped successfully");
}

bool FlagGrasperNode::rotate_wrist()
{
  const auto joint_names = move_group_interface_->getJointNames();
  auto joint_values = move_group_interface_->getCurrentJointValues();

  const auto it = std::find(joint_names.begin(), joint_names.end(), wrist_joint_name_);
  if (it == joint_names.end()) {
    RCLCPP_ERROR(
      get_logger(), "Joint '%s' not found in group '%s'",
      wrist_joint_name_.c_str(), planning_group_.c_str());
    return false;
  }

  const auto joint_index = static_cast<std::size_t>(std::distance(joint_names.begin(), it));
  joint_values[joint_index] += wrist_rotation_deg_ * M_PI / 180.0;

  move_group_interface_->setJointValueTarget(joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_interface_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Wrist rotation planning failed");
    return false;
  }

  if (move_group_interface_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Wrist rotation execution failed");
    return false;
  }

  return true;
}

bool FlagGrasperNode::close_gripper()
{
  if (!gripper_client_->wait_for_service(GRIPPER_SERVICE_WAIT_TIMEOUT)) {
    RCLCPP_ERROR(get_logger(), "Gripper service unavailable");
    return false;
  }

  const int position = compute_gripper_position(flag_thickness_mm_, gripper_max_aperture_mm_);
  RCLCPP_INFO(
    get_logger(), "Gripper close: position=%d (flag=%.1f mm / %.1f mm aperture)",
    position, flag_thickness_mm_, gripper_max_aperture_mm_);

  auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
  request->position = position;
  request->speed = gripper_speed_;
  request->force = gripper_force_;

  auto future = gripper_client_->async_send_request(request);
  if (future.wait_for(GRIPPER_RESPONSE_TIMEOUT) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Gripper service response timed out");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Gripper response: %s", future.get()->response.c_str());
  return true;
}

}  // namespace flag_grasper