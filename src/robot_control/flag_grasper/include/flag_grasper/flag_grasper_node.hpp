#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <gripper_srv/srv/gripper_service.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace flag_grasper
{

class FlagGrasperNode : public rclcpp::Node
{
public:
  explicit FlagGrasperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Must be called after the executor has started spinning.
  void init_moveit();

private:
  void on_grasp_requested(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool rotate_wrist();
  bool close_gripper();

  double flag_thickness_mm_;
  double wrist_rotation_deg_;
  double gripper_max_aperture_mm_;
  int gripper_speed_;
  int gripper_force_;
  std::string planning_group_;
  std::string wrist_joint_name_;

  rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr gripper_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grasp_service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

}  // namespace flag_grasper