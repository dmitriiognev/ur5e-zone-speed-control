#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lemniscate_executor
{

/// Build a Pose from position (x, y, z) and orientation given as RPY angles.
inline geometry_msgs::msg::Pose pose_from_rpy(
  double x, double y, double z,
  double r, double p, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(r, p, yaw);

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf2::toMsg(q);
  return pose;
}

}  // namespace lemniscate_executor
