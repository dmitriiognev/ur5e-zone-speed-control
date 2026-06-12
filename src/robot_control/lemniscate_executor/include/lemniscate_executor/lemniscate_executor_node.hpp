#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ur_msgs/srv/set_speed_slider_fraction.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "lemniscate_executor/cubic_spline.hpp"
#include "lemniscate_executor/lemniscate.hpp"

namespace lemniscate_executor
{

class LemniscateExecutorNode : public rclcpp::Node
{
public:
  /// Static motion parameters populated once from ROS parameters at startup.
  struct Config
  {
    std::string planning_group;
    LemniscateShape shape;
    double centre_roll;
    double centre_pitch;
    double centre_yaw;
    int samples_per_loop;
    double nominal_velocity_scale;
    int num_cycles;
    double dt;
    std::string controller_name;
  };

  explicit LemniscateExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~LemniscateExecutorNode();

  /// One-time startup: home, centre pose, IK, spline fit, first trajectory goal.
  /// Must be called from a thread that is NOT the executor spin thread,
  /// with the executor already spinning.
  void run();

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  // Topic / action / service handlers
  void on_paused_received(std_msgs::msg::Bool::SharedPtr msg);
  void on_collaborative_mode_received(std_msgs::msg::Bool::SharedPtr msg);
  void on_trajectory_feedback(double start_phase_nominal, double elapsed_seconds);
  void on_goal_response(GoalHandle::SharedPtr handle, double start_phase_nominal);
  void on_trajectory_result(
    const GoalHandle::WrappedResult & wrapped_result, double start_phase_nominal);

  // Startup pipeline
  void generate_cartesian_waypoints();
  void solve_ik_for_waypoints(moveit::planning_interface::MoveGroupInterface & move_group);
  void compute_nominal_durations(moveit::planning_interface::MoveGroupInterface & move_group);
  void compute_spline_coefficients();
  void validate_parameters() const;

  // Trajectory
  trajectory_msgs::msg::JointTrajectory build_trajectory(double start_phase_nominal) const;
  void send_trajectory_goal(double start_phase_nominal);

  // Collaborative-mode transitions (run on the worker thread)
  void worker_loop();
  void go_home_and_wait();
  void go_home_and_resume();

  /// Blocks the calling worker thread until the slider request is acknowledged;
  /// the executor keeps spinning and processes the response.
  void set_speed_slider(double fraction);

  /// Routes a MoveIt plan through the trajectory action (blocking); bypasses
  /// MoveIt's execution deadline, which misfires when the UR slider is below 1.0.
  bool execute_plan_via_action(
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::string & label);

  // ROS interface
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr paused_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collaborative_mode_subscription_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;

  // MoveIt interface, initialised in run(), reused by the worker thread
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  /// Single owned worker runs the long-blocking MoveIt transitions off the
  /// executor; callbacks post the latest job into pending_, the destructor joins.
  enum class Transition {None, GoHomeWait, GoHomeResume};
  std::thread worker_thread_;
  std::atomic<bool> worker_stop_ {false};

  /// Guards every field below; held only briefly, never across a MoveIt or action call.
  std::mutex state_mutex_;
  Transition pending_ {Transition::None};
  bool transition_active_ {false};   ///< home transition in flight; pause skips cancel
  bool collaborative_mode_ {true};
  bool is_paused_ {false};
  bool waiting_to_resume_ {false};   ///< goal ended while paused; resume on unpause
  double resume_phase_ {0.0};
  double last_known_phase_ {0.0};

  /// Proactive stitching: a successor goal is sent one cycle before the current
  /// goal ends, so the controller's point buffer never drains at the boundary.
  bool next_goal_queued_ {false};
  rclcpp_action::GoalUUID latest_goal_id_ {};   ///< filters stale superseded results

  Config config_;
  std::string collaborative_mode_topic_;   ///< subscribed late, in run(), after MoveIt startup

  // Spline data, computed once in run()
  std::vector<geometry_msgs::msg::Pose> waypoints_;
  std::vector<std::string> joint_names_;
  std::vector<std::vector<double>> via_points_;
  std::vector<double> nominal_durations_;
  std::vector<std::vector<SplineSegment>> spline_coefficients_;
  double nominal_cycle_time_ {0.0};
};

}  // namespace lemniscate_executor