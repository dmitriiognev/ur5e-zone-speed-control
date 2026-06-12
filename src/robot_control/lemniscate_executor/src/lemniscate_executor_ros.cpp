#include "lemniscate_executor/lemniscate_executor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "lemniscate_executor/pose_utils.hpp"

namespace lemniscate_executor
{

namespace
{

// Accounts for accumulated dt rounding so the final trajectory point is always included.
constexpr double TIME_EPSILON = 1e-9;

const moveit::core::JointModelGroup * get_joint_model_group(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & group_name)
{
  return move_group.getRobotModel()->getJointModelGroup(group_name);
}

}  // namespace

LemniscateExecutorNode::LemniscateExecutorNode(const rclcpp::NodeOptions & options)
: Node("lemniscate_executor", options)
{
  config_.planning_group = declare_parameter<std::string>("planning_group", "ur5e_arm");
  config_.shape.centre_x = declare_parameter("centre.x", 1.19);
  config_.shape.centre_y = declare_parameter("centre.y", 0.20);
  config_.shape.centre_z = declare_parameter("centre.z", 0.50);
  config_.centre_roll = declare_parameter("centre.roll", -M_PI);
  config_.centre_pitch = declare_parameter("centre.pitch", M_PI_2);
  config_.centre_yaw = declare_parameter("centre.yaw", M_PI_2);
  config_.shape.amplitude_x = declare_parameter("amplitude.x", 0.30);
  config_.shape.amplitude_z = declare_parameter("amplitude.z", 0.20);
  config_.samples_per_loop = static_cast<int>(declare_parameter("samples_per_loop", 200));
  config_.nominal_velocity_scale = declare_parameter("nominal_velocity_scale", 0.25);
  config_.num_cycles = static_cast<int>(declare_parameter("num_cycles", 10));
  config_.dt = declare_parameter("dt", 0.01);
  config_.controller_name = declare_parameter<std::string>(
    "controller_name", "scaled_joint_trajectory_controller");
  const auto paused_topic = declare_parameter<std::string>("paused_topic", "/motion/paused");
  collaborative_mode_topic_ = declare_parameter<std::string>(
    "collaborative_mode_topic", "/operator/collaborative_mode");
  const auto speed_slider_service = declare_parameter<std::string>(
    "speed_slider_service", "/io_and_status_controller/set_speed_slider");

  validate_parameters();

  trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/" + config_.controller_name + "/follow_joint_trajectory");

  speed_slider_client_ = create_client<ur_msgs::srv::SetSpeedSliderFraction>(
    speed_slider_service);

  auto latched_qos = rclcpp::QoS(1).transient_local();
  paused_subscription_ = create_subscription<std_msgs::msg::Bool>(
    paused_topic, latched_qos,
    std::bind(&LemniscateExecutorNode::on_paused_received, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "LemniscateExecutorNode constructed. Controller: /%s/follow_joint_trajectory",
    config_.controller_name.c_str());

  // Safe to start now: the worker idles until a callback posts a job, and jobs
  // are only posted after run() initialises move_group_interface_.
  worker_thread_ = std::thread(&LemniscateExecutorNode::worker_loop, this);
}

LemniscateExecutorNode::~LemniscateExecutorNode()
{
  worker_stop_.store(true);
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void LemniscateExecutorNode::run()
{
  if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_ERROR(
      get_logger(), "Trajectory action server /%s/follow_joint_trajectory not available.",
      config_.controller_name.c_str());
    rclcpp::shutdown();
    return;
  }

  if (speed_slider_client_->wait_for_service(std::chrono::seconds(5))) {
    auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    request->speed_slider_fraction = 1.0;
    speed_slider_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Speed slider reset to 1.0 before startup moves.");
  } else {
    RCLCPP_WARN(get_logger(), "Speed slider service not available.");
  }

  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), config_.planning_group);
  auto & move_group = *move_group_interface_;

  RCLCPP_INFO(get_logger(), "Planning frame : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(get_logger(), "End effector   : %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(
    get_logger(), "Lemniscate centre [%.3f, %.3f, %.3f], amp X: %.3f, amp Z: %.3f",
    config_.shape.centre_x, config_.shape.centre_y, config_.shape.centre_z,
    config_.shape.amplitude_x, config_.shape.amplitude_z);

  {
    RCLCPP_INFO(get_logger(), "Moving to home position...");
    move_group.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to plan to home, aborting");
      rclcpp::shutdown();
      return;
    }
    if (!execute_plan_via_action(plan, "home")) {
      RCLCPP_ERROR(get_logger(), "Failed to execute home move, aborting");
      rclcpp::shutdown();
      return;
    }
    move_group.clearPoseTargets();
    RCLCPP_INFO(get_logger(), "Reached home position");
  }

  {
    geometry_msgs::msg::Pose centre = pose_from_rpy(
      config_.shape.centre_x, config_.shape.centre_y, config_.shape.centre_z,
      config_.centre_roll, config_.centre_pitch, config_.centre_yaw);
    move_group.setPoseTarget(centre);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "Centre pose [%.3f, %.3f, %.3f] RPY [%.3f, %.3f, %.3f] is NOT reachable. "
        "Adjust centre.* params.",
        config_.shape.centre_x, config_.shape.centre_y, config_.shape.centre_z,
        config_.centre_roll, config_.centre_pitch, config_.centre_yaw);
      rclcpp::shutdown();
      return;
    }
    if (!execute_plan_via_action(plan, "centre pose")) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to execute move to centre pose - check orientation params or arm config.");
      rclcpp::shutdown();
      return;
    }
    move_group.clearPoseTargets();
    RCLCPP_INFO(get_logger(), "Centre pose reached - starting spline setup");
  }

  try {
    solve_ik_for_waypoints(move_group);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "IK pipeline failed: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  compute_nominal_durations(move_group);
  compute_spline_coefficients();

  // Created after move_group_interface_ exists: the callback's transitions use it.
  {
    auto latched_qos = rclcpp::QoS(1).transient_local();
    collaborative_mode_subscription_ = create_subscription<std_msgs::msg::Bool>(
      collaborative_mode_topic_, latched_qos,
      std::bind(
        &LemniscateExecutorNode::on_collaborative_mode_received, this, std::placeholders::_1));
  }

  send_trajectory_goal(0.0);

  RCLCPP_INFO(
    get_logger(),
    "run() complete - trajectory active, node spinning. Nominal cycle time: %.2f s.",
    nominal_cycle_time_);
}

void LemniscateExecutorNode::on_paused_received(std_msgs::msg::Bool::SharedPtr msg)
{
  const bool paused = msg->data;
  bool do_cancel = false;
  bool do_resume = false;
  double phase = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // During a home transition the worker is executing MoveGroup moves; cancelling
    // here would race execute_plan_via_action and corrupt the action-client state.
    if (transition_active_) {
      return;
    }

    is_paused_ = paused;
    if (paused) {
      do_cancel = true;
    } else if (waiting_to_resume_) {
      waiting_to_resume_ = false;
      do_resume = true;
      phase = resume_phase_;
    }
  }

  // Act outside the lock - state_mutex_ must never be held across an action call.
  if (do_cancel) {
    // is_paused_ is already set, so the CANCELED result callback saves the resume phase.
    trajectory_action_client_->async_cancel_all_goals();
  } else if (do_resume) {
    RCLCPP_INFO(
      get_logger(), "[motion] Unpaused - resuming trajectory from phase %.3f s.", phase);
    send_trajectory_goal(phase);
  }
}

void LemniscateExecutorNode::on_collaborative_mode_received(std_msgs::msg::Bool::SharedPtr msg)
{
  const bool collaborative = msg->data;
  bool resuming = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (collaborative == collaborative_mode_) {
      return;
    }
    collaborative_mode_ = collaborative;

    if (collaborative) {
      resuming = true;
      transition_active_ = true;
      pending_ = Transition::GoHomeResume;
    }
    // Entering non-collab needs no job here: zone_speed_controller publishes
    // paused=true, the pause handler cancels the goal, and the CANCELED result
    // callback posts GoHomeWait.
  }

  RCLCPP_INFO(
    get_logger(), "%s", resuming
    ? "[collab] Exiting non-collaborative mode - resuming."
    : "[collab] Entering non-collaborative mode.");
}

void LemniscateExecutorNode::generate_cartesian_waypoints()
{
  const auto positions = lemniscate_positions(config_.shape, config_.samples_per_loop);

  waypoints_.clear();
  waypoints_.reserve(positions.size());
  for (const auto & position : positions) {
    waypoints_.push_back(
      pose_from_rpy(
        position[0], position[1], position[2],
        config_.centre_roll, config_.centre_pitch, config_.centre_yaw));
  }
}

void LemniscateExecutorNode::solve_ik_for_waypoints(
  moveit::planning_interface::MoveGroupInterface & move_group)
{
  generate_cartesian_waypoints();

  const auto * joint_model_group = get_joint_model_group(move_group, config_.planning_group);
  joint_names_ = joint_model_group->getActiveJointModelNames();

  // Seed IK from the current robot state so the solver starts near a valid solution.
  moveit::core::RobotStatePtr state = move_group.getCurrentState();

  via_points_.clear();
  via_points_.reserve(waypoints_.size() + 1);

  RCLCPP_INFO(get_logger(), "Solving IK for %zu waypoints...", waypoints_.size());

  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    if (!state->setFromIK(joint_model_group, waypoints_[i], 0.1)) {
      throw std::runtime_error(
              "IK failed at waypoint " + std::to_string(i) +
              " - reduce amplitudes or adjust centre pose");
    }

    std::vector<double> joint_positions;
    state->copyJointGroupPositions(joint_model_group, joint_positions);

    if (!via_points_.empty()) {
      unwrap_towards(joint_positions, via_points_.back());
    }
    via_points_.push_back(joint_positions);

    // Seed the next IK call from the unwrapped solution.
    state->setJointGroupPositions(joint_model_group, joint_positions);
  }

  // Periodic closure q_N = q_0, required by CubicSpline::fit.
  via_points_.push_back(via_points_.front());

  RCLCPP_INFO(get_logger(), "IK solved for %zu unique via-points.", via_points_.size() - 1);
}

void LemniscateExecutorNode::compute_nominal_durations(
  moveit::planning_interface::MoveGroupInterface & move_group)
{
  const auto * joint_model_group = get_joint_model_group(move_group, config_.planning_group);
  const auto & joint_models = joint_model_group->getActiveJointModels();

  std::vector<double> max_velocities;
  max_velocities.reserve(joint_models.size());
  for (const auto * joint_model : joint_models) {
    max_velocities.push_back(joint_model->getVariableBounds()[0].max_velocity_);
  }

  nominal_durations_ = nominal_segment_durations(
    via_points_, max_velocities, config_.nominal_velocity_scale);
  nominal_cycle_time_ = std::accumulate(
    nominal_durations_.begin(), nominal_durations_.end(), 0.0);

  RCLCPP_INFO(
    get_logger(), "Nominal cycle time: %.2f s (%zu segments, velocity scale %.2f).",
    nominal_cycle_time_, nominal_durations_.size(), config_.nominal_velocity_scale);
}

void LemniscateExecutorNode::compute_spline_coefficients()
{
  const int joint_count = static_cast<int>(joint_names_.size());
  const int segment_count = static_cast<int>(nominal_durations_.size());
  spline_coefficients_.resize(joint_count);

  for (int j = 0; j < joint_count; ++j) {
    std::vector<double> joint_values(segment_count + 1);
    for (int i = 0; i <= segment_count; ++i) {
      joint_values[i] = via_points_[i][j];
    }
    spline_coefficients_[j] = CubicSpline::fit(joint_values, nominal_durations_);
  }

  RCLCPP_INFO(
    get_logger(), "Spline fitted: %d joints x %d segments.", joint_count, segment_count);
}

trajectory_msgs::msg::JointTrajectory
LemniscateExecutorNode::build_trajectory(double start_phase_nominal) const
{
  trajectory_msgs::msg::JointTrajectory msg;
  // header.stamp stays empty: the controller uses goal-acceptance time as t=0.
  msg.joint_names = joint_names_;

  const int joint_count = static_cast<int>(joint_names_.size());
  const double total_time = nominal_cycle_time_ * config_.num_cycles;

  for (double t = 0.0; t <= total_time + TIME_EPSILON; t += config_.dt) {
    const double t_nominal = std::fmod(start_phase_nominal + t, nominal_cycle_time_);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(joint_count);
    point.velocities.resize(joint_count);
    point.accelerations.resize(joint_count);

    for (int j = 0; j < joint_count; ++j) {
      double position, velocity, acceleration;
      CubicSpline::evaluate(spline_coefficients_[j], t_nominal, position, velocity, acceleration);
      point.positions[j] = position;
      point.velocities[j] = velocity;
      point.accelerations[j] = acceleration;
    }

    point.time_from_start = rclcpp::Duration::from_seconds(t);
    msg.points.push_back(std::move(point));
  }

  // Zero the final velocities so the arm comes cleanly to rest if a goal ever
  // runs to completion; proactive stitching normally replaces the goal earlier.
  if (!msg.points.empty()) {
    auto & last = msg.points.back();
    std::fill(last.velocities.begin(), last.velocities.end(), 0.0);
    std::fill(last.accelerations.begin(), last.accelerations.end(), 0.0);
  }

  return msg;
}

void LemniscateExecutorNode::send_trajectory_goal(double start_phase_nominal)
{
  if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(
      get_logger(), "Action server /%s/follow_joint_trajectory not available.",
      config_.controller_name.c_str());
    return;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = build_trajectory(start_phase_nominal);

  RCLCPP_INFO(
    get_logger(), "Sending trajectory goal: %zu points, %.1f s (phase %.2f s).",
    goal.trajectory.points.size(), nominal_cycle_time_ * config_.num_cycles,
    start_phase_nominal);

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions goal_options;

  goal_options.feedback_callback =
    [this, start_phase_nominal](
    const GoalHandle::SharedPtr &,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> & feedback) {
      on_trajectory_feedback(
        start_phase_nominal, rclcpp::Duration(feedback->desired.time_from_start).seconds());
    };

  goal_options.goal_response_callback =
    [this, start_phase_nominal](const GoalHandle::SharedPtr & handle) {
      on_goal_response(handle, start_phase_nominal);
    };

  goal_options.result_callback =
    [this, start_phase_nominal](const GoalHandle::WrappedResult & wrapped_result) {
      on_trajectory_result(wrapped_result, start_phase_nominal);
    };

  trajectory_action_client_->async_send_goal(goal, goal_options);
}

void LemniscateExecutorNode::on_trajectory_feedback(
  double start_phase_nominal, double elapsed_seconds)
{
  // desired.time_from_start is trajectory time, already compensated for the UR
  // speed slider scaling applied by scaled_joint_trajectory_controller.
  const double phase = std::fmod(start_phase_nominal + elapsed_seconds, nominal_cycle_time_);

  bool send_next = false;
  double resume_from = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_known_phase_ = phase;

    // Proactive stitching: once this goal enters its final cycle, queue a fresh
    // goal from the current phase. Skipped while paused / non-collab / transitioning
    // so a successor never fights the pause or go-home paths.
    const bool last_cycle =
      elapsed_seconds >= (config_.num_cycles - 1) * nominal_cycle_time_;
    if (!last_cycle) {
      next_goal_queued_ = false;
    } else if (!next_goal_queued_ && collaborative_mode_ && !is_paused_ &&
      !transition_active_)
    {
      next_goal_queued_ = true;
      send_next = true;
      resume_from = phase;
    }
  }

  // Send outside the lock - state_mutex_ must never be held across the call.
  if (send_next) {
    send_trajectory_goal(resume_from);
  }
}

void LemniscateExecutorNode::on_goal_response(
  GoalHandle::SharedPtr handle, double start_phase_nominal)
{
  if (!handle) {
    // Controller not yet active (e.g. ExternalControl URCap just starting).
    RCLCPP_WARN(get_logger(), "Goal rejected (controller not active). Retrying in 2 s...");
    retry_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this, start_phase_nominal]() {
        retry_timer_->cancel();
        retry_timer_.reset();
        send_trajectory_goal(start_phase_nominal);
      });
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_goal_id_ = handle->get_goal_id();
}

void LemniscateExecutorNode::on_trajectory_result(
  const GoalHandle::WrappedResult & wrapped_result, double start_phase_nominal)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (wrapped_result.goal_id != latest_goal_id_) {
      // Superseded by a proactively-queued successor; the stale result must not
      // trigger a spurious re-send, go-home, or pause-resume.
      return;
    }
  }
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
        const double next_phase = std::fmod(
          start_phase_nominal + config_.num_cycles * nominal_cycle_time_,
          nominal_cycle_time_);
        bool hold = false;
        bool resend = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (!collaborative_mode_) {
            pending_ = Transition::GoHomeWait;
          } else if (is_paused_) {
            hold = true;
            resume_phase_ = next_phase;
            waiting_to_resume_ = true;
          } else {
            resend = true;
          }
        }
        if (hold) {
          RCLCPP_INFO(
            get_logger(), "[motion] Paused after completion - holding at phase %.3f s.",
            next_phase);
        } else if (resend) {
          RCLCPP_INFO(
            get_logger(), "Trajectory completed. Re-sending from phase %.3f s.", next_phase);
          send_trajectory_goal(next_phase);
        }
        break;
      }
    case rclcpp_action::ResultCode::CANCELED: {
        bool paused_resume = false;
        bool plain_cancel = false;
        double phase = 0.0;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (!collaborative_mode_) {
            pending_ = Transition::GoHomeWait;
          } else if (is_paused_) {
            // Cancelled by the pause handler on zone 0: save phase and wait for unpause.
            resume_phase_ = last_known_phase_;
            waiting_to_resume_ = true;
            paused_resume = true;
            phase = resume_phase_;
          } else {
            plain_cancel = true;
          }
        }
        if (paused_resume) {
          RCLCPP_INFO(
            get_logger(),
            "[motion] Goal cancelled (pause) - will resume from phase %.3f s on unpause.",
            phase);
        } else if (plain_cancel) {
          RCLCPP_INFO(get_logger(), "[motion] Trajectory goal cancelled.");
        }
        break;
      }
    default:
      RCLCPP_ERROR(
        get_logger(), "Trajectory failed (code %d).", static_cast<int>(wrapped_result.code));
      break;
  }
}

void LemniscateExecutorNode::worker_loop()
{
  using namespace std::chrono_literals;
  while (!worker_stop_.load()) {
    Transition job = Transition::None;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      job = pending_;
      pending_ = Transition::None;
    }
    switch (job) {
      case Transition::GoHomeWait: go_home_and_wait(); break;
      case Transition::GoHomeResume: go_home_and_resume(); break;
      case Transition::None: break;
    }
    std::this_thread::sleep_for(50ms);
  }
}

void LemniscateExecutorNode::set_speed_slider(double fraction)
{
  if (!speed_slider_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Speed slider service not available - skipping.");
    return;
  }
  auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
  request->speed_slider_fraction = std::clamp(fraction, 0.01, 1.0);
  speed_slider_client_->async_send_request(request).wait();
}

bool LemniscateExecutorNode::execute_plan_via_action(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const std::string & label)
{
  FollowJointTrajectory::Goal goal;
  goal.trajectory = plan.trajectory.joint_trajectory;

  auto result_promise = std::make_shared<std::promise<bool>>();
  auto result_future = result_promise->get_future();

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions goal_options;
  goal_options.result_callback =
    [result_promise](const GoalHandle::WrappedResult & wrapped_result) {
      result_promise->set_value(wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  auto goal_handle_future = trajectory_action_client_->async_send_goal(goal, goal_options);
  if (!goal_handle_future.get()) {
    RCLCPP_ERROR(get_logger(), "%s: goal rejected by controller", label.c_str());
    return false;
  }
  const bool succeeded = result_future.get();
  if (!succeeded) {
    RCLCPP_ERROR(get_logger(), "%s: controller reported failure", label.c_str());
  }
  return succeeded;
}

void LemniscateExecutorNode::go_home_and_wait()
{
  RCLCPP_INFO(get_logger(), "[non-collab] Setting speed slider to 1.0 before home move.");
  set_speed_slider(1.0);

  auto & move_group = *move_group_interface_;

  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "[non-collab] Failed to plan to home.");
    return;
  }
  if (!execute_plan_via_action(plan, "home (non-collab)")) {
    RCLCPP_ERROR(get_logger(), "[non-collab] Home move failed.");
    return;
  }
  move_group.clearPoseTargets();
  RCLCPP_INFO(get_logger(), "[non-collab] At home - waiting for collaborative mode gesture.");
}

void LemniscateExecutorNode::go_home_and_resume()
{
  RCLCPP_INFO(get_logger(), "[collab-resume] Setting speed slider to 1.0 before moves.");
  set_speed_slider(1.0);

  auto & move_group = *move_group_interface_;

  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  if (move_group.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS) {
    execute_plan_via_action(plan_home, "home (collab-resume)");
  } else {
    RCLCPP_WARN(get_logger(), "[collab-resume] Could not plan to home - skipping.");
  }
  move_group.clearPoseTargets();

  // waypoints_[0] is the Cartesian pose at phase 0; moving there first avoids a
  // large initial jump in the trajectory goal.
  move_group.setPoseTarget(waypoints_[0]);
  moveit::planning_interface::MoveGroupInterface::Plan plan_start;
  if (move_group.plan(plan_start) == moveit::core::MoveItErrorCode::SUCCESS) {
    execute_plan_via_action(plan_start, "lemniscate start (collab-resume)");
  } else {
    RCLCPP_WARN(get_logger(), "[collab-resume] Could not plan to lemniscate start.");
  }
  move_group.clearPoseTargets();

  RCLCPP_INFO(get_logger(), "[collab-resume] Starting trajectory at phase 0.");
  send_trajectory_goal(0.0);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    transition_active_ = false;
  }
}

void LemniscateExecutorNode::validate_parameters() const
{
  if (config_.samples_per_loop < 4) {
    throw std::invalid_argument(
            "samples_per_loop must be >= 4, got " + std::to_string(config_.samples_per_loop));
  }
  if (config_.nominal_velocity_scale <= 0.0 || config_.nominal_velocity_scale > 1.0) {
    throw std::invalid_argument(
            "nominal_velocity_scale must be in (0, 1], got " +
            std::to_string(config_.nominal_velocity_scale));
  }
  if (config_.num_cycles <= 0) {
    throw std::invalid_argument(
            "num_cycles must be > 0, got " + std::to_string(config_.num_cycles));
  }
  if (config_.dt <= 0.0) {
    throw std::invalid_argument("dt must be > 0, got " + std::to_string(config_.dt));
  }
}

}  // namespace lemniscate_executor