#include "zone_speed_controller/speed_fsm.hpp"

#include <cstddef>
#include <stdexcept>

namespace zone_speed_controller
{

SpeedFsm::SpeedFsm(int num_zones, double skeleton_timeout)
: num_zones_(num_zones),
  skeleton_timeout_(skeleton_timeout)
{
  if (num_zones_ < 2) {
    throw std::invalid_argument("num_zones must be >= 2");
  }
  zone_speeds_.resize(static_cast<std::size_t>(num_zones_), 0.0);
  for (int zone = 1; zone < num_zones_; ++zone) {
    zone_speeds_[zone] = static_cast<double>(zone) / static_cast<double>(num_zones_ - 1);
  }
}

SpeedFsm::Command SpeedFsm::on_zone(int zone)
{
  if (state_ == State::NONCOLLAB || !is_valid_zone(zone)) {
    return {};
  }
  if (zone == 0) {
    return state_ == State::PAUSED ? Command{} : to_paused();
  }
  if (state_ == State::RUNNING && zone == current_zone_) {
    return {};
  }
  return to_running(zone);
}

SpeedFsm::Command SpeedFsm::on_collaborative_mode(bool collaborative)
{
  if (!collaborative && state_ != State::NONCOLLAB) {
    state_ = State::NONCOLLAB;
    current_zone_ = -1;
    Command command;
    command.paused = true;
    return command;
  }
  if (collaborative && state_ == State::NONCOLLAB) {
    state_ = State::IDLE;
    Command command;
    command.paused = false;
    return command;
  }
  return {};
}

void SpeedFsm::on_skeleton(double time_s)
{
  last_skeleton_time_ = time_s;
}

SpeedFsm::Command SpeedFsm::on_tick(double time_s)
{
  const bool watchdog_armed = last_skeleton_time_ >= 0.0;
  const bool motion_allowed = state_ == State::IDLE || state_ == State::RUNNING;
  if (!watchdog_armed || !motion_allowed) {
    return {};
  }
  if (time_s - last_skeleton_time_ <= skeleton_timeout_) {
    return {};
  }
  return to_paused();
}

bool SpeedFsm::is_valid_zone(int zone) const
{
  return zone >= 0 && zone < num_zones_;
}

SpeedFsm::State SpeedFsm::state() const
{
  return state_;
}

const char * SpeedFsm::state_name() const
{
  switch (state_) {
    case State::IDLE: return "IDLE";
    case State::RUNNING: return "RUNNING";
    case State::PAUSED: return "PAUSED";
    case State::NONCOLLAB: return "NONCOLLAB";
    default: return "UNKNOWN";
  }
}

SpeedFsm::Command SpeedFsm::to_running(int zone)
{
  Command command;
  command.speed = zone_speeds_[static_cast<std::size_t>(zone)];
  if (state_ == State::PAUSED) {
    command.paused = false;
  }
  state_ = State::RUNNING;
  current_zone_ = zone;
  return command;
}

SpeedFsm::Command SpeedFsm::to_paused()
{
  state_ = State::PAUSED;
  current_zone_ = -1;
  Command command;
  command.paused = true;
  return command;
}

}  // namespace zone_speed_controller