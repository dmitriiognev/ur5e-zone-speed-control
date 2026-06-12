#pragma once

#include <optional>
#include <vector>

namespace zone_speed_controller
{

class SpeedFsm
{
public:
  enum class State {IDLE, RUNNING, PAUSED, NONCOLLAB};

  /// Side effects of a transition; the caller must apply speed before paused.
  struct Command
  {
    std::optional<double> speed;
    std::optional<bool> paused;
  };

  /// Throws std::invalid_argument if num_zones < 2.
  SpeedFsm(int num_zones, double skeleton_timeout);

  Command on_zone(int zone);
  Command on_collaborative_mode(bool collaborative);
  void on_skeleton(double time_s);
  Command on_tick(double time_s);

  bool is_valid_zone(int zone) const;
  State state() const;
  const char * state_name() const;

private:
  Command to_running(int zone);
  Command to_paused();

  int num_zones_;
  double skeleton_timeout_;
  std::vector<double> zone_speeds_;
  State state_ {State::IDLE};
  int current_zone_ {-1};
  double last_skeleton_time_ {-1.0};
};

}  // namespace zone_speed_controller