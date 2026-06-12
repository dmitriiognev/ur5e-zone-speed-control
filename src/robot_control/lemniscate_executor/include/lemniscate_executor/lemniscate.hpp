#pragma once

#include <array>
#include <vector>

namespace lemniscate_executor
{

/// Gerono lemniscate in the XZ plane: x(t) = cx + ax*cos(t), z(t) = cz + az*sin(t)*cos(t).
struct LemniscateShape
{
  double centre_x;
  double centre_y;
  double centre_z;
  double amplitude_x;
  double amplitude_z;
};

std::vector<std::array<double, 3>> lemniscate_positions(
  const LemniscateShape & shape, int samples_per_loop);

/// Shifts each joint by multiples of 2*pi onto the branch closest to `previous`.
void unwrap_towards(std::vector<double> & joints, const std::vector<double> & previous);

/// Duration of each via-point segment so the fastest joint moves at velocity_scale
/// times its maximum velocity; short segments are floored to avoid degenerate durations.
std::vector<double> nominal_segment_durations(
  const std::vector<std::vector<double>> & via_points,
  const std::vector<double> & max_velocities,
  double velocity_scale);

}  // namespace lemniscate_executor