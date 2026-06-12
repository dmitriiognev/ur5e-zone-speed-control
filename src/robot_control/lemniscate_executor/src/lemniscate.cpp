#include "lemniscate_executor/lemniscate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace lemniscate_executor
{

static constexpr double MIN_SEGMENT_DURATION = 0.02;

std::vector<std::array<double, 3>> lemniscate_positions(
  const LemniscateShape & shape, int samples_per_loop)
{
  std::vector<std::array<double, 3>> positions;
  positions.reserve(static_cast<std::size_t>(samples_per_loop));

  for (int i = 0; i < samples_per_loop; ++i) {
    // Phase starts at pi/2 so the first via-point coincides with the centre pose.
    const double t =
      M_PI / 2.0 + 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples_per_loop);
    positions.push_back({
      shape.centre_x + shape.amplitude_x * std::cos(t),
      shape.centre_y,
      shape.centre_z + shape.amplitude_z * std::sin(t) * std::cos(t)});
  }
  return positions;
}

void unwrap_towards(std::vector<double> & joints, const std::vector<double> & previous)
{
  for (std::size_t j = 0; j < joints.size(); ++j) {
    while (joints[j] - previous[j] > M_PI) {joints[j] -= 2.0 * M_PI;}
    while (joints[j] - previous[j] < -M_PI) {joints[j] += 2.0 * M_PI;}
  }
}

std::vector<double> nominal_segment_durations(
  const std::vector<std::vector<double>> & via_points,
  const std::vector<double> & max_velocities,
  double velocity_scale)
{
  const std::size_t segment_count = via_points.size() - 1;
  std::vector<double> durations(segment_count, 0.0);

  for (std::size_t i = 0; i < segment_count; ++i) {
    double max_duration = 0.0;
    for (std::size_t j = 0; j < max_velocities.size(); ++j) {
      const double max_velocity = max_velocities[j] > 0.0 ? max_velocities[j] : 1.0;
      const double delta = std::abs(via_points[i + 1][j] - via_points[i][j]);
      max_duration = std::max(max_duration, delta / (velocity_scale * max_velocity));
    }
    durations[i] = std::max(max_duration, MIN_SEGMENT_DURATION);
  }
  return durations;
}

}  // namespace lemniscate_executor