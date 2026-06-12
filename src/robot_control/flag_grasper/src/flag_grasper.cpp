#include "flag_grasper/flag_grasper.hpp"

#include <algorithm>
#include <cmath>

namespace flag_grasper
{

int compute_gripper_position(double flag_thickness_mm, double gripper_max_aperture_mm)
{
  const double closed_ratio = 1.0 - (flag_thickness_mm / gripper_max_aperture_mm);
  return static_cast<int>(std::round(255.0 * std::clamp(closed_ratio, 0.0, 1.0)));
}

}  // namespace flag_grasper