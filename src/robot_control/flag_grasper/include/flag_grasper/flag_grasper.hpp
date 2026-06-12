#pragma once

namespace flag_grasper
{

/// Maps flag thickness to a Robotiq 2F position byte: 0 = fully open, 255 = fully closed.
int compute_gripper_position(double flag_thickness_mm, double gripper_max_aperture_mm);

}  // namespace flag_grasper