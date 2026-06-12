# flag_grasper

Rotates the UR5e wrist and closes the Robotiq gripper to grasp a flag of configured thickness.

## Overview

`flag_grasper_node` exposes a single trigger service `/flag_grasp`.
On each call it rotates the wrist joint by `wrist_rotation_deg` through a MoveIt joint-space plan, then closes the Robotiq 2F-85 gripper until the gap matches `flag_thickness_mm`.

## ROS interface

### Service servers

| Service | Type | Description |
| --- | --- | --- |
| `/flag_grasp` | `std_srvs/Trigger` | Runs the grasp sequence |

### Service clients

| Service | Type | Description |
| --- | --- | --- |
| `gripper_service` | `gripper_srv/GripperService` | Robotiq position/speed/force command, served by the `cocohrip` gripper node |

### Parameters

| Parameter | Default | Unit | Description |
| --- | --- | --- | --- |
| `flag_thickness_mm` | `3.0` | mm | Flag pole thickness; sets the target gripper gap |
| `gripper_max_aperture_mm` | `85.0` | mm | Robotiq 2F-85 maximum aperture |
| `wrist_rotation_deg` | `90.0` | deg | Angle added to the wrist joint, positive = CCW from the tool-tip view |
| `wrist_joint_name` | `ur5e_wrist_3_joint` | — | Joint rotated by the grasp sequence; must belong to `planning_group` |
| `gripper_speed` | `128` | 0–255 | Robotiq speed byte |
| `gripper_force` | `100` | 0–255 | Robotiq force byte |
| `planning_group` | `ur5e_arm` | — | MoveIt planning group |
| `gripper_service` | `gripper_service` | — | Name of the Robotiq command service |

Defaults live in `config/flag_grasper.yaml`, loaded by the launch file.
Parameters are read once at startup; `ros2 param set` has no effect on a running node — edit the YAML and restart instead.

## Build

```bash
colcon build --packages-select flag_grasper
source install/setup.bash
```

## Run

The UR5e MoveIt stack (`move_group`) and the `cocohrip` gripper service node must already be running.

```bash
ros2 launch flag_grasper flag_grasper.launch.py
```

Trigger the grasp:

```bash
ros2 service call /flag_grasp std_srvs/srv/Trigger
```

## Gripper position mapping

The Robotiq 2F position byte runs from 0 (fully open, `gripper_max_aperture_mm`) to 255 (fully closed).
The target byte closes the gripper until the gap equals the flag thickness:

$$
p = \mathrm{round}\left(255 \cdot \mathrm{clamp}\left(1 - \frac{t}{a},\ 0,\ 1\right)\right)
$$

where $t$ is `flag_thickness_mm` and $a$ is `gripper_max_aperture_mm`.
For the shipped defaults ($t = 3$, $a = 85$) the command byte is $246$.

## Startup and concurrency

`MoveGroupInterface` cannot be created in the node constructor (`shared_from_this()` is forbidden there) and needs an already-spinning executor to fetch the robot state.
`main()` therefore starts a `MultiThreadedExecutor` in a background thread and calls `init_moveit()` afterwards.
A service call arriving before initialization finishes is rejected with "MoveGroupInterface not initialized".

The grasp service runs in a reentrant callback group: its handler blocks on MoveIt plan/execute and on the gripper response, while the executor keeps processing MoveIt's own callbacks in parallel.