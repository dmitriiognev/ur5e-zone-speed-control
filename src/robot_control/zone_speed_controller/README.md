# zone_speed_controller

Maps the operator's proximity zone to a UR5e speed limit, pausing the robot in the danger zone and on perception loss.

## Overview

`speed_controller_node` consumes the zone index from `operator_zones` and drives two outputs: the UR hardware speed slider and the latched pause flag for `lemniscate_executor_node`.
Zones 1 and above set the slider to the zone's speed fraction; zone 0 pauses the robot.
A perception watchdog applies the same pause when skeleton frames stop arriving, i.e. the Kinect driver or the pose detector died.
Zone changes take effect immediately in both directions.

## ROS interface

### Subscribed topics

| Topic | Type | Description |
| --- | --- | --- |
| `/operator/zone` | `std_msgs/Int32` | Operator zone index from `operator_zones` |
| `/pose/operator_skeleton` | `geometry_msgs/PoseArray` | Watchdog liveness signal; message content is unused |
| `/operator/collaborative_mode` | `std_msgs/Bool` (transient local) | Collaborative-mode flag from `gesture_detector` |

### Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/motion/paused` | `std_msgs/Bool` (transient local) | Pause flag consumed by `lemniscate_executor_node`; `false` is published once at startup |

### Services called

| Service | Type | Description |
| --- | --- | --- |
| `/io_and_status_controller/set_speed_slider` | `ur_msgs/SetSpeedSliderFraction` | UR driver speed slider; accepts fractions in $[0.01, 1.0]$ |

### Parameters

| Parameter | Default | Unit | Description |
| --- | --- | --- | --- |
| `num_zones` | `5` | — | Total zones including zone 0; must match `num_zones` in `operator_zones` |
| `skeleton_timeout` | `1.0` | s | Skeleton silence after which the watchdog pauses the robot |
| `watchdog_rate` | `10.0` | Hz | Watchdog check rate |
| `speed_slider_service` | `/io_and_status_controller/set_speed_slider` | — | UR speed slider service |
| `zone_topic` | `/operator/zone` | — | Zone input topic |
| `skeleton_topic` | `/pose/operator_skeleton` | — | Skeleton input topic |
| `collaborative_mode_topic` | `/operator/collaborative_mode` | — | Collaborative-mode input topic |
| `paused_topic` | `/motion/paused` | — | Pause flag output topic |

Defaults live in `config/speed_controller.yaml`, loaded by the launch file.

## Build

```bash
colcon build --packages-select zone_speed_controller
source install/setup.bash
```

## Run

```bash
ros2 launch zone_speed_controller speed_controller.launch.py
```

The node also runs standalone with its built-in defaults:

```bash
ros2 run zone_speed_controller speed_controller_node
```

## Speed mapping

Zone $i$ of $N$ maps to the slider fraction

$$
v_i = \frac{i}{N - 1}, \quad i = 1 \dots N - 1
$$

so the outermost zone always runs at full speed.
For the shipped defaults ($N = 5$) the speeds are $0.25, 0.5, 0.75, 1.0$.
Zone 0 has no slider value: it triggers the pause flag instead, and the slider keeps its last setting for the resume.

## State machine

| State | Meaning |
| --- | --- |
| `IDLE` | No zone received yet; pause flag `false` |
| `RUNNING` | Slider set to the current zone's speed |
| `PAUSED` | Pause flag `true`: operator in zone 0, or perception lost |
| `NONCOLLAB` | Collaborative mode off; zone processing frozen |

Transitions:

- Zone 0 leads to `PAUSED`; zone $k > 0$ leads to `RUNNING` at $v_k$, from `PAUSED` it also clears the pause flag.
- Collaborative mode off raises the pause flag and enters `NONCOLLAB` from any state; mode on returns to `IDLE` and clears the flag.
  Zone messages are ignored in `NONCOLLAB`, and the speed slider stays untouched so `lemniscate_executor_node` can drive it for its home move.
- Watchdog timeout in `IDLE` or `RUNNING` leads to `PAUSED`.

## Perception watchdog

The watchdog monitors `/pose/operator_skeleton` because `kinect_pose_detector` publishes it on every frame, with or without a detection: silence on this topic means the perception pipeline itself is down, while an empty workspace keeps the robot running.
It arms on the first received skeleton, so the controller can start before the perception pipeline.
On timeout the controller pauses the robot in place and logs an error: with monitoring lost the operator position is unknown, and any motion would be unmonitored.
The robot resumes at the next zone message, so recovery requires the operator to be detected again.