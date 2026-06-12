# save_kinect_rgbd

Captures one synchronized Kinect v2 RGB + depth frame pair and saves it to disk.

## Overview

`save_rgbd_node` waits until both Kinect v2 streams have delivered at least one frame, writes the latest pair to disk, and shuts down, so only one capture per run.
The pair is not hardware-synchronized: at the Kinect v2 frame rate of 30 Hz the frames are at most $\approx 33$ ms apart.

## ROS interface

### Subscribed topics

| Topic | Type | Description |
| --- | --- | --- |
| `/kinect2/sd/image_color_rect` | `sensor_msgs/Image` | Rectified RGB stream, `bgr8` |
| `/kinect2/sd/image_depth_rect` | `sensor_msgs/Image` | Rectified depth stream; must be `16UC1` (millimeters), other encodings are rejected |

Both subscriptions use sensor-data QoS (best effort, keep last 5), matching the Kinect v2 driver publishers.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `rgb_topic` | `/kinect2/sd/image_color_rect` | RGB input topic |
| `depth_topic` | `/kinect2/sd/image_depth_rect` | Depth input topic |
| `output_directory` | `.` | Directory the pair is written to; a relative path resolves against the working directory; must exist |

## Build

```bash
colcon build --packages-select save_kinect_rgbd
source install/setup.bash
```

## Run

By default the pair is saved to the current working directory:

```bash
ros2 run save_kinect_rgbd save_rgbd_node
```

`output_directory` redirects the capture to any existing directory:

```bash
ros2 run save_kinect_rgbd save_rgbd_node --ros-args -p output_directory:=docs/test_images_no_calibration
```

## Output naming

- `kinect_rgb_<N>.jpg` — RGB frame, JPEG quality 95
- `kinect_depth_<N>.png` — depth frame, 16-bit PNG

`<N>` is the highest number among existing `kinect_rgb_<N>.jpg` files in `output_directory` plus one, so previous captures are never overwritten.