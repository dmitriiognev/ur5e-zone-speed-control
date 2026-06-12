# operator_detection_common

Shared, ROS-free constants and helpers that the producer and consumers of the operator skeleton must agree on.

Centralised here so `kinect_pose_detector`, `operator_zones`, and `gesture_detector` stay synchronized.

## API

`KEYPOINT_NAMES` - the 33 MediaPipe Pose landmark names in model output order (index `0..32`).
The index is the contract: `/pose/operator_skeleton` carries exactly 33 `Pose` entries in this order, so `KEYPOINT_NAMES[i]` maps any slot back to its landmark.

`is_invalid_joint(position)` - `True` when a joint carries the `(0, 0, 0)` "missing / invalid" sentinel.
`kinect_pose_detector` writes `(0, 0, 0)` for any joint whose 3D position could not be determined (low visibility, missing depth, stale timeout, cross-joint outlier); consumers use this helper instead of hard-coding the threshold.

The full skeleton contract is documented in the [`kinect_pose_detector` README](../kinect_pose_detector/README.md#design-notes), the package that produces the skeleton.
