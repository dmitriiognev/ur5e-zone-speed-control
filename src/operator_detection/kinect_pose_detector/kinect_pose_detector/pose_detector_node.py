#!/usr/bin/env python3
"""ROS2 node: MediaPipe 3D operator pose detection from a Kinect v2 RGB-D stream."""
from collections import deque
from dataclasses import dataclass
import os
import signal
import statistics
import sys
import time
import traceback
from typing import List
from typing import Optional
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import RunningMode
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
import numpy as np
from operator_detection_common.keypoints import KEYPOINT_NAMES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Fixed lookup tables (Kinect v2 hardware modes / bundled model files).
VALID_RESOLUTIONS = ('sd', 'qhd', 'hd')
MODEL_FILENAMES = {
    'lite': 'pose_landmarker_lite.task',
    'full': 'pose_landmarker_full.task',
    'heavy': 'pose_landmarker_heavy.task',
}
DEPTH_MM_PER_M = 1000.0
MIN_JOINTS_FOR_GLOBAL_FILTER = 3

# Visualization constants (debug overlays + RViz markers; not runtime-tuned).
SKELETON_LINE_COLOR_BGR = (255, 0, 0)
SKELETON_LINE_THICKNESS_PX = 2
RGB_JOINT_COLOR_BGR = (0, 0, 255)
JOINT_CIRCLE_RADIUS_PX = 4
DISTANCE_LABEL_FONT_SCALE = 0.3
DISTANCE_LABEL_COLOR_BGR = (0, 255, 0)
DISTANCE_LABEL_OFFSET_PX = (5, -5)
DISTANCE_LABEL_THICKNESS_PX = 1
MARKER_JOINT_COLOR_RGBA = (0.0, 1.0, 0.0, 1.0)
MARKER_BONE_COLOR_RGBA = (0.0, 0.0, 1.0, 1.0)
MARKER_JOINT_SCALE_M = 0.05
MARKER_BONE_SCALE_M = 0.02
MARKER_LIFETIME_NS = 300_000_000
BONE_MARKER_ID_OFFSET = 1000

# Logging / reporting cadence (in frames / seconds).
DEPTH_HISTORY_CLEANUP_INTERVAL = 100
FPS_WINDOW = 30
FPS_REPORT_INTERVAL = 30
LOG_THROTTLE_SEC = 5.0


def iir_filtered(new_value: float, previous_filtered: float, alpha: float) -> float:
    """First-order IIR low-pass: y[n] = alpha*x[n] + (1-alpha)*y[n-1]."""
    return alpha * new_value + (1.0 - alpha) * previous_filtered


@dataclass
class Joint3D:
    """A skeleton joint: pixel position, 3D camera-frame position (m), and IIR-filtered position."""

    name: str
    pixel_x: float
    pixel_y: float
    cam_x: float
    cam_y: float
    cam_z: float
    cam_x_filtered: float
    cam_y_filtered: float
    cam_z_filtered: float
    valid: bool
    stale_count: int
    spatial_depth_mm: Optional[float]
    visibility: float

    @classmethod
    def invalid(cls, name: str, pixel_x: float, pixel_y: float, visibility: float) -> 'Joint3D':
        """Joint with no usable 3D position (low visibility or missing depth)."""
        return cls(name, pixel_x, pixel_y,
                   0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0,
                   valid=False, stale_count=0, spatial_depth_mm=None, visibility=visibility)

    @classmethod
    def detected(cls, name: str, pixel_x: float, pixel_y: float,
                 cam_x: float, cam_y: float, cam_z: float,
                 spatial_depth_mm: Optional[float], visibility: float,
                 previous: Optional['Joint3D'], iir_alpha: float) -> 'Joint3D':
        """Valid joint; filtered position is IIR-smoothed from the previous valid joint if any."""
        if previous is not None and previous.valid:
            cam_x_filtered = iir_filtered(cam_x, previous.cam_x_filtered, iir_alpha)
            cam_y_filtered = iir_filtered(cam_y, previous.cam_y_filtered, iir_alpha)
            cam_z_filtered = iir_filtered(cam_z, previous.cam_z_filtered, iir_alpha)
        else:
            cam_x_filtered = cam_x
            cam_y_filtered = cam_y
            cam_z_filtered = cam_z
        return cls(name, pixel_x, pixel_y,
                   cam_x, cam_y, cam_z,
                   cam_x_filtered, cam_y_filtered, cam_z_filtered,
                   valid=True, stale_count=0,
                   spatial_depth_mm=spatial_depth_mm, visibility=visibility)

    @classmethod
    def carried_over(cls, name: str, pixel_x: float, pixel_y: float,
                     previous: 'Joint3D', visibility: float) -> 'Joint3D':
        """Hold the previous valid position for one more frame (short occlusion)."""
        return cls(name, pixel_x, pixel_y,
                   previous.cam_x, previous.cam_y, previous.cam_z,
                   previous.cam_x_filtered, previous.cam_y_filtered, previous.cam_z_filtered,
                   valid=True, stale_count=previous.stale_count + 1,
                   spatial_depth_mm=None, visibility=visibility)


class PoseDetectorNode(Node):
    """Detect the operator skeleton, map it to 3D, and publish annotated images + skeleton."""

    def __init__(self):
        super().__init__('pose_detector_node')
        self._declare_parameters()
        self._read_parameters()
        self._bridge = CvBridge()
        self._pose = self._build_pose_landmarker()
        self._pose_connections = vision.PoseLandmarksConnections.POSE_LANDMARKS
        self._init_state()
        self._create_interfaces()
        self._setup_tf()
        self.get_logger().info(
            f'Pose detector node started: {self._resolution}, model={self._model_type}')

    def _declare_parameters(self):
        """Declare every ROS parameter (defaults live in config/pose_detector.yaml)."""
        self.declare_parameter('resolution', 'sd')
        self.declare_parameter('model_type', 'full')
        self.declare_parameter('annotated_image_topic', '/pose/annotated_image')
        self.declare_parameter('skeleton_markers_topic', '/pose/skeleton_3d')
        self.declare_parameter('operator_skeleton_topic', '/pose/operator_skeleton')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('camera_optical_frame', 'kinect2_rgb_optical_frame')
        self.declare_parameter('min_pose_detection_confidence', 0.5)
        self.declare_parameter('min_pose_presence_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('min_visibility', 0.1)
        self.declare_parameter('spatial_window_radius', 2)
        self.declare_parameter('temporal_window_size', 5)
        self.declare_parameter('iir_alpha', 0.3)
        self.declare_parameter('max_stale_frames', 10)
        self.declare_parameter('global_outlier_threshold', 0.6)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('qos_depth', 10)

    def _read_parameters(self):
        """Read parameters into members, validating the enum-like ones with a logged fallback."""
        resolution = self.get_parameter('resolution').value
        if resolution not in VALID_RESOLUTIONS:
            self.get_logger().error(
                f'Invalid resolution: {resolution}. '
                f'Valid: {", ".join(VALID_RESOLUTIONS)}. Using sd as fallback.')
            resolution = 'sd'
        self._resolution = resolution

        model_type = self.get_parameter('model_type').value
        if model_type not in MODEL_FILENAMES:
            self.get_logger().error(
                f'Invalid model_type: {model_type}. '
                f'Valid: {", ".join(MODEL_FILENAMES)}. Using full as fallback.')
            model_type = 'full'
        self._model_type = model_type

        self._rgb_topic = f'/kinect2/{resolution}/image_color_rect'
        self._depth_topic = f'/kinect2/{resolution}/image_depth_rect'
        self._camera_info_topic = f'/kinect2/{resolution}/camera_info'
        self._annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self._skeleton_markers_topic = self.get_parameter('skeleton_markers_topic').value
        self._operator_skeleton_topic = self.get_parameter('operator_skeleton_topic').value

        self._target_frame = self.get_parameter('target_frame').value
        self._camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self._min_pose_detection_confidence = \
            self.get_parameter('min_pose_detection_confidence').value
        self._min_pose_presence_confidence = \
            self.get_parameter('min_pose_presence_confidence').value
        self._min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        self._min_visibility = self.get_parameter('min_visibility').value
        self._spatial_window_radius = self.get_parameter('spatial_window_radius').value
        self._temporal_window_size = self.get_parameter('temporal_window_size').value
        self._iir_alpha = self.get_parameter('iir_alpha').value
        self._max_stale_frames = self.get_parameter('max_stale_frames').value
        self._global_outlier_threshold = self.get_parameter('global_outlier_threshold').value
        self._sync_queue_size = self.get_parameter('sync_queue_size').value
        self._sync_slop = self.get_parameter('sync_slop').value
        self._qos_depth = self.get_parameter('qos_depth').value

    def _resolve_model_path(self, model_filename: str) -> str:
        """Return the model file path in the package share; raise with a hint when missing."""
        model_path = os.path.join(
            get_package_share_directory('kinect_pose_detector'), 'models', model_filename)
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f'Model file {model_path} not found. '
                f'Download the models (see README, section Models) and rebuild the package.')
        return model_path

    def _build_pose_landmarker(self) -> vision.PoseLandmarker:
        model_filename = MODEL_FILENAMES[self._model_type]
        model_path = self._resolve_model_path(model_filename)
        self.get_logger().info(f'Loading MediaPipe model: {self._model_type} ({model_filename})')
        base_options = mp.tasks.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=self._min_pose_detection_confidence,
            min_pose_presence_confidence=self._min_pose_presence_confidence,
            min_tracking_confidence=self._min_tracking_confidence,
            output_segmentation_masks=False)
        return vision.PoseLandmarker.create_from_options(options)

    def _init_state(self):
        self._camera_info = None
        self._frames_dropped_no_camera_info = 0
        self._depth_history = {}
        self._previous_joints = None
        self._last_timestamp_ms = -1
        self._frame_count = 0
        self._frame_times = deque(maxlen=FPS_WINDOW)
        self._last_mediapipe_ms = 0.0
        self._last_total_ms = 0.0

    def _create_interfaces(self):
        self._camera_info_subscription = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_callback, self._qos_depth)
        self._rgb_subscriber = Subscriber(self, Image, self._rgb_topic)
        self._depth_subscriber = Subscriber(self, Image, self._depth_topic)
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size=self._sync_queue_size,
            slop=self._sync_slop)
        self._sync.registerCallback(self._synced_callback)
        self._annotated_image_publisher = self.create_publisher(
            Image, self._annotated_image_topic, self._qos_depth)
        self._skeleton_marker_publisher = self.create_publisher(
            MarkerArray, self._skeleton_markers_topic, self._qos_depth)
        self._operator_skeleton_publisher = self.create_publisher(
            PoseArray, self._operator_skeleton_topic, self._qos_depth)

    def _setup_tf(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cached_transform = None

    def _camera_info_callback(self, camera_info_msg: CameraInfo):
        """Store camera intrinsics once (used by the pinhole projection)."""
        if self._camera_info is None:
            self._camera_info = camera_info_msg
            if self._frames_dropped_no_camera_info > 0:
                self.get_logger().info(
                    f'Camera info received. '
                    f'Dropped {self._frames_dropped_no_camera_info} frames while waiting.')
            self.destroy_subscription(self._camera_info_subscription)

    def _pixel_to_3d(self, pixel_x: float, pixel_y: float,
                     depth_mm: Optional[float]) -> Tuple[Optional[float], Optional[float],
                                                         Optional[float]]:
        """Inverse pinhole projection: pixel (x, y) + depth -> 3D camera-frame point (m)."""
        if self._camera_info is None or depth_mm is None or depth_mm <= 0:
            return None, None, None
        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]
        cam_z = depth_mm / DEPTH_MM_PER_M
        cam_x = (pixel_x - cx) * cam_z / fx
        cam_y = (pixel_y - cy) * cam_z / fy
        return cam_x, cam_y, cam_z

    @staticmethod
    def _spatial_median_depth(depth_image: np.ndarray, pixel_x: float, pixel_y: float,
                              window_radius: int) -> Optional[float]:
        """Median of the non-zero depths in the (2r+1) x (2r+1) window around the pixel."""
        image_height, image_width = depth_image.shape[:2]
        center_x = int(round(pixel_x))
        center_y = int(round(pixel_y))
        x_min = max(0, center_x - window_radius)
        x_max = min(image_width, center_x + window_radius + 1)
        y_min = max(0, center_y - window_radius)
        y_max = min(image_height, center_y + window_radius + 1)
        region = depth_image[y_min:y_max, x_min:x_max]
        valid_depths = region[region > 0]
        if valid_depths.size == 0:
            return None
        return float(np.median(valid_depths))

    def _temporal_median_depth(self, joint_name: str,
                               raw_depth_mm: Optional[float]) -> Optional[float]:
        """Median of the last temporal_window_size depths for this joint (kills ToF spikes)."""
        if joint_name not in self._depth_history:
            self._depth_history[joint_name] = deque(maxlen=self._temporal_window_size)
        if raw_depth_mm is not None and raw_depth_mm > 0:
            self._depth_history[joint_name].append(raw_depth_mm)
        history = self._depth_history[joint_name]
        if len(history) == 0:
            return None
        return statistics.median(history)

    def _filter_global_outliers(self, joints: List[Joint3D]) -> List[Joint3D]:
        """Cross-joint validation: drop joints whose depth is far from the skeleton median."""
        valid_joints = [joint for joint in joints if joint.valid and joint.cam_z > 0]
        if len(valid_joints) < MIN_JOINTS_FOR_GLOBAL_FILTER:
            return joints
        median_depth = statistics.median(joint.cam_z for joint in valid_joints)
        for joint in joints:
            if not joint.valid or joint.cam_z <= 0:
                continue
            if abs(joint.cam_z - median_depth) > self._global_outlier_threshold:
                joint.valid = False
        return joints

    def _detect_pose_3d(self, rgb_image: np.ndarray, depth_image: np.ndarray,
                        timestamp_ms: int) -> Optional[List[Joint3D]]:
        """Run MediaPipe and build the filtered 3D joint list for the frame."""
        if timestamp_ms <= self._last_timestamp_ms:
            self.get_logger().warn(
                f'Non-monotonic timestamp ({timestamp_ms}ms <= '
                f'{self._last_timestamp_ms}ms); skipping frame',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return None
        self._last_timestamp_ms = timestamp_ms
        mediapipe_start_time = time.time()
        rgb = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        image_height, image_width = rgb_image.shape[:2]

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = self._pose.detect_for_video(mp_image, timestamp_ms)
        self._last_mediapipe_ms = (time.time() - mediapipe_start_time) * 1000.0

        if not results.pose_landmarks:
            return None

        landmarks = results.pose_landmarks[0]
        joints = [
            self._build_joint_for_landmark(index, landmark, depth_image, image_width, image_height)
            for index, landmark in enumerate(landmarks)
        ]
        self._prune_depth_history(joints)
        return joints

    def _build_joint_for_landmark(self, index: int, landmark, depth_image: np.ndarray,
                                  image_width: int, image_height: int) -> Joint3D:
        """Build one Joint3D from a MediaPipe landmark: visibility -> depth -> 3D -> holdover."""
        pixel_x = float(np.clip(landmark.x, 0.0, 1.0) * image_width)
        pixel_y = float(np.clip(landmark.y, 0.0, 1.0) * image_height)
        visibility = landmark.visibility if hasattr(landmark, 'visibility') else 1.0
        name = KEYPOINT_NAMES[index] if index < len(KEYPOINT_NAMES) else f'joint_{index}'

        if visibility < self._min_visibility:
            self._depth_history.pop(name, None)
            return Joint3D.invalid(name, pixel_x, pixel_y, visibility)

        spatial_depth_mm = self._spatial_median_depth(
            depth_image, pixel_x, pixel_y, self._spatial_window_radius)
        if spatial_depth_mm is not None and spatial_depth_mm > 0:
            depth_mm = self._temporal_median_depth(name, spatial_depth_mm)
        else:
            depth_mm = None

        cam_x, cam_y, cam_z = self._pixel_to_3d(pixel_x, pixel_y, depth_mm)
        previous = self._previous_joint(index)

        if cam_x is None or cam_y is None or cam_z is None or cam_z <= 0:
            if (previous is not None and previous.valid
                    and previous.stale_count < self._max_stale_frames):
                return Joint3D.carried_over(name, pixel_x, pixel_y, previous, visibility)
            return Joint3D.invalid(name, pixel_x, pixel_y, visibility)

        return Joint3D.detected(name, pixel_x, pixel_y, cam_x, cam_y, cam_z,
                                spatial_depth_mm, visibility, previous, self._iir_alpha)

    def _previous_joint(self, index: int) -> Optional[Joint3D]:
        if self._previous_joints is not None and index < len(self._previous_joints):
            return self._previous_joints[index]
        return None

    def _prune_depth_history(self, joints: List[Joint3D]):
        """Periodically drop temporal-history entries for joints no longer present."""
        if self._frame_count % DEPTH_HISTORY_CLEANUP_INTERVAL != 0:
            return
        current_names = {joint.name for joint in joints}
        stale_names = [name for name in self._depth_history if name not in current_names]
        for name in stale_names:
            del self._depth_history[name]
        if stale_names:
            self.get_logger().debug(
                f'Cleaned {len(stale_names)} stale entries from depth_history')

    def _draw_skeleton(self, image: np.ndarray, joints: List[Joint3D],
                       joint_color_bgr, draw_distance_labels: bool) -> np.ndarray:
        """Draw bones and joints onto image; optionally label each joint with its distance."""
        for connection in self._pose_connections:
            start_index, end_index = connection.start, connection.end
            if start_index < len(joints) and end_index < len(joints):
                start_joint = joints[start_index]
                end_joint = joints[end_index]
                if start_joint.valid and end_joint.valid:
                    start_point = (int(start_joint.pixel_x), int(start_joint.pixel_y))
                    end_point = (int(end_joint.pixel_x), int(end_joint.pixel_y))
                    cv2.line(image, start_point, end_point,
                             SKELETON_LINE_COLOR_BGR, SKELETON_LINE_THICKNESS_PX)

        for joint in joints:
            if joint.valid and joint.cam_z > 0:
                center = (int(joint.pixel_x), int(joint.pixel_y))
                cv2.circle(image, center, JOINT_CIRCLE_RADIUS_PX, joint_color_bgr, -1)
                if draw_distance_labels:
                    label = f'{joint.cam_z_filtered:.2f}m'
                    label_origin = (center[0] + DISTANCE_LABEL_OFFSET_PX[0],
                                    center[1] + DISTANCE_LABEL_OFFSET_PX[1])
                    cv2.putText(image, label, label_origin, cv2.FONT_HERSHEY_SIMPLEX,
                                DISTANCE_LABEL_FONT_SCALE, DISTANCE_LABEL_COLOR_BGR,
                                DISTANCE_LABEL_THICKNESS_PX)
        return image

    def _draw_pose(self, rgb_image: np.ndarray, joints: List[Joint3D]) -> np.ndarray:
        """Skeleton on a copy of the RGB image, with per-joint distance labels."""
        return self._draw_skeleton(rgb_image.copy(), joints, RGB_JOINT_COLOR_BGR,
                                   draw_distance_labels=True)

    def _lookup_transform(self):
        """Look up and cache the static transform camera_optical_frame -> target_frame."""
        if self._cached_transform is not None:
            return self._cached_transform
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._camera_optical_frame,
                rclpy.time.Time())
            self._cached_transform = tf_stamped
            self.get_logger().info(
                f'TF cached: {self._target_frame} <- {self._camera_optical_frame}')
            return self._cached_transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            self.get_logger().warn(
                f'TF lookup failed ({self._target_frame} <- {self._camera_optical_frame}): {error}',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return None

    def _lookup_and_frame(self):
        """Return (transform, frame_id): target_frame if TF is available, else the camera frame."""
        tf_stamped = self._lookup_transform()
        frame_id = self._target_frame if tf_stamped is not None else self._camera_optical_frame
        return tf_stamped, frame_id

    def _transform_point(self, cam_x: float, cam_y: float, cam_z: float,
                         tf_stamped) -> Tuple[float, float, float]:
        """Transform a 3D point from the camera optical frame into the target frame."""
        pose = Pose()
        pose.position.x = float(cam_x)
        pose.position.y = float(cam_y)
        pose.position.z = float(cam_z)
        pose.orientation.w = 1.0
        transformed = do_transform_pose(pose, tf_stamped)
        return transformed.position.x, transformed.position.y, transformed.position.z

    def _joint_position_in_output_frame(self, joint: Joint3D,
                                        tf_stamped) -> Tuple[float, float, float]:
        """Filtered joint position in the output frame (transformed if TF is available)."""
        if tf_stamped is not None:
            return self._transform_point(
                joint.cam_x_filtered, joint.cam_y_filtered, joint.cam_z_filtered, tf_stamped)
        return joint.cam_x_filtered, joint.cam_y_filtered, joint.cam_z_filtered

    def _make_joint_marker(self, marker_id: int, joint_x: float, joint_y: float, joint_z: float,
                           frame_id: str, timestamp) -> Marker:
        """Sphere Marker for a single skeleton joint."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = 'joints'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = joint_x
        marker.pose.position.y = joint_y
        marker.pose.position.z = joint_z
        marker.pose.orientation.w = 1.0
        marker.scale.x = MARKER_JOINT_SCALE_M
        marker.scale.y = MARKER_JOINT_SCALE_M
        marker.scale.z = MARKER_JOINT_SCALE_M
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = MARKER_JOINT_COLOR_RGBA
        marker.lifetime.nanosec = MARKER_LIFETIME_NS
        return marker

    def _make_bone_marker(self, marker_id: int, start_point: Tuple[float, float, float],
                          end_point: Tuple[float, float, float], frame_id: str,
                          timestamp) -> Marker:
        """Line-strip Marker for a skeleton bone between two joints."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = 'skeleton'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [
            Point(x=start_point[0], y=start_point[1], z=start_point[2]),
            Point(x=end_point[0], y=end_point[1], z=end_point[2]),
        ]
        marker.scale.x = MARKER_BONE_SCALE_M
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = MARKER_BONE_COLOR_RGBA
        marker.lifetime.nanosec = MARKER_LIFETIME_NS
        return marker

    def _publish_skeleton_markers(self, joints: List[Joint3D], timestamp):
        """Publish the skeleton as a MarkerArray (joints + bones) for RViz."""
        tf_stamped, frame_id = self._lookup_and_frame()
        marker_array = MarkerArray()

        for index, joint in enumerate(joints):
            if not joint.valid or joint.cam_z <= 0:
                continue
            world_x, world_y, world_z = self._joint_position_in_output_frame(joint, tf_stamped)
            marker_array.markers.append(
                self._make_joint_marker(index, world_x, world_y, world_z, frame_id, timestamp))

        bone_marker_id = BONE_MARKER_ID_OFFSET
        for connection in self._pose_connections:
            start_index, end_index = connection.start, connection.end
            if start_index < len(joints) and end_index < len(joints):
                start_joint, end_joint = joints[start_index], joints[end_index]
                if (not start_joint.valid or not end_joint.valid
                        or start_joint.cam_z <= 0 or end_joint.cam_z <= 0):
                    continue
                start_point = self._joint_position_in_output_frame(start_joint, tf_stamped)
                end_point = self._joint_position_in_output_frame(end_joint, tf_stamped)
                marker_array.markers.append(
                    self._make_bone_marker(
                        bone_marker_id, start_point, end_point, frame_id, timestamp))
                bone_marker_id += 1

        self._skeleton_marker_publisher.publish(marker_array)

    def _publish_operator_poses(self, joints: List[Joint3D], timestamp):
        """Publish the skeleton as a PoseArray: exactly len(KEYPOINT_NAMES) entries in order."""
        tf_stamped, frame_id = self._lookup_and_frame()
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = frame_id

        for joint in joints:
            pose = Pose()
            pose.orientation.w = 1.0
            if joint.valid and joint.cam_z > 0:
                world_x, world_y, world_z = self._joint_position_in_output_frame(joint, tf_stamped)
                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = world_z
            pose_array.poses.append(pose)

        self._operator_skeleton_publisher.publish(pose_array)

    def _publish_empty_skeleton(self, timestamp):
        """Publish an all-invalid skeleton (no operator detected this frame)."""
        self._skeleton_marker_publisher.publish(MarkerArray())
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = self._lookup_and_frame()[1]
        for _ in KEYPOINT_NAMES:
            pose = Pose()
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self._operator_skeleton_publisher.publish(pose_array)

    def _synced_callback(self, rgb_msg: Image, depth_msg: Image):
        """Synchronized RGB + depth callback: detect, filter, annotate, publish."""
        if self._camera_info is None:
            self._frames_dropped_no_camera_info += 1
            self.get_logger().warn(
                f'Waiting for camera info... '
                f'({self._frames_dropped_no_camera_info} frames dropped)',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        try:
            frame_start_time = time.time()
            self._frame_count += 1
            self._frame_times.append(frame_start_time)

            rgb_image = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            if depth_image.shape[:2] != rgb_image.shape[:2]:
                self.get_logger().error(
                    f'Depth/RGB size mismatch: depth {depth_image.shape[:2]} vs '
                    f'rgb {rgb_image.shape[:2]}; 3D lift needs registered same-size frames',
                    throttle_duration_sec=LOG_THROTTLE_SEC)
                return

            timestamp_ms = (
                rgb_msg.header.stamp.sec * 1_000_000_000
                + rgb_msg.header.stamp.nanosec) // 1_000_000

            joints = self._detect_pose_3d(rgb_image, depth_image, timestamp_ms)
            if joints is not None:
                joints = self._filter_global_outliers(joints)
                self._previous_joints = joints

            if joints is not None:
                annotated_image = self._draw_pose(rgb_image, joints)
            else:
                annotated_image = rgb_image.copy()

            annotated_image_msg = self._bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_image_msg.header = rgb_msg.header
            self._annotated_image_publisher.publish(annotated_image_msg)

            timestamp = rgb_msg.header.stamp
            if joints is not None:
                self._publish_skeleton_markers(joints, timestamp)
                self._publish_operator_poses(joints, timestamp)
            else:
                self._publish_empty_skeleton(timestamp)

            self._last_total_ms = (time.time() - frame_start_time) * 1000.0
            self._log_performance(joints)
        except Exception as error:
            self.get_logger().error(
                f'Processing error: {error}\n{traceback.format_exc()}',
                throttle_duration_sec=LOG_THROTTLE_SEC)

    def _log_performance(self, joints: Optional[List[Joint3D]]):
        """Throttled FPS / timing / joint-count log."""
        if len(self._frame_times) < FPS_WINDOW or self._frame_count % FPS_REPORT_INTERVAL != 0:
            return
        window_duration = self._frame_times[-1] - self._frame_times[0]
        fps = (FPS_WINDOW - 1) / window_duration if window_duration > 0 else 0.0
        valid_count = 0
        stale_count = 0
        if joints is not None:
            valid_count = sum(1 for joint in joints if joint.valid)
            stale_count = sum(1 for joint in joints if joint.valid and joint.stale_count > 0)
        self.get_logger().info(
            f'Performance: {fps:.1f} FPS | '
            f'MediaPipe: {self._last_mediapipe_ms:.1f}ms | '
            f'Total: {self._last_total_ms:.1f}ms | '
            f'Valid joints: {valid_count}/{len(KEYPOINT_NAMES)} (stale: {stale_count}) | '
            f'Frame #{self._frame_count}')

    def destroy_node(self):
        """Release MediaPipe resources on shutdown."""
        if self._pose:
            self._pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    node = PoseDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
