#!/usr/bin/env python3
"""Compute the operator's proximity zone from the skeleton and publish it on /operator/zone."""

import math
import signal
import statistics
import sys
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from operator_detection_common.keypoints import is_invalid_joint
from operator_detection_common.keypoints import KEYPOINT_NAMES
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Int32
import tf2_ros
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

_NAME_TO_INDEX = {name: index for index, name in enumerate(KEYPOINT_NAMES)}

DEFAULT_TRACKED_JOINTS = [
    'LEFT_WRIST', 'RIGHT_WRIST',
    'LEFT_SHOULDER', 'RIGHT_SHOULDER',
    'LEFT_HIP', 'RIGHT_HIP',
    'NOSE',
]

MARKER_PUBLISH_PERIOD_SEC = 2.0
MARKER_ALPHA = 0.5
MARKER_CIRCLE_SEGMENTS = 64


# ==================================================================================================
# Logic layer (pure Python, no ROS dependencies; unit-testable without ROS)
# ==================================================================================================
class ZoneModel:
    """Radial zone geometry: distance -> zone index and zone -> colour, with no ROS dependency."""

    def __init__(self, danger_radius: float, workspace_radius: float, num_zones: int):
        if num_zones < 2:
            raise ValueError('num_zones must be >= 2')
        if danger_radius >= workspace_radius:
            raise ValueError(
                f'danger_radius ({danger_radius}) must be less than '
                f'workspace_radius ({workspace_radius})')

        self.danger_radius = danger_radius
        self.workspace_radius = workspace_radius
        self.num_zones = num_zones
        self.boundaries = self._compute_boundaries()

    def _compute_boundaries(self) -> List[float]:
        """Zone 0 ends at danger_radius; the rest are evenly spaced out to workspace_radius."""
        if self.num_zones == 2:
            return [self.danger_radius]
        span = self.workspace_radius - self.danger_radius
        inner = [
            self.danger_radius + span * index / (self.num_zones - 1)
            for index in range(1, self.num_zones - 1)
        ]
        return [self.danger_radius] + inner

    def zone_for_distance(self, distance: float) -> int:
        """Return the zone index (0 = closest to the robot base) for a distance."""
        for index, boundary in enumerate(self.boundaries):
            if distance < boundary:
                return index
        return self.num_zones - 1

    def color_for_zone(self, zone_index: int) -> Tuple[float, float, float]:
        """Red (zone 0 / danger) to green (zone N-1 / safe) gradient as an (r, g, b) triple."""
        fraction = zone_index / max(self.num_zones - 1, 1)
        red = max(0.0, min(1.0, 2.0 * (1.0 - fraction)))
        green = max(0.0, min(1.0, 2.0 * fraction))
        return red, green, 0.0


def distances_to_base(
        points_xy: List[Tuple[float, float]],
        base_x: float,
        base_y: float) -> List[float]:
    """Euclidean XY distances from each point to the robot base."""
    return [math.hypot(x - base_x, y - base_y) for x, y in points_xy]


def operator_distance_xy(
        key_points_xy: List[Tuple[float, float]],
        all_points_xy: List[Tuple[float, float]],
        base_x: float,
        base_y: float) -> Optional[float]:
    """Operator distance: min over the key joints, else the median over all valid joints."""
    if key_points_xy:
        return min(distances_to_base(key_points_xy, base_x, base_y))
    all_distances = distances_to_base(all_points_xy, base_x, base_y)
    return statistics.median(all_distances) if all_distances else None


# ==================================================================================================
# ROS layer (skeleton -> distance -> zone; subscriptions, TF, publishers, markers, logging)
# ==================================================================================================
class OperatorZonesNode(Node):
    """Map the operator skeleton to a proximity zone; publish the zone and RViz zone markers."""

    def __init__(self):
        super().__init__('operator_zones_node')
        self._declare_parameters()
        self._read_parameters()

        self._zones = ZoneModel(self._danger_radius, self._workspace_radius, self._num_zones)
        self._tracked_indices = self._resolve_tracked_indices()
        self._current_zone: Optional[int] = None

        self._setup_tf()
        self._create_interfaces()
        self._log_startup()

    def _declare_parameters(self):
        """Declare every ROS parameter (defaults live in config/zones_node.yaml)."""
        self.declare_parameter('robot_base_frame', '')
        self.declare_parameter('robot_base_x', 0.0)
        self.declare_parameter('robot_base_y', 0.0)
        self.declare_parameter('robot_base_z', 0.0)
        self.declare_parameter('danger_radius', 0.5)
        self.declare_parameter('workspace_radius', 2.6)
        self.declare_parameter('num_zones', 5)
        self.declare_parameter('tracked_joints', DEFAULT_TRACKED_JOINTS)
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('operator_skeleton_topic', '/pose/operator_skeleton')
        self.declare_parameter('zone_topic', '/operator/zone')
        self.declare_parameter('zone_markers_topic', '/operator/zone_markers')
        self.declare_parameter('qos_depth', 10)

    def _read_parameters(self):
        self._robot_base_frame = self.get_parameter('robot_base_frame').value.strip()
        self._robot_base = (
            self.get_parameter('robot_base_x').value,
            self.get_parameter('robot_base_y').value,
            self.get_parameter('robot_base_z').value,
        )
        self._danger_radius = self.get_parameter('danger_radius').value
        self._workspace_radius = self.get_parameter('workspace_radius').value
        self._num_zones = self.get_parameter('num_zones').value
        self._tracked_joints = list(self.get_parameter('tracked_joints').value)
        self._reference_frame = self.get_parameter('reference_frame').value
        self._operator_skeleton_topic = self.get_parameter('operator_skeleton_topic').value
        self._zone_topic = self.get_parameter('zone_topic').value
        self._zone_markers_topic = self.get_parameter('zone_markers_topic').value
        self._qos_depth = self.get_parameter('qos_depth').value

    def _resolve_tracked_indices(self) -> List[int]:
        """Map the configured tracked joint names to KEYPOINT_NAMES indices."""
        indices = []
        for name in self._tracked_joints:
            if name in _NAME_TO_INDEX:
                indices.append(_NAME_TO_INDEX[name])
            else:
                self.get_logger().warn(f'Unknown joint name in tracked_joints: {name}')
        return indices

    def _setup_tf(self):
        """Create a TF listener only when a robot_base_frame is configured."""
        if self._robot_base_frame:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _create_interfaces(self):
        self.create_subscription(
            PoseArray, self._operator_skeleton_topic, self._skeleton_callback, self._qos_depth)
        self._zone_publisher = self.create_publisher(Int32, self._zone_topic, self._qos_depth)
        self._zone_markers_publisher = self.create_publisher(
            MarkerArray, self._zone_markers_topic, self._qos_depth)
        self.create_timer(MARKER_PUBLISH_PERIOD_SEC, self._publish_zone_markers)

    def _log_startup(self):
        base_source = (
            f'TF frame "{self._robot_base_frame}"'
            if self._robot_base_frame
            else f'static {self._robot_base}'
        )
        self.get_logger().info(
            f'Zones node started. {self._zones.num_zones} zones, '
            f'boundaries: {[f"{boundary:.3f}" for boundary in self._zones.boundaries]} m '
            f'(danger_radius={self._zones.danger_radius} m, '
            f'workspace_radius={self._zones.workspace_radius} m), '
            f'robot base: {base_source}, '
            f'tracked joints: {self._tracked_joints}'
        )

    def _update_robot_base_from_tf(self) -> bool:
        """Look up reference_frame -> robot_base_frame and update _robot_base; True on success."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._reference_frame, self._robot_base_frame, rclpy.time.Time())
            translation = transform.transform.translation
            self._robot_base = (translation.x, translation.y, translation.z)
            return True
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False

    @staticmethod
    def _valid_points_xy(msg: PoseArray, indices: Iterable[int]) -> List[Tuple[float, float]]:
        """Collect the (x, y) of every valid joint at the given indices."""
        points = []
        for index in indices:
            position = msg.poses[index].position
            if not is_invalid_joint(position):
                points.append((position.x, position.y))
        return points

    def _operator_distance(self, msg: PoseArray) -> Optional[float]:
        """Return the operator's 2D distance to the robot base, or None if undetected."""
        if len(msg.poses) < len(KEYPOINT_NAMES):
            return None
        base_x, base_y, _ = self._robot_base
        key_points = self._valid_points_xy(msg, self._tracked_indices)
        all_points = self._valid_points_xy(msg, range(len(msg.poses)))
        return operator_distance_xy(key_points, all_points, base_x, base_y)

    def _make_zone_markers(self) -> MarkerArray:
        """Build one flat filled annular Marker per zone for RViz (zone 0 is a disk)."""
        marker_array = MarkerArray()
        base_x, base_y, base_z = self._robot_base
        now = self.get_clock().now().to_msg()

        # Outer radius per zone: its boundary for zones 0..N-2, workspace_radius for zone N-1.
        outer_radii = list(self._zones.boundaries) + [self._zones.workspace_radius]

        for zone_index in range(self._zones.num_zones):
            inner_radius = outer_radii[zone_index - 1] if zone_index > 0 else 0.0
            outer_radius = outer_radii[zone_index]
            red, green, blue = self._zones.color_for_zone(zone_index)

            marker = Marker()
            marker.header.frame_id = self._reference_frame
            marker.header.stamp = now
            marker.ns = 'zone_rings'
            marker.id = zone_index
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.position.x = base_x
            marker.pose.position.y = base_y
            marker.pose.position.z = base_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = MARKER_ALPHA
            marker.points = self._annulus_points(inner_radius, outer_radius)
            marker.lifetime = Duration(sec=0, nanosec=0)
            marker_array.markers.append(marker)

        return marker_array

    def _annulus_points(self, inner_radius: float, outer_radius: float) -> List[Point]:
        """Triangle-list vertices for a flat annulus (a disk when inner_radius is 0)."""
        points = []
        for segment in range(MARKER_CIRCLE_SEGMENTS):
            angle = 2.0 * math.pi * segment / MARKER_CIRCLE_SEGMENTS
            next_angle = 2.0 * math.pi * (segment + 1) / MARKER_CIRCLE_SEGMENTS
            points.append(self._circle_point(inner_radius, angle))
            points.append(self._circle_point(outer_radius, angle))
            points.append(self._circle_point(outer_radius, next_angle))
            points.append(self._circle_point(inner_radius, angle))
            points.append(self._circle_point(outer_radius, next_angle))
            points.append(self._circle_point(inner_radius, next_angle))
        return points

    @staticmethod
    def _circle_point(radius: float, angle: float) -> Point:
        """A Point at (radius, angle) in the XY plane, z = 0."""
        point = Point()
        point.x = radius * math.cos(angle)
        point.y = radius * math.sin(angle)
        point.z = 0.0
        return point

    def _publish_zone_markers(self):
        self._zone_markers_publisher.publish(self._make_zone_markers())

    def _skeleton_callback(self, msg: PoseArray):
        if self._robot_base_frame:
            if not self._update_robot_base_from_tf():
                self.get_logger().warn(
                    f'TF lookup failed for "{self._robot_base_frame}" in '
                    f'"{self._reference_frame}" frame, '
                    f'using last known base position {self._robot_base}',
                    throttle_duration_sec=5.0,
                )

        min_distance = self._operator_distance(msg)
        if min_distance is None:
            self.get_logger().info(
                'Operator not detected in workspace',
                throttle_duration_sec=2.0,
            )
            return

        zone = self._zones.zone_for_distance(min_distance)

        zone_msg = Int32()
        zone_msg.data = zone
        self._zone_publisher.publish(zone_msg)

        if zone != self._current_zone:
            self._current_zone = zone
            self.get_logger().info(
                f'[ZONE CHANGE] -> Zone {zone}/{self._zones.num_zones - 1} '
                f'(dist={min_distance:.2f} m)'
            )
        else:
            self.get_logger().info(
                f'Zone {zone}/{self._zones.num_zones - 1} | dist={min_distance:.2f} m',
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    node = OperatorZonesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
