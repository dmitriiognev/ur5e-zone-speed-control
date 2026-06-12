#!/usr/bin/env python3
"""ROS2 node: collaborative-mode toggle from a held 'both wrists above nose' gesture."""
from enum import auto
from enum import Enum
import signal
import sys
from typing import Tuple

from geometry_msgs.msg import PoseArray
from operator_detection_common.keypoints import is_invalid_joint
from operator_detection_common.keypoints import KEYPOINT_NAMES
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

NOSE_INDEX = KEYPOINT_NAMES.index('NOSE')
LEFT_WRIST_INDEX = KEYPOINT_NAMES.index('LEFT_WRIST')
RIGHT_WRIST_INDEX = KEYPOINT_NAMES.index('RIGHT_WRIST')

EXPECTED_SKELETON_LENGTH = len(KEYPOINT_NAMES)

INITIAL_COLLABORATIVE_MODE = True

DEFAULT_RAISE_MARGIN_M = 0.05

LATCHED_QOS_DEPTH = 1


# ==================================================================================================
# Logic layer (pure Python, no ROS dependencies; unit-testable without ROS)
# ==================================================================================================
class GestureState(Enum):
    """Explicit states of the hold-and-toggle gesture FSM (one step per frame)."""

    IDLE = auto()              # No gesture in progress.
    HOLDING = auto()           # Gesture active; counting consecutive frames toward a trigger.
    AWAITING_RELEASE = auto()  # Post-trigger; block re-trigger until the arms drop once.


def wrists_raised(nose_z: float, left_wrist_z: float, right_wrist_z: float, margin: float) -> bool:
    """Return True when both wrists clear the nose by `margin` (Z up): the raise edge."""
    return left_wrist_z > nose_z + margin and right_wrist_z > nose_z + margin


def wrists_lowered(nose_z: float, left_wrist_z: float, right_wrist_z: float) -> bool:
    """Return True when a wrist has dropped back to nose level or below: the release edge."""
    return left_wrist_z <= nose_z or right_wrist_z <= nose_z


class GestureFsm:
    """Hold-and-toggle gesture state machine. Pure logic: hysteresis edges in, toggle out."""

    def __init__(self, hold_frames: int):
        self.hold_frames = hold_frames
        self.state = GestureState.IDLE
        self.hold_count = 0
        self.collaborative_mode = INITIAL_COLLABORATIVE_MODE

    def step(self, raised: bool, lowered: bool) -> bool:
        """Advance the FSM one frame; return True iff the mode toggled this frame."""
        if self.state is GestureState.AWAITING_RELEASE:
            self._await_release(lowered)
            return False
        return self._track_hold(raised)

    def _track_hold(self, raised: bool) -> bool:
        """IDLE/HOLDING: accumulate consecutive raised frames; trigger at hold_frames."""
        if not raised:
            self.hold_count = 0
            self.state = GestureState.IDLE
            return False
        self.hold_count += 1
        self.state = GestureState.HOLDING
        if self.hold_count >= self.hold_frames:
            self._trigger()
            return True
        return False

    def _await_release(self, lowered: bool) -> None:
        """AWAITING_RELEASE: block a new trigger until the wrists drop to the nose once."""
        if lowered:
            self.state = GestureState.IDLE

    def _trigger(self) -> None:
        """Toggle the mode and wait for the arms to drop before re-arming."""
        self.collaborative_mode = not self.collaborative_mode
        self.hold_count = 0
        self.state = GestureState.AWAITING_RELEASE


# ==================================================================================================
# ROS layer (thin wrapper around the FSM, marshaling messages; ROS dependencies only here)
# ==================================================================================================
class GestureDetectorNode(Node):
    """Translate the operator skeleton into collaborative-mode toggles via a pure GestureFsm."""

    def __init__(self):
        super().__init__('gesture_detector_node')
        self._declare_parameters()
        self._read_parameters()
        self._fsm = GestureFsm(self._hold_frames)
        self._create_interfaces()
        self._publish_mode()
        self.get_logger().info(
            f'Gesture detector node started: '
            f'hold={self._hold_frames} frames, raise_margin={self._raise_margin} m')

    def _declare_parameters(self):
        """Declare every ROS parameter (defaults live in config/gesture_detector_node.yaml)."""
        self.declare_parameter('hold_frames', 20)
        self.declare_parameter('raise_margin', DEFAULT_RAISE_MARGIN_M)
        self.declare_parameter('operator_skeleton_topic', '/pose/operator_skeleton')
        self.declare_parameter('collaborative_mode_topic', '/operator/collaborative_mode')
        self.declare_parameter('qos_depth', 10)

    def _read_parameters(self):
        self._hold_frames = self.get_parameter('hold_frames').value
        self._raise_margin = self.get_parameter('raise_margin').value
        self._operator_skeleton_topic = self.get_parameter('operator_skeleton_topic').value
        self._collaborative_mode_topic = self.get_parameter('collaborative_mode_topic').value
        self._qos_depth = self.get_parameter('qos_depth').value

    def _create_interfaces(self):
        latched_qos = QoSProfile(
            depth=LATCHED_QOS_DEPTH,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._collaborative_mode_publisher = self.create_publisher(
            Bool, self._collaborative_mode_topic, latched_qos)
        self._skeleton_subscription = self.create_subscription(
            PoseArray, self._operator_skeleton_topic, self._skeleton_callback, self._qos_depth)

    def _skeleton_callback(self, msg: PoseArray) -> None:
        """Step the FSM from the latest skeleton frame; publish only when the mode toggles."""
        if len(msg.poses) < EXPECTED_SKELETON_LENGTH:
            return
        raised, lowered = self._wrist_edges(msg)
        if self._fsm.step(raised, lowered):
            self._publish_mode()
            self.get_logger().info(
                f'Gesture triggered -> collaborative_mode={self._fsm.collaborative_mode}')

    def _wrist_edges(self, msg: PoseArray) -> Tuple[bool, bool]:
        """Unpack the landmarks into the (raised, lowered) edges; (False, False) if any invalid."""
        nose = msg.poses[NOSE_INDEX].position
        left_wrist = msg.poses[LEFT_WRIST_INDEX].position
        right_wrist = msg.poses[RIGHT_WRIST_INDEX].position
        if (is_invalid_joint(nose) or is_invalid_joint(left_wrist)
                or is_invalid_joint(right_wrist)):
            return False, False
        raised = wrists_raised(nose.z, left_wrist.z, right_wrist.z, self._raise_margin)
        lowered = wrists_lowered(nose.z, left_wrist.z, right_wrist.z)
        return raised, lowered

    def _publish_mode(self) -> None:
        """Publish the current collaborative-mode flag on the latched topic."""
        message = Bool()
        message.data = self._fsm.collaborative_mode
        self._collaborative_mode_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    node = GestureDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
