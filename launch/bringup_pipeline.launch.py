"""
Bringup for the perception -> zones -> speed/motion pipeline (everything except the robot).

Start the robot side first, in a separate terminal:
    ros2 launch launch/bringup_real.launch.py robot_ip:=192.168.0.5

Then this pipeline (from the workspace root, after source install/setup.bash):
    ros2 launch launch/bringup_pipeline.launch.py

Composition (each stage is the package's own launch file; data flow self-sequences the chain):
    kinect2_bridge          -> /kinect2/sd/{image_color_rect, image_depth_rect, camera_info}
    pose_detector_node      -> /pose/operator_skeleton
    zones_node              -> /operator/zone
    gesture_detector_node   -> /operator/collaborative_mode
    speed_controller_node   -> /motion/paused + /io_and_status_controller/set_speed_slider
    lemniscate_executor_node    -> FollowJointTrajectory (waits for MoveIt + slider service itself)
    operator_gui_node       -> fullscreen operator display

No artificial start delays: lemniscate_executor blocks on the MoveGroupInterface and the slider
service itself, and the speed controller only acts once /operator/zone starts flowing.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KINECT_LAUNCH = os.path.join(_WORKSPACE_ROOT, 'launch', 'kinect2_optimized.launch.yaml')


def _include(package, launch_file):
    """Include a package's own launch file by share path."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        )
    )


def generate_launch_description():
    kinect = IncludeLaunchDescription(AnyLaunchDescriptionSource(_KINECT_LAUNCH))

    pose_detector = _include('kinect_pose_detector', 'pose_detector.launch.py')
    zones = _include('operator_zones', 'zones_node.launch.py')
    gesture = _include('gesture_detector', 'gesture_detector_node.launch.py')
    speed_controller = _include('zone_speed_controller', 'speed_controller.launch.py')
    lemniscate_executor = _include('lemniscate_executor', 'lemniscate_executor.launch.py')
    operator_gui = _include('operator_gui', 'operator_gui.launch.py')

    return LaunchDescription([
        kinect,
        pose_detector,
        zones,
        gesture,
        speed_controller,
        lemniscate_executor,
        operator_gui,
    ])
