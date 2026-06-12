import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (override the matching entries in config/gesture_detector_node.yaml)
    hold_frames_arg = DeclareLaunchArgument(
        'hold_frames',
        default_value='20',
        description='Consecutive frames the gesture must hold before triggering (~0.67 s)'
    )

    raise_margin_arg = DeclareLaunchArgument(
        'raise_margin',
        default_value='0.05',
        description='Metres the wrists must clear the nose to count as raised (hysteresis)'
    )

    config_file = os.path.join(
        get_package_share_directory('gesture_detector'),
        'config', 'gesture_detector_node.yaml')

    gesture_detector = Node(
        package='gesture_detector',
        executable='gesture_detector_node',
        name='gesture_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'hold_frames': LaunchConfiguration('hold_frames'),
                'raise_margin': LaunchConfiguration('raise_margin'),
            },
        ]
    )

    return LaunchDescription([
        hold_frames_arg,
        raise_margin_arg,
        gesture_detector,
    ])
