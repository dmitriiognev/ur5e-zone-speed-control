import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('operator_gui'),
        'config', 'operator_gui.yaml')

    operator_gui = Node(
        package='operator_gui',
        executable='operator_gui_node',
        name='operator_gui_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )

    return LaunchDescription([
        operator_gui,
    ])
