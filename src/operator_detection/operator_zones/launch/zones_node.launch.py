import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('operator_zones'),
        'config', 'zones_node.yaml'
    )

    zones_node = Node(
        package='operator_zones',
        executable='zones_node',
        name='operator_zones_node',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([
        zones_node,
    ])
