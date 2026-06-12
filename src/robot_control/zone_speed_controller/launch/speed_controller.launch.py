import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('zone_speed_controller'),
        'config', 'speed_controller.yaml'
    )

    speed_controller_node = Node(
        package='zone_speed_controller',
        executable='speed_controller_node',
        name='speed_controller',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([
        speed_controller_node,
    ])