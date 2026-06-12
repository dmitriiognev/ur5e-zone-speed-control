import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('flag_grasper'),
        'config', 'flag_grasper.yaml'
    )

    flag_grasper_node = Node(
        package='flag_grasper',
        executable='flag_grasper_node',
        name='flag_grasper_node',
        parameters=[config],
        output='screen',
    )
    return LaunchDescription([flag_grasper_node])