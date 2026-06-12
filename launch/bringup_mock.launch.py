"""Mock-hardware bringup: UR driver (mock) + move_group + RViz."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.0.5',
            'use_mock_hardware': 'true',
            'mock_sensor_commands': 'true',
            'tf_prefix': 'ur5e_',
            'description_launchfile': PathJoinSubstitution([
                FindPackageShare('cocohrip_control'),
                'launch',
                'rsp.launch.py',
            ]),
            'controller_spawner_timeout': '30',
            'launch_rviz': 'false',
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cocohrip_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        )
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cocohrip_moveit_config'),
                'launch',
                'moveit_rviz.launch.py',
            ])
        )
    )

    return LaunchDescription([
        robot_control,
        move_group,
        moveit_rviz,
    ])
