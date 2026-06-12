"""
Bringup for the real UR5e + cocohrip cell.

Replaces the direct include of cocohrip_control/bringup_all.launch.py so that
RViz can be disabled via launch_rviz:=false (default).  Running RViz during
normal operation costs ~30% CPU and causes controller overruns on the laptop.

Usage (from workspace root, after source install/setup.bash):
    ros2 launch launch/bringup_real.launch.py robot_ip:=192.168.0.5
    ros2 launch launch/bringup_real.launch.py robot_ip:=192.168.0.5 launch_rviz:=true
    ros2 launch launch/bringup_real.launch.py robot_ip:=192.168.0.5 use_mock_hardware:=true

Troubleshooting — if robot does not connect after (re)starting bringup:
    On the teach pendant: press STOP, then PLAY.
    This forces the ExternalControl URCap to reconnect to the new driver instance.

Troubleshooting — [RTPS_TRANSPORT_SHM Error] Failed init_port on startup:
    Stale /dev/shm/fastrtps_* files left by a crashed session; remove them:
    rm /dev/shm/fastrtps_*
"""

import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _robot_iface_ip() -> str:
    """Return PC's IP address on the 192.168.0.x robot network, or a warning."""
    try:
        result = subprocess.run(
            ['ip', '-4', 'addr', 'show'],
            capture_output=True, text=True, timeout=2
        )
        for line in result.stdout.splitlines():
            if '192.168.0.' in line:
                return line.strip().split()[1].split('/')[0]
    except Exception:
        pass
    return 'NOT FOUND — check network cable or run: ip addr show'


def _print_connection_info(context):
    pc_ip = _robot_iface_ip()
    lines = [
        '=' * 56,
        f'PC IP for URCap ExternalControl: {pc_ip}',
        'If robot does not connect after restart:',
        '  -> Press STOP then PLAY on the teach pendant',
        '=' * 56,
    ]
    return [LogInfo(msg=line) for line in lines]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.0.5',
            description='IP address of the UR5e robot.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware instead of real robot.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description=(
                'Launch RViz for MoveIt visualisation '
                '(costs ~30% CPU — disable during normal operation).'),
        ),
    ]

    # ── Robot control (UR driver + ros2_control + spawners) ───────────────────
    start_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cocohrip_control'),
                'launch',
                'start_robot_control.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
        }.items(),
    )

    # ── MoveIt move_group ─────────────────────────────────────────────────────
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cocohrip_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        )
    )

    # ── RViz (optional — disabled by default to save CPU) ────────────────────
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cocohrip_moveit_config'),
                'launch',
                'moveit_rviz.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # ── Gripper ───────────────────────────────────────────────────────────────
    gripper_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robotiq_hande_ros2_driver'),
                'launch',
                'gripper_bringup.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=_print_connection_info),
            start_robot_control,
            move_group,
            moveit_rviz,
            gripper_control,
        ]
    )
