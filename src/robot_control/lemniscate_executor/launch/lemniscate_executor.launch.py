from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('lemniscate_executor'), 'config', 'lemniscate_executor.yaml']
    )

    # URDF, SRDF and kinematics from the COCOHRIP MoveIt package — the node
    # builds its own RobotModel for IK and joint limits.
    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='cocohrip_moveit_config')
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    lemniscate_executor_node = Node(
        package='lemniscate_executor',
        executable='lemniscate_executor_node',
        name='lemniscate_executor',
        parameters=[
            LaunchConfiguration('config'),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='Path to the parameter YAML file',
        ),
        lemniscate_executor_node,
    ])