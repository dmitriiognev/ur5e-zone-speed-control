import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (override the matching entries in config/pose_detector.yaml)
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='sd',
        description='Kinect resolution: sd (512x424), qhd (960x540), hd (1920x1080)'
    )

    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='full',
        description='MediaPipe model: lite (fastest), full (balanced), heavy (most accurate)'
    )

    # Camera extrinsics: position and orientation of kinect2_link in the world frame.
    # kinect2_link = kinect2_rgb_optical_frame (identity in kinect2_bridge).
    # Optical frame convention: X right, Y down, Z forward (into scene).
    #
    # Default rotation RPY(-π/2, 0, -π/2) assumes the camera looks along +X_world:
    #   camera Z (forward) -> +X_world
    #   camera X (right)   -> -Y_world
    #   camera Y (down)    -> -Z_world
    # This makes the skeleton stand upright (Z_world = up).
    #
    # Adjust cam_x/y/z to match your physical camera placement relative to the robot.
    cam_x_arg = DeclareLaunchArgument(
        'cam_x', default_value='0.0',
        description='Camera X position in world frame (meters)')
    cam_y_arg = DeclareLaunchArgument(
        'cam_y', default_value='0.0',
        description='Camera Y position in world frame (meters)')
    cam_z_arg = DeclareLaunchArgument(
        'cam_z', default_value='1.0',
        description='Camera Z position in world frame (meters)')
    cam_roll_arg = DeclareLaunchArgument(
        'cam_roll', default_value='-1.5708',
        description='Camera roll in world frame (radians)')
    cam_pitch_arg = DeclareLaunchArgument(
        'cam_pitch', default_value='0.0',
        description='Camera pitch in world frame (radians)')
    cam_yaw_arg = DeclareLaunchArgument(
        'cam_yaw', default_value='-1.5708',
        description='Camera yaw in world frame (radians)')

    resolution = LaunchConfiguration('resolution')
    model_type = LaunchConfiguration('model_type')

    config_file = os.path.join(
        get_package_share_directory('kinect_pose_detector'),
        'config', 'pose_detector.yaml')

    pose_detector = Node(
        package='kinect_pose_detector',
        executable='pose_detector_node',
        name='pose_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {
                'resolution': resolution,
                'model_type': model_type,
            },
        ]
    )

    # Static TF: world -> kinect2_link (camera extrinsic calibration)
    kinect_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect2_static_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'world',
            '--child-frame-id', 'kinect2_link',
        ]
    )

    return LaunchDescription([
        resolution_arg,
        model_type_arg,
        cam_x_arg,
        cam_y_arg,
        cam_z_arg,
        cam_roll_arg,
        cam_pitch_arg,
        cam_yaw_arg,
        pose_detector,
        kinect_static_tf,
    ])
