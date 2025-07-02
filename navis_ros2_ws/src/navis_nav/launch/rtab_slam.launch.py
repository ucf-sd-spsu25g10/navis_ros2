import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile

from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PythonExpression, Command
)
from launch_ros.actions import (
    ComposableNodeContainer, LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    stereo_pkg_dir = get_package_share_directory('stereo_cam')
    imu_pkg_dir = get_package_share_directory('mpu9250driver')
    nav_pkg_dir = get_package_share_directory('navis_nav')

    ekf_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf.yaml')
    ekf_params = ParameterFile(ekf_config_path)

    mpu_imu_config_path = os.path.join(imu_pkg_dir, 'params', 'mpu_imu.yaml')
    mpu_imu_params = ParameterFile(mpu_imu_config_path)

    mapping_config_path = os.path.join(nav_pkg_dir, 'config', 'rtabmap_mapping.yaml')
    mapping_params = ParameterFile(mapping_config_path)

    localize_config_path = os.path.join(nav_pkg_dir, 'config', 'rtabmap_localization.yaml')
    localize_params = ParameterFile(mapping_config_path)



    log_level = LaunchConfiguration('log_level')
    enable_mapping = LaunchConfiguration('enable_mapping')


    return LaunchDescription([
        DeclareLaunchArgument('enable_mapping', default_value='false', description='Run in mapping mode, default is localization'),
        DeclareLaunchArgument('log_level', default_value='WARN'),

        # IMU Driver
        Node(
            package='mpu9250driver',
            executable='mpu9250driver',
            name='mpu9250driver_node',
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[mpu_imu_params],
            arguments=['--ros-args', '--log-level', log_level]
        ),

        # Synced Stereo Camera Driver -> Rectifier -> Stereo Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_cam'),
                    'launch',
                    'image_proc_pipeline.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': 'true',
                'enable_disparity': 'false',
                'log_level': log_level
            }.items()
        ),

        # Sensor Fusion via robot_localization Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            arguments=['--ros-args', '--log-level', log_level]
        ),

        # SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[localize_params],
            remappings=[
                ('/left/camera_info', '/left/camera_info_rect'),
                ('/right/camera_info', '/right/camera_info_rect'),
                ('odom', '/odom'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
            condition=UnlessCondition(enable_mapping),
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[mapping_params],
            remappings=[
                ('/left/camera_info', '/left/camera_info_rect'),
                ('/right/camera_info', '/right/camera_info_rect'),
                ('odom', '/odom'),
            ],
            arguments=['--ros-args', '--log-level', log_level],
            condition=IfCondition(enable_mapping),
        ),
    ])
