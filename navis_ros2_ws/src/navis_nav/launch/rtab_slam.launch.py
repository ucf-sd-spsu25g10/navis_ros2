import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    stereo_pkg_dir = get_package_share_directory('stereo_cam')
    imu_pkg_dir = get_package_share_directory('mpu9250driver')
    nav_pkg_dir = get_package_share_directory('navis_nav')

    ekf_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf.yaml')
    mpu_imu_config_path = os.path.join(imu_pkg_dir, 'params', 'mpu_imu.yaml')
    rtab_slam_config_path = os.path.join(nav_pkg_dir, 'config', 'rtabmap_slam.yaml')

    ekf_params  = ParameterFile(ekf_config_path)
    imu_params  = ParameterFile(mpu_imu_config_path)
    rtab_params = ParameterFile(rtab_slam_config_path)

    localization = LaunchConfiguration('localization') == "true"
    slam_mode_params = {
        'Mem/IncrementalMemory':  "false" if localization else "true",
        'Mem/InitWMWithAllNodes': "true" if localization else "false"
    }

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='false', description='Run in localization mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='WARN'),
        DeclareLaunchArgument('use_raspi', default_value='false'),

        # IMU driver 
        Node(
            package='mpu9250driver',
            executable='mpu9250driver',
            name='mpu9250driver_node',
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[imu_params],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Stereo camera pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(stereo_pkg_dir, 'launch', 'image_proc_pipeline.launch.py')
            ),
            launch_arguments={
                'use_raspi': LaunchConfiguration('use_raspi'),
                'enable_disparity': 'false',
                'log_level': LaunchConfiguration('log_level')
            }.items()
        ),

        # EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # RTAB-Map node 
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[slam_mode_params,
                        rtab_params],
            remappings=[
                ('/left/camera_info', '/left/camera_info_rect'),
                ('/right/camera_info', '/right/camera_info_rect'),
                ('odom', '/odom'),
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
