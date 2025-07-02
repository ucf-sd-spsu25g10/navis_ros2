import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def get_rtabmap_params(context):
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    db_path = LaunchConfiguration('database_path').perform(context)

    params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': False,
        'subscribe_stereo': True,
        'subscribe_scan': False,
        'Mem/IncrementalMemory': 'False' if localization else 'True',
        'Mem/InitWMWithAllNodes': 'True' if localization else 'False',
        'Grid/FromDepth': "True",
        'Grid/3D': "False",
        'RGBD/CreateOccupancyGrid': "True",
        'Rtabmap/DatabasePath': db_path,
        'Reg/Force3DoF': "True",
        'RGBD/OptimizeMaxError': "3.0",
        'Optimizer/Slam2D': "True",
        'cloud_noise_filtering_radius': "0.05",
        'cloud_noise_filtering_min_neighbors': "2",
        'Rtabmap/MaxRetrieved': "1",
        'Rtabmap/MaxLoopClosureDistance': "10.0",
        'Rtabmap/LoopThr': "0.11",
        'approx_sync': True,
        'approx_sync_max_interval': 0.01,
    }
    return params

def add_rtabmap_node(context, *args, **kwargs):
    params = get_rtabmap_params(context)

    return [
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[params],
            remappings=[
                ('/left/camera_info', '/left/camera_info_rect'),
                ('/right/camera_info', '/right/camera_info_rect'),
                ('odom', '/odom'),
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ]

def generate_launch_description():
    stereo_pkg_dir = get_package_share_directory('stereo_cam')
    imu_pkg_dir = get_package_share_directory('mpu9250driver')
    nav_pkg_dir = get_package_share_directory('navis_nav')

    ekf_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf.yaml')
    mpu_imu_config_path = os.path.join(imu_pkg_dir, 'params', 'mpu_imu.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='false', description='Run in localization mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/rtab_maps/map.db'),
        DeclareLaunchArgument('log_level', default_value='WARN'),

        # IMU driver node
        # Node(
        #     package='mpu9250driver',
        #     executable='mpu9250driver',
        #     name='mpu9250driver_node',
        #     output='screen',
        #     respawn=True,
        #     respawn_delay=4,
        #     emulate_tty=True,
        #     parameters=[mpu_imu_config_path],
        #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        # ),

        # Stereo camera pipeline launch include
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(stereo_pkg_dir, 'launch', 'image_proc_pipeline.launch.py')
            ),
            launch_arguments={
                'use_raspi': 'false',
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
            parameters=[ekf_config_path],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # RTAB-Map node via OpaqueFunction
        OpaqueFunction(function=add_rtabmap_node),
    ])
