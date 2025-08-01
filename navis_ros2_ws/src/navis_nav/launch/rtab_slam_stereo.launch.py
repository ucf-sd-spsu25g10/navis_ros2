import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    stereo_pkg_dir = get_package_share_directory('stereo_cam')
    ekf_mpu_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf_mpu.yaml')
    ekf_mpu_params = ParameterFile(ekf_mpu_config_path)
    ekf_bno_config_path = os.path.join(stereo_pkg_dir, 'config', 'odom', 'ekf_bno.yaml')
    ekf_bno_params = ParameterFile(ekf_bno_config_path)

    bno_pkg_dir = get_package_share_directory('bno08x_driver')
    bno_imu_config_path = os.path.join(bno_pkg_dir, 'config', 'bno085_i2c.yaml')
    bno_imu_params = ParameterFile(bno_imu_config_path)

    mpu_pkg_dir = get_package_share_directory('mpu9250driver')
    mpu_imu_config_path = os.path.join(mpu_pkg_dir, 'params', 'mpu_imu.yaml')
    mpu_imu_params  = ParameterFile(mpu_imu_config_path)

    rtab_params = {
        'approx_sync': True,
        'approx_sync_max_interval': 0.01,
        'use_sim_time': False,

        'wait_imu_to_init': False,

        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_rgb': False,
        'subscribe_stereo': True,
        'subscribe_scan': False,

        'Reg/Force3DoF': "True",
        'Grid/3D': "False",
        'RGBD/CreateOccupancyGrid': "True",
        'Vis/CorType': "1", #better
        
        # Dont know if these actually do anything, but they were on performance recommendations in this tutorial: 
        # https://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning
        'cloud_noise_filtering_radius': 0.05,
        'cloud_noise_filtering_min_neighbors': 2, # Is this relevant for 2D?
        'Optimizer/Slam2D': "True", # Can't find this in below param list, but its in documentation as a recommended setting
        
        "Grid/RayTracing": "True",
        "Grid/NormalsSegmentation": "False",
        "Grid/NoiseFilteringRadius": "0.0",
        "Grid/RangeMax": "4.0",
        "Grid/MinClusterSize": "20"
    }

    use_bno = LaunchConfiguration('use_bno')

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='false', description='Run in localization mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='WARN'),
        DeclareLaunchArgument('use_raspi', default_value='true'),
        DeclareLaunchArgument('use_bno', default_value='true'),

        # IMU driver 
        Node(
            package='mpu9250driver',
            executable='mpu9250driver',
            name='mpu9250driver_node',
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[mpu_imu_params],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=UnlessCondition(use_bno)
        ),

        Node(
            package='bno08x_driver',  
            executable='bno08x_driver',  
            name='bno08x_driver',
            output='screen',
            parameters=[bno_imu_params],
            condition=IfCondition(use_bno)
        ),

        # Stereo camera pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(stereo_pkg_dir, 'launch', 'image_proc_pipeline.launch.py')
            ),
            launch_arguments={
                'use_raspi': LaunchConfiguration('use_raspi'),
                'enable_disparity': 'true',
                'log_level': LaunchConfiguration('log_level')
            }.items()
        ),

        # EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_bno_params],
            condition=IfCondition(use_bno)
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_mpu_params],
            condition=UnlessCondition(use_bno)
        ),

        # RTAB-Map node 
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                        'Mem/IncrementalMemory': "false",
                        'Mem/InitWMWithAllNodes': "true",
                        "database_path": '/home/pi/.ros/rtabmap_3_aisle_creol_full.db' # if you see something about missing words in a dictionary, 
                                                                               # run 'rtabmap-recovery ~/.ros/rtabmap.db' to recover corrupted db
                        },
                        rtab_params],
            remappings=[
                ('odom', '/odometry/filtered'),
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=IfCondition(LaunchConfiguration('localization'))
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                        'Mem/IncrementalMemory':  "true",
                        'Mem/InitWMWithAllNodes': "False",
                        },
                        rtab_params],
            remappings=[
                ('odom', '/odometry/filtered'),
            ],
            arguments=['-d', '--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=UnlessCondition(LaunchConfiguration('localization'))
        )
    ])
