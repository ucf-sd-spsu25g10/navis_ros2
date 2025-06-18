from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        os.path.dirname(__file__), '../config/ekf.yaml'
    )

    return LaunchDescription([

        # Drivers
        Node(
            package='mpu9250driver',
            executable='mpu9250driver',
            name='mpu9250driver_node',
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[
                {'calibrate': True },
                {'gyro_range': 0 },     # Gyroscope range: 0 -> +-250°/s, 1 -> +-500°/s, 2 -> +-1000°/s, 3 -> +-2000°/s
                {'accel_range': 0 },    # Acceleration range: 0 -> +-2g, 1 -> +-4g, 2 -> +-8g, 3 -> +-16g
                {'dlpf_bandwidth': 2 },   # Digital low pass filter bandwidth [0-6]: 0 -> 260Hz, 1 -> 184Hz, 2 -> 94Hz, 3 -> 44Hz, 4 -> 21Hz, 5 -> 10Hz, 6 -> 5Hz
                {'frequency': 100 }
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_cam'),
                    'launch',
                    'stereo_cam.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': 'true',
                'enable_depth': 'false',
                'enable_rect': 'true',
            }.items()
        ),

        # 2. Stereo visual odometry
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='rtabmap_odom',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'subscribe_imu': False,
                'use_sim_time': False
            }]
        ),

        # 3. EKF Node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path]
        ),

        # 4. RTAB-Map SLAM node
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_odom_info': True,
                'odom_topic': '/odometry/filtered',
                'use_sim_time': False
            }],
            remappings=[
                ('odom', '/odometry/filtered')
            ]
        )
    ])
