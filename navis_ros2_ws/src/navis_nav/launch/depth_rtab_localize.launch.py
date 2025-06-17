# Depth image based SLAM Based off the RTAB-Map Kinect example launch file found here:
#     https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/kinect_xbox_360.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # localization = LaunchConfiguration('localization')
    
    use_imu = LaunchConfiguration('use_imu')
    use_raspi = LaunchConfiguration('use_raspi')
    rviz = LaunchConfiguration('rviz')

    parameters=[{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'subscribe_imu': True,
        'use_sim_time': False
    }]

    remappings=[
          ('rgb/image', '/camera/right/image_raw'),
          ('rgb/camera_info', '/camera/right/camera_info'),
          ('depth/image', '/camera/depth/image_raw'),
          ('imu', '/imu/data_raw')]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='Use IMU input (set to false for laptops or non-IMU devices)'
        ),
        DeclareLaunchArgument(
            'use_raspi',
            default_value='false',
            description='Use Raspberry Pi config if true, laptop config otherwise'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Whether to start RVIZ'
        ),

        # Include stereo_cam.launch.py with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('stereo_cam'),
                    'launch',
                    'stereo_cam.launch.py'
                ])
            ),
            launch_arguments={
                'use_raspi': use_raspi,
                'enable_depth': 'true',
                'enable_rect': 'false',
                'rviz': rviz
            }.items()
        ),

        Node(
            package='navis_nav', executable='imu_publisher', output='screen',
            condition=IfCondition(use_imu),
            parameters=parameters,
            remappings=remappings,
        ),

        # Optical rotation
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect_rgb"]),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
            namespace="rtabmap"),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"),
    ])