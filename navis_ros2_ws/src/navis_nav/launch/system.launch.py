import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_raspi = LaunchConfiguration('use_raspi')
    rviz = LaunchConfiguration('rviz')

    navis_nav_path = get_package_share_directory('navis_nav')

    # 1. High-Level Waypoint Management
    waypoint_manager = Node(
        package='navis_nav',
        executable='waypoint_manager',
        name='waypoint_manager'
    )

    waypoint_manager_delayed = TimerAction(
        period=20.0,  # seconds
        actions=[waypoint_manager]
    )

    # 2. Mid-Level Stereo-SLAM stack
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navis_nav_path,
                'launch',
                'rtab_slam_stereo.launch.py'
            )
        ),
        launch_arguments={
            'use_raspi': use_raspi,
            'rviz': rviz,
            'log_level': 'WARN',
            'localization': 'true',
        }.items()
    )

    # 3. Low-Level Planning, Control, and Hardware Interfacing Nodes
    obstacle_detector = Node(
        package='navis_nav',
        executable='obstacle_detector',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    control_out_node = Node(
        package='navis_nav',
        executable='control_out_node',
    )
    

    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('use_raspi',    default_value='true', description='Use Raspberry Pi config if true, laptop config otherwise' ),

        waypoint_manager_delayed,
        slam,
        control_out_node,
        obstacle_detector
    ])
