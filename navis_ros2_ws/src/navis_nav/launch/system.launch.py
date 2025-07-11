import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_raspi = LaunchConfiguration('use_raspi')
    rviz = LaunchConfiguration('rviz')

    navis_nav_path = get_package_share_directory('navis_nav')

    # 1. High-Level Waypoint Management
    waypoint_orderer = Node(
        package='navis_nav',
        executable='waypoint_orderer',
        name='waypoint_orderer'
    )

    waypoint_manager = Node(
        package='navis_nav',
        executable='waypoint_manager',
        name='waypoint_manager'
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
            'log_level': 'WARN'
        }.items()
    )

    # 3. Low-Level Planning, Control, and Hardware Interfacing Nodes
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(navis_nav_path, "config", "gpio_controller.yaml")],
        output="screen",
    )

    spawn_gpio_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller"],
        output="screen",
    )

    # control_out_node = Node(
    #     package='navis_nav',
    #     executable='control_out_node',
    # )

    control_action_calc = Node(
        package='navis_nav',
        executable='control_action_calc'
    )

    # Delay node_b until node_a exits
    # system_delay = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=waypoint_orderer,
    #         on_exit=[slam]
    #     )
    # )

    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('use_raspi',    default_value='true', description='Use Raspberry Pi config if true, laptop config otherwise' ),

        controller_manager,
        spawn_gpio_controller,
        waypoint_orderer,
        waypoint_manager,
        # control_out_node,
        control_action_calc,
        # system_delay
    ])
