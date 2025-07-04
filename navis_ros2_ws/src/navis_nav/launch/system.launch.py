from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, Command
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os

def generate_launch_description():

    use_raspi = LaunchConfiguration('use_raspi')
    rviz = LaunchConfiguration('rviz')
    rtabmap_rviz = LaunchConfiguration('rtabmap_rviz')

    navis_nav_path = get_package_share_directory('navis_nav')

    robot_description = Command(["xacro ", os.path.join(navis_nav_path, "urdf", "robot.urdf")])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

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

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('navis_nav'),
                'launch',
                'depth_rtab_localize.launch.py'
            )
        ),
        launch_arguments={
            'use_raspi': use_raspi,
            'rviz': rviz,
            'rtabmap_rviz': rtabmap_rviz,
        }.items()
    )

    # Delay node_b until node_a exits
    system_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=waypoint_orderer,
            on_exit=[slam]
        )
    )

    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('use_raspi', default_value='false', description='Use Raspberry Pi config if true, laptop config otherwise' ),

        robot_state_publisher,
        controller_manager,
        spawn_gpio_controller,
        waypoint_orderer,
        waypoint_manager,
        system_delay
    ])
