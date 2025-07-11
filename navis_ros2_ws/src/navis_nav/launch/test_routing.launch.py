from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file created to independently test the functionality of the ordering and management systems

ros2 topic pub /goal_reached std_msgs/msg/Bool "{data: true}"

"""

def generate_launch_description():
    return LaunchDescription([
        Node(package='navis_nav', executable='waypoint_orderer', output='screen'),
        Node(package='navis_nav', executable='waypoint_manager', output='screen'),
    ])