from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file created to independently test the functionality of the IMU subscriber
When wired appropriately over the I2C bus, launch this and run ros2 topic list to ensure data is being published from the imu
"""

def generate_launch_description():

    remappings=[
          ('imu', '/imu/data'),]

    return LaunchDescription([

        Node(
            package='navis_nav', executable='imu_publisher', output='screen',
            remappings=remappings,
        ),

        Node(
            package='navis_nav', executable='bag_recorder', output='screen',
        )
    ])