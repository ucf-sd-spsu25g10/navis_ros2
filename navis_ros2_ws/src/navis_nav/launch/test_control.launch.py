from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file created to independently test the functionality of the control_out_node
When wired appropriately over the UART bus, launch this and run ros2 topic list to ensure data is being pushed to the MCU

ros2 topic pub --once /control_output navis_msgs/msg/ControlOut "{buzzer_strength: 100, speaker_wav_index: 3}"

"""

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='navis_nav', executable='control_out_node', output='screen',
        ),
    ])