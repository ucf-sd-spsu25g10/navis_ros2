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

        # Node(
        #     package='navis_nav', executable='imu_publisher', output='screen',
        #     remappings=remappings,
        # ),

        Node(
            package="mpu9250",
            executable="mpu9250",
            name="mpu9250",
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[
            {
                #"print" : True,
                "frequency" : 10,
                "i2c_address" : 0x68,
                "i2c_port" : 1,
                "frame_id" : "imu_link",
                "acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535],
                "acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202],
                "gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845],
                "magnetometer_scale": [1.0, 1.0, 1.0],
                "magnetometer_bias": [1.335, 4.0, -2.53],
                "magnetometer_transform": [1.0246518952703103, -0.0240401565528902, 0.0030740476998857395,
                                            -0.024040156552890175, 0.9926708357001245, 0.002288563295390304,
                                            0.0030740476998857356, 0.0022885632953903268, 0.9837206150979054]
            }
            ],
        ),

        Node(
            package='navis_nav', executable='bag_recorder', output='screen',
        )
    ])