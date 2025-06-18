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
            {'gyro_x_offset':  0.0 },  # If "calibrate" is true, these values will be overriden by the calibration procedure
            {'gyro_y_offset': 0.0},
            {'gyro_z_offset': 0.0 },
            {'accel_x_offset': 0.0 },
            {'accel_y_offset': 0.0 },
            {'accel_z_offset': 0.0 },
            {'frequency': 100 }
            ]
        ),

        Node(
            package='navis_nav', executable='bag_recorder', output='screen',
        )
    ])