# Based off Stereo Launch example found here:
#   https://github.com/introlab/rtabmap_ros/blob/jazzy-devel/rtabmap_demos/launch/stereo_outdoor_demo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_imu = LaunchConfiguration('use_imu')
    use_raspi = LaunchConfiguration('use_raspi')
    rviz = LaunchConfiguration('rviz')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')

    parameters={
          'frame_id':'camera_link',
          'subscribe_stereo':False,
          'subscribe_rgbd':True,
          'approx_sync':False, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored':True,
          'subscribe_odom_info': True,

          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '1000',
          'GFTT/MinDistance': '10',
          'GFTT/QualityLevel': '0.00001',
          #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d

          'wait_imu_to_init':False,
          'subscribe_imu': False,
        }

    remappings=[
          ('imu', '/imu/data'),
          ('imu/data_raw', '/camera/imu'),
        #   ('rgbd_image', '/stereo_camera/rgbd_image'),
        ]

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
        DeclareLaunchArgument(
            'localization', 
            default_value='false',  
            description='Launch in localization mode.'
        ),
        DeclareLaunchArgument(
            'rtabmap_viz', 
            default_value='false',  
            description='Launch rtabmap visualization'
        ),
        DeclareLaunchArgument(
            'rviz', 
            default_value='false',  
            description='Launch rviz visualization'
        ),

        # Launch camera driver ad stereo_image_proc
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
                'enable_depth': 'false',
                'enable_rect': 'true',
                'rviz': rviz
            }.items()
        ),

        # IMU Publisher and Processing
        Node(
            package='navis_nav', executable='imu_publisher', output='screen',
            condition=IfCondition(use_imu),
            parameters=[parameters],
            remappings=remappings,
        ),

        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            condition=IfCondition(use_imu),
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=remappings),

        # Syncronization
        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            # namespace='stereo_sync',
            parameters=[parameters],
            # remappings=[
            #     ('stereo_sync/left/image_rect',   'left/image_rect'),
            #     ('stereo_sync/right/image_rect',  'right/image_rect'),
            #     ('stereo_sync/left/camera_info',  'left/camera_info'),
            #     ('stereo_sync/right/camera_info', 'right/camera_info')]
            ),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),
        
        # Mapping mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(rtabmap_viz),
            parameters=[parameters],
            remappings=remappings),

        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(rviz),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
    ])