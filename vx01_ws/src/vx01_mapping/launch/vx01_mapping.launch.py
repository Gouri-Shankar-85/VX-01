import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(
        get_package_share_directory('vx01_description'),
        'urdf', 'vx01.urdf.xacro'
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file, ' use_sim:=false']),
                'use_sim_time': use_sim_time,
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_frame_remap',
            arguments=['0', '0', '0', '0', '0', '0',
                       'depth_camera_optical_frame', 'vx01_camera_ascamera_0'],
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time':              use_sim_time,
                'frame_id':                  'depth_camera_optical_frame',
                'odom_frame_id':             'odom',
                'map_frame_id':              'map',
                'subscribe_rgb':             True,
                'subscribe_depth':           True,
                'subscribe_imu':             False,
                'approx_sync':               True,
                'sync_queue_size':           30,
                'topic_queue_size':          30,
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace':     'true',
                'RGBD/AngularUpdate':        '0.01',
                'RGBD/LinearUpdate':         '0.01',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Reg/Force3DoF':             'false',
                'Vis/MinInliers':            '12',
            }],
            remappings=[
                ('rgb/image',       '/vx01_camera/camera_publisher/rgb0/image'),
                ('depth/image',     '/vx01_camera/camera_publisher/depth0/image_raw'),
                ('rgb/camera_info', '/vx01_camera/camera_publisher/depth0/camera_info'),
            ],
        ),
    ])