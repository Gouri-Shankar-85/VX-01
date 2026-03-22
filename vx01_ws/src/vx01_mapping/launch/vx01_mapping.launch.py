from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_to_camera',
            arguments=['0', '0', '0', '0', '0', '0',
                       'odom', 'vx01_camera_ascamera_0'],
        ),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time':                use_sim_time,
                'frame_id':                    'vx01_camera_ascamera_0',
                'odom_frame_id':               'odom',
                'map_frame_id':                'map',
                'subscribe_rgb':               True,
                'subscribe_depth':             True,
                'subscribe_imu':               True,
                'approx_sync':                 True,
                'wait_imu_to_init':            True,
                'queue_size':                  10,

                # RTAB-Map parameters
                'RGBD/NeighborLinkRefining':   'true',
                'RGBD/ProximityBySpace':       'true',
                'RGBD/AngularUpdate':          '0.01',
                'RGBD/LinearUpdate':           '0.01',
                'RGBD/OptimizeFromGraphEnd':   'false',
                'Reg/Force3DoF':               'false',
                'Vis/MinInliers':              '12',
                'Icp/VoxelSize':               '0.05',
                'Icp/MaxCorrespondenceDistance': '0.1',
            }],
            remappings=[
                ('rgb/image',        '/vx01_camera/camera_publisher/rgb0/image'),
                ('depth/image',      '/vx01_camera/camera_publisher/depth0/image_raw'),
                ('rgb/camera_info',  '/vx01_camera/camera_publisher/depth0/camera_info'),
                ('imu',              '/imu'),
            ],
        ),
    ])