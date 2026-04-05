import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rtabmap_dir = get_package_share_directory('rtabmap_launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            # Odometry tuning: lower inlier threshold for sim low-texture scenes
            # Use GFTT detector (type=1), larger F2M map, auto-reinit on lost odom
            'rtabmap_args':         '--delete_db_on_start'
                                    ' --Vis/MinInliers 10'
                                    ' --Vis/FeatureType 1'
                                    ' --OdomF2M/MaxSize 3000'
                                    ' --Odom/ResetCountdown 1',
            'frame_id':             'base_footprint',
            'visual_odometry':      'true',
            'rgb_topic':            '/depth_camera/color/image_raw',
            'depth_topic':          '/depth_camera/depth/image_raw',
            'camera_info_topic':    '/depth_camera/camera_info',
            # NOTE: do NOT pass imu_topic:='' — that generates '-r imu:=' which
            # crashes rtabmap. Instead disable IMU waiting; spam errors are harmless.
            'wait_imu_to_init':     'false',
            'approx_sync':          'true',
            'qos':                  '2',
            'use_sim_time':         use_sim_time,
            'rviz':                 'false',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        rtabmap_launch
    ])
