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
            # Strategy: ORB visual odometry with very low thresholds.
            # ICP not used here since it needs point cloud topic configuration.
            # OdomStrategy 0 = F2M (frame-to-map), most robust for slow motion.
            # DepthAsMask 0: don't skip frames just because some pixels lack depth.
            # Kp/RoiRatios: mask top 20% (sky/ceiling = always invalid depth).
            # MinInliers 3: very permissive for low-texture scenes.
            # MaxFeatures 2000: extract as many features as possible.
            'rtabmap_args':             '--delete_db_on_start'
                                        ' --Vis/FeatureType 11'
                                        ' --Kp/DetectorStrategy 11'
                                        ' --Vis/MaxFeatures 2000'
                                        ' --Kp/MaxFeatures 2000'
                                        ' --Vis/MinInliers 3'
                                        ' --OdomF2M/MinInliers 3'
                                        ' --OdomF2M/MaxSize 5000'
                                        ' --Odom/ResetCountdown 1'
                                        ' --Odom/GuessMotion false'
                                        ' --Vis/MaxDepth 3.5'
                                        ' --Vis/DepthAsMask false'
                                        ' --Kp/RoiRatios "0 0 0.2 0"'
                                        ' --OdomF2M/BundleAdjustment 0',
            'frame_id':                 'base_footprint',
            'visual_odometry':          'true',
            'rgb_topic':                '/depth_camera/color/image_raw',
            'depth_topic':              '/depth_camera/depth/image_raw',
            'camera_info_topic':        '/depth_camera/camera_info',
            # DO NOT pass imu_topic:='' — crashes rtabmap with invalid remap rule
            'wait_imu_to_init':         'false',
            'approx_sync':              'true',
            # 150ms window tolerates the ~66ms RGB/depth timestamp gap in sim
            'approx_sync_max_interval': '0.15',
            'qos':                      '2',
            'use_sim_time':             use_sim_time,
            'rviz':                     'false',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        rtabmap_launch
    ])

