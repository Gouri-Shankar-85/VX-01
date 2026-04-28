#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    vx01_bringup_pkg = get_package_share_directory('vx01_bringup')
    vx01_camera_pkg  = get_package_share_directory('vx01_camera')
    # vx01_imu_pkg     = get_package_share_directory('vx01_imu')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyMAESTRO',
        description='Serial port for Pololu Maestro'
    )

    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vx01_bringup_pkg, 'launch', 'vx01_hw_launch.py')
        ),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()
    )

    camera_launch = TimerAction(
        period=5.0,
        actions=[
            GroupAction(
                actions=[
                    SetRemap(src='/vx01_camera/camera_publisher/rgb0/image', dst='/depth_camera/color/image_raw'),
                    SetRemap(src='/vx01_camera/camera_publisher/depth0/image_raw', dst='/depth_camera/depth/image_raw'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(vx01_camera_pkg, 'launch', 'vx01_camera.launch.py')
                        )
                    )
                ]
            )
        ]
    )

    # imu_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(vx01_imu_pkg, 'launch', 'vx01_imu.launch.py')
    #     )
    # )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}]
    )

    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        parameters=[{'use_sim_time': False}],
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    static_tf_camera_remap = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_remap',
        parameters=[{'use_sim_time': False}],
        arguments=['0', '0', '0', '0', '0', '0',
                   'depth_camera_optical_frame', 'vx01_camera_ascamera_0'],
    )

    return LaunchDescription([
        serial_port_arg,
        hw_launch,
        camera_launch,
        # imu_launch,
        web_video_server,
        static_tf_odom,
        static_tf_camera_remap,
    ])