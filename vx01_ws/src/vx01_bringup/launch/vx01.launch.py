#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    vx01_bringup_pkg = get_package_share_directory('vx01_bringup')
    vx01_camera_pkg  = get_package_share_directory('vx01_camera')
    vx01_imu_pkg     = get_package_share_directory('vx01_imu')

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

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vx01_camera_pkg, 'launch', 'vx01_camera.launch.py')
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vx01_imu_pkg, 'launch', 'vx01_imu.launch.py')
        )
    )

    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    static_tf_camera_remap = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_remap',
        arguments=['0', '0', '0', '0', '0', '0',
                   'depth_camera_optical_frame', 'vx01_camera_ascamera_0'],
    )

    return LaunchDescription([
        serial_port_arg,
        hw_launch,
        camera_launch,
        imu_launch,
        static_tf_odom,
        static_tf_camera_remap,
    ])