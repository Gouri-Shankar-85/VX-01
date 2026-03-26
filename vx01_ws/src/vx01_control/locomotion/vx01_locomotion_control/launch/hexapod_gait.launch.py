#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vx01_locomotion_control'),
        'config', 'hexapod_gait.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (true for Gazebo, false for hardware)'
    )

    gait_node = Node(
        package='vx01_locomotion_control',
        executable='hexapod_gait_node',
        name='hexapod_gait_node',
        parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        gait_node,
    ])
