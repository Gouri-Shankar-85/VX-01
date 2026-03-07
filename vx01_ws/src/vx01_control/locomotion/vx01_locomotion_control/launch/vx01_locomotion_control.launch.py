#!/usr/bin/env python3
"""
vx01_locomotion_control.launch.py

Launches the VX-01 locomotion action server.

Run standalone (after the sim + controllers are up):
    ros2 launch vx01_locomotion_control vx01_locomotion_control.launch.py

Or include from vx01_sim.launch.py after the last leg spawner.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('vx01_locomotion_control')
    params_file = os.path.join(pkg_share, 'config', 'locomotion_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    locomotion_node = Node(
        package='vx01_locomotion_control',
        executable='locomotion_node',
        name='locomotion_node',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        use_sim_time_arg,
        locomotion_node,
    ])
