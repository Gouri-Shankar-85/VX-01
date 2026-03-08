#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vx01_locomotion_control'),
        'config', 'hexapod_locomotion.yaml'
    )

    locomotion_node = Node(
        package='vx01_locomotion_control',
        executable='hexapod_locomotion_node',
        name='hexapod_locomotion_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([locomotion_node])
