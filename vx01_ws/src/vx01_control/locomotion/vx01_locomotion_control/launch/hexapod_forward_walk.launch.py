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

    node = Node(
        package='vx01_locomotion_control',
        executable='hexapod_forward_walk_node',
        name='hexapod_forward_walk_node',
        parameters=[config, {'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([node])