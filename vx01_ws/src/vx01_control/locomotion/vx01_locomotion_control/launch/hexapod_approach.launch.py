#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vx01_locomotion_control'),
        'config', 'hexapod_locomotion.yaml'
    )

    forward_walk_node = LifecycleNode(
        package='vx01_locomotion_control',
        executable='hexapod_forward_walk_node',
        name='hexapod_forward_walk_node',
        namespace='',
        parameters=[config, {'use_sim_time': False}],
        output='screen',
    )

    turn_node = LifecycleNode(
        package='vx01_locomotion_control',
        executable='hexapod_turn_node',
        name='hexapod_turn_node',
        namespace='',
        parameters=[config, {'use_sim_time': False}],
        output='screen',
    )

    approach_node = Node(
        package='vx01_locomotion_control',
        executable='hexapod_approach_node',
        name='hexapod_approach_node',
        output='screen',
        parameters=[{
            'arrival_distance': 0.5,
            'turn_threshold':   0.15,
        }]
    )

    return LaunchDescription([
        forward_walk_node,
        turn_node,
        approach_node,
    ])