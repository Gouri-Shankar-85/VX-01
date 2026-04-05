#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    vx01_sim_pkg = get_package_share_directory('vx01_simulation')

    # Include core simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vx01_sim_pkg, 'launch', 'vx01_sim.launch.py')
        )
    )

    # Launch ArduPilot SITL
    sitl_layer = ExecuteProcess(
        cmd=[
            'sim_vehicle.py', 
            '-v', 'ArduCopter', 
            '-f', 'gazebo-iris', 
            '--out', '127.0.0.1:14550',
            '--no-rebuild'
        ],
        output='screen'
    )

    # Launch MAVROS connecting to SITL
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14550@14555'},
            {'gcs_url': ''},
            {'target_system_id': 1},
            {'target_component_id': 1},
            {'fcu_protocol': 'v2.0'}
        ]
    )

    # Hybrid Mode Nodes
    mode_manager = Node(
        package='vx01_mode_manager',
        executable='mode_manager_node',
        output='screen'
    )

    aerial_controller = Node(
        package='vx01_aerial_control',
        executable='aerial_controller_node',
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        sitl_layer,
        TimerAction(
            period=10.0,
            actions=[mavros_node, mode_manager, aerial_controller]
        )
    ])
