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

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vx01_sim_pkg, 'launch', 'vx01_sim.launch.py')
        )
    )

    sitl_layer = ExecuteProcess(
        cmd=[
            '/ardupilot/build/sitl/bin/arducopter',
            '--model', 'quad',
            '--serial0=tcp:0.0.0.0:5760',
            '--defaults',
            '/ardupilot/Tools/autotest/default_params/copter.parm'
        ],
        output='screen'
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            {'fcu_url': 'tcp://127.0.0.1:5760'},
            {'use_sim_time': True},
            os.path.join(
                get_package_share_directory('vx01_bringup'),
                'config',
                'mavros_sim_config.yaml'
            ),
            {'gcs_url': ''},
            {'target_system_id': 1},
            {'target_component_id': 1},
            {'fcu_protocol': 'v2.0'},
            {'plugin_denylist': ['ftp']}
        ]
    )

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

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    set_guided_mode = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/mavros/set_mode',
            'mavros_msgs/srv/SetMode',
            "{custom_mode: 'GUIDED'}"
        ],
        output='screen'
    )

    arm_vehicle = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/mavros/cmd/arming',
            'mavros_msgs/srv/CommandBool',
            "{value: true}"
        ],
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        sitl_layer,

        TimerAction(
            period=20.0,
            actions=[mavros_node]
        ),

        TimerAction(
            period=23.0,
            actions=[mode_manager, aerial_controller, web_video_server]
        ),

        TimerAction(
            period=35.0,
            actions=[set_guided_mode]
        ),

        TimerAction(
            period=120.0,
            actions=[arm_vehicle]
        ),

        TimerAction(
            period=130.0,
            actions=[arm_vehicle]
        ),

        TimerAction(
            period=150.0,
            actions=[arm_vehicle]
        ),

        TimerAction(
            period=170.0,
            actions=[arm_vehicle]
        ),

        TimerAction(
            period=200.0,
            actions=[arm_vehicle]
        ),

        TimerAction(
            period=220.0,
            actions=[arm_vehicle]
        )
    ])