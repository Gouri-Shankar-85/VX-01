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

    # Launch ArduPilot SITL binary.
    # --out udp:127.0.0.1:14550 → MAVROS receives on UDP (fire-and-forget, no blocking)
    # TCP port 5760 still active for Mission Planner / GCS tools
    sitl_layer = ExecuteProcess(
        cmd=[
            '/ardupilot/build/sitl/bin/arducopter',
            '-S',
            '--model',    'JSON',
            '--speedup',  '1',
            '--slave',    '0',
            '--defaults',
            '/ardupilot/Tools/autotest/default_params/copter.parm,'
            '/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,'
            '/vx01_ws/src/vx01_bringup/config/ardupilot_sim_bypass.parm',
            '--sim-address', '127.0.0.1',
            # CRITICAL: '--out' does NOT exist in this ArduPilot build.
            # Use '--serial0' to set SERIAL0 (MAVLink port) to UDP output.
            # Verified: --serial0 udp:... launches SITL successfully; --out causes exit code 1.
            '--serial0',  'udp:127.0.0.1:14550',
            '-I',         '0',
        ],
        cwd='/ardupilot/ArduCopter',
        output='screen'
    )

    # Launch MAVROS — use UDP (not TCP) to prevent param sync from blocking heartbeat.
    # TCP: if param sync stalls, it blocks heartbeat → 'Lost connection' → sync restarts forever.
    # UDP: param timeouts can't block heartbeat — FCU link stays stable during full param download.
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
        # UDP MAVROS: bind on all interfaces so ArduPilot's --out udp:127.0.0.1:14550 arrives.
            # Format: udp://[local_bind_addr]:[local_port]@  (empty @ = no GCS forward)
            {'fcu_url': 'udp://0.0.0.0:14550@'},
            {'use_sim_time': True},
            os.path.join(get_package_share_directory('vx01_bringup'), 'config', 'mavros_sim_config.yaml'),
            {'gcs_url': ''},
            {'target_system_id': 1},
            {'target_component_id': 1},
            {'fcu_protocol': 'v2.0'},
            # 'param' plugin MUST be active for dashboard ARMING_CHECK bypass to work
            {'plugin_denylist': ['ftp']}
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

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
        sim_launch,
        sitl_layer,
        TimerAction(
            # Delay is critical: ArduPilot SITL needs ~12-15s to init JSON physics bridge.
        # MAVROS connecting too early → 'Lost connection' loop.
        # 18s gives headroom for slow machines and sim startup jitter.
        period=18.0,
            actions=[mavros_node, mode_manager, aerial_controller, web_video_server]
        )
    ])
