#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from os.path import join
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():

    vx01_description_pkg = get_package_share_directory('vx01_description')
    vx01_simulation_pkg  = get_package_share_directory('vx01_simulation')
    vx01_bringup_pkg     = get_package_share_directory('vx01_bringup')

    description_pkg_parent = os.path.dirname(vx01_description_pkg)
    bridge_yaml = os.path.join(vx01_simulation_pkg, 'config', 'bridge.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Ignition Gazebo simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_file = LaunchConfiguration(
        'world_file',
        default=join(vx01_simulation_pkg, 'worlds', 'empty_world.sdf')
    )

    xacro_file = os.path.join(vx01_description_pkg, 'urdf', 'vx01.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']),
        value_type=str
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=description_pkg_parent
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        output='screen',
    )

    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='vx01_bridge',
        parameters=[{
            'config_file': bridge_yaml,
            'use_sim_time': use_sim_time,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_vx01',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'vx01',
                    '-x', '0.0', '-y', '0.0', '-z', '0.035', '-Y', '0.0',
                ],
                output='screen'
            )
        ]
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    # Chain: t+8s → JSB → hexapod_controller → drone_arm_controller

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='hexapod_controller_spawner',
        arguments=[
            'hexapod_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    drone_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='drone_arm_controller_spawner',
        arguments=[
            'drone_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # ── Sequential loading chain ──────────────────────────────────────────────
    # t+8s → JSB → hexapod_controller → drone_arm_controller

    load_jsb = TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # JSB exits → hexapod_controller
    load_hexapod = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[hexapod_controller_spawner],
        )
    )

    # hexapod_controller exits → drone_arm_controller
    load_drone = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hexapod_controller_spawner,
            on_exit=[drone_arm_controller_spawner],
        )
    )

    return LaunchDescription([
        set_ign_resource_path,
        use_sim_time_arg,
        robot_state_publisher_node,
        gz_sim,
        bridge_node,
        spawn_entity,
        load_jsb,
        load_hexapod,
        load_drone,
    ])