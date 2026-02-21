#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:       vx01_world.launch.py
*  Description:    Launch Ignition Gazebo Fortress world
*  Modified by:    Gouri Shankar
*****************************************************************************************
'''

import os
from os.path import join
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():

    vx01_description_pkg = get_package_share_directory("vx01_description")
    vx01_simulation_pkg = get_package_share_directory("vx01_simulation")

    description_pkg_parent = os.path.dirname(vx01_description_pkg)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file = os.path.join(vx01_description_pkg, 'urdf', 'vx01.urdf.xacro')

    world_file = LaunchConfiguration(
        "world_file",
        default=join(vx01_simulation_pkg, "worlds", "empty_world.sdf")
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── Fix: expose vx01_description to Ignition's model:// resolver ──
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=description_pkg_parent
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen',
    )

    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    # Spawn Entity (Robot in Gazebo)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'vx01',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.2',
                    '-Y', '0.0',
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Environment must be set FIRST, before Gazebo launches
        set_ign_resource_path,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument("world_file", default_value=world_file),

        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
    ])