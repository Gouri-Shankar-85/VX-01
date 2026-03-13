#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():

    vx01_description_pkg = get_package_share_directory('vx01_description')
    vx01_bringup_pkg = get_package_share_directory('vx01_bringup')

    controller_yaml = os.path.join(vx01_bringup_pkg, 'config', 'hexapod', 'hexapod_controller_manager_hw.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyMAESTRO',
        description='Serial port for Pololu Maestro (check: ls /dev/ttyMAESTRO*)'
    )
    serial_port = LaunchConfiguration('serial_port')

    xacro_file = os.path.join(vx01_description_pkg, 'urdf', 'vx01.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_sim:=false',
                 ' serial_port:=', serial_port,
                 ' baud_rate:=115200']),
        value_type=str
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml, {'robot_description': robot_description, 'use_sim_time': False}],
        output="both",
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    leg0_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_0_controller", "-c", "/controller_manager"],
    )
    
    leg1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_1_controller", "-c", "/controller_manager"],
    )
    
    leg2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_2_controller", "-c", "/controller_manager"],
    )
    
    leg3_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_3_controller", "-c", "/controller_manager"],
    )
    
    leg4_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_4_controller", "-c", "/controller_manager"],
    )
    
    leg5_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_5_controller", "-c", "/controller_manager"],
    )
    
    delay_leg0_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg0_controller_spawner],
        )
    )
    
    delay_leg1_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg1_controller_spawner],
        )
    )
    
    delay_leg2_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg2_controller_spawner],
        )
    )
    
    delay_leg3_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg3_controller_spawner],
        )
    )
    
    delay_leg4_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg4_controller_spawner],
        )
    )
    
    delay_leg5_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg5_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_leg0_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_leg1_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_leg2_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_leg3_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_leg4_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_leg5_controller_spawner_after_joint_state_broadcaster_spawner
    ]
    
    return LaunchDescription([serial_port_arg] + nodes)