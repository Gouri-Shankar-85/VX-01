#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():

    vx01_description_pkg = get_package_share_directory('vx01_description')
    vx01_bringup_pkg     = get_package_share_directory('vx01_bringup')

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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description,
        }],
        output='screen',
    )

    # ros2_control controller manager — talks directly to HexapodHardwareInterface
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            {'robot_description': robot_description},
            controller_yaml,
            {'use_sim_time': False},
        ],
        output='screen',
    )

    def make_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            name=f'{name}_spawner',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '30',
            ],
            output='screen',
        )

    jsb_spawner   = make_spawner('joint_state_broadcaster')
    leg_0_spawner = make_spawner('leg_0_controller')
    leg_1_spawner = make_spawner('leg_1_controller')
    leg_2_spawner = make_spawner('leg_2_controller')
    leg_3_spawner = make_spawner('leg_3_controller')
    leg_4_spawner = make_spawner('leg_4_controller')
    leg_5_spawner = make_spawner('leg_5_controller')

    # Give controller_manager 3 seconds to start and load the hardware plugin
    load_jsb = TimerAction(period=3.0, actions=[jsb_spawner])

    load_leg_0 = RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner, on_exit=[leg_0_spawner]))
    load_leg_1 = RegisterEventHandler(OnProcessExit(
        target_action=leg_0_spawner, on_exit=[leg_1_spawner]))
    load_leg_2 = RegisterEventHandler(OnProcessExit(
        target_action=leg_1_spawner, on_exit=[leg_2_spawner]))
    load_leg_3 = RegisterEventHandler(OnProcessExit(
        target_action=leg_2_spawner, on_exit=[leg_3_spawner]))
    load_leg_4 = RegisterEventHandler(OnProcessExit(
        target_action=leg_3_spawner, on_exit=[leg_4_spawner]))
    load_leg_5 = RegisterEventHandler(OnProcessExit(
        target_action=leg_4_spawner, on_exit=[leg_5_spawner]))

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        controller_manager,
        load_jsb,
        load_leg_0,
        load_leg_1,
        load_leg_2,
        load_leg_3,
        load_leg_4,
        load_leg_5,
    ])