#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:           vx01_rviz.launch.py
*  Description:        Launch the bot in Rviz with its URDF model
*  Modified by:        Gouri Shankar
*****************************************************************************************
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    
    #Package details
    pkg_name = 'vx01_description'
    pkg_velatrix_share = get_package_share_directory(pkg_name)
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    #Process the URDF file using xacro
    xacro_file = os.path.join(pkg_velatrix_share, 'urdf', 'vx01.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    robot_state_publisher_node= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                    'robot_description': robot_description}],
        output='screen',
    ) 
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],        
        output='screen',
    )    
    
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_path],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    # ) 
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # rviz_node,
    ])
    