import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('vx01_hexapod_locomotion')
    config = PathJoinSubstitution([pkg, 'config', 'hexapod.yaml'])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    hexapod_node = Node(
        package='vx01_hexapod_locomotion',
        executable='hexapod_node',
        name='hexapod_node',
        output='screen',
        parameters=[
            config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        hexapod_node,
    ])
