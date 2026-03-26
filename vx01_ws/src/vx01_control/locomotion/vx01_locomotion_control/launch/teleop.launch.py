import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("vx01_locomotion_control")
    config = PathJoinSubstitution([pkg, "config", "gait_config.yaml"])

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    teleop_node = Node(
        package="vx01_locomotion_control",
        executable="teleop_node",
        name="teleop_node",
        output="screen",
        prefix="xterm -e",
        parameters=[
            config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        teleop_node,
    ])
