import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            parameters=[
              {'use_sim_time': True},
              {'max_laser_range': 20.0},
              {'resolution': 0.05},
              {'base_frame': 'base_footprint'},
              {'map_frame': 'map'},
              {'odom_frame': 'odom'}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])
