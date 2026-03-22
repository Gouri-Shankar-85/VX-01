from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vx01_imu',
            executable='vx01_imu_node',
            name='vx01_imu_node',
            output='screen',
            parameters=[{
                'port':      '/dev/ttyUSB0',
                'baud_rate': 9600,
                'frame_id':  'imu_link',
            }]
        )
    ])