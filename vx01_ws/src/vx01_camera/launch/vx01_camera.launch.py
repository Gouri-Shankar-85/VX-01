from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    vx01_camera_node = Node(
        namespace= "vx01_camera",
        package='vx01_camera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "/vx01_ws/src/vx01_camera/configurationfiles"},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": -1},
            {"rgb_height": -1},
            {"fps": 15},
        ],
        remappings=[]
    )

    ld.add_action(vx01_camera_node)
    return ld
