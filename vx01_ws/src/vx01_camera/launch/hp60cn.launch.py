from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "vx01_camera_hp60cn",
            package='vx01_camera',
            executable='ascamera_node',
            respawn=True,
            output='both',
            parameters=[
                {"confiPath": "./vx01_camera/configurationfiles"},
				{"pub_tfTree": True},
                {"color_pcl": False}
            ]
        ),
    ])
