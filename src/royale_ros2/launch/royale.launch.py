from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="royale_ros2",
            executable="royale_node",
            name="royale_node",
            parameters=[{
                "frame_id": "royale_optical_frame",
                "publish_cloud": True
            }]
        )
    ])
