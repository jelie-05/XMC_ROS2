from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 1.0,
                'scale_angular.yaw': 1.0,
            }],
            remappings=[('/cmd_vel', '/zukimo_car/cmd_vel')],
            output='screen'
        )
    ])
