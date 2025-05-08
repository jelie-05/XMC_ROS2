# Adapted from https://github.com/f1tenth/f1tenth_system/blob/foxy-devel/f1tenth_stack/launch/bringup_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('launch_node'),
        'config',
        'joy_teleop.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Path to the joy teleop config file'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('joy_config')],
    )

    # joy_teleop_node = Node(
    #     package = 'joy_teleop',
    #     executable = 'joy_teleop',
    #     name = 'joy_teleop',
    #     parameters = [LaunchConfiguration('joy_config')],
    # )

    joy_to_steer_node = Node(
        package = 'launch_node',
        executable = 'joy_to_steer',
        name = 'joy_to_steer',
    )

    # finalize
    ld = LaunchDescription([joy_la])
    ld.add_action(joy_node)
    ld.add_action(joy_to_steer_node)

    return ld