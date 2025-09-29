from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
import os

def generate_launch_description():
    # Define config path for joy teleop
    joy_teleop_config = os.path.join(
        get_package_share_directory('launch_node'),
        'config',
        'joy_teleop.yaml'
    )

    # Realsense Launch File
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # Launch argument for joystick config
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Path to the joy teleop config file'
    )

    # device_port can be empty string to mean "disabled"
    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='',
        description='Serial port for the joy_to_steer device. Leave empty to disable joy nodes.'
    )

    device_port = LaunchConfiguration('device_port')

    # Condition: only start joy nodes if device_port != ""
    joy_condition = IfCondition(PythonExpression(["'", device_port, "' != ''"]))

    # Joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('joy_config')],
        # condition=joy_condition
    )

    # Joystick to steering converter
    joy_to_steer_node = Node(
        package='launch_node',
        executable='joy_to_steer',
        name='joy_to_steer',
        parameters=[{'device_port': device_port}],
        condition=joy_condition
    )

    # Royale ToF node
    royale_root = os.path.expanduser('~/Documents/libroyale-4.24.0.1201-LINUX-x86-64Bit')
    tof_node = Node(
        package='royale_ros2',
        executable='royale_node',
        name='royale_node',
        additional_env={
            'ROYALE_ROOT': royale_root,
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', '') + ':' + os.path.join(royale_root, 'bin'),
            'PYTHONPATH': os.environ.get('PYTHONPATH', '') + ':' +
                          os.path.join(royale_root, 'build-pywrap') + ':' +
                          os.path.join(royale_root, 'build-pywrap', 'bin'),
        },
        output='screen'
    )

    # Include the RealSense launch file
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )

    # # RViz2 (force XCB already handled by Qt on your system; add CycloneDDS + small delay)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     additional_env={
    #         'QT_QPA_PLATFORM': 'xcb',               # stable GUI backend
    #         'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'  # avoid FastDDS-related crashes
    #     }
    # )

    # # Start RViz a bit later so camera topics are up
    # rviz_delayed = TimerAction(period=2.0, actions=[rviz_node])

    static_tf_royale_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_royale_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'royale_optical_frame']
    )

    # Finalize LaunchDescription
    ld = LaunchDescription([joy_la, device_port_arg])
    ld.add_action(joy_node)
    ld.add_action(joy_to_steer_node)
    ld.add_action(tof_node)
    ld.add_action(realsense)
    # ld.add_action(rviz_delayed)
    ld.add_action(static_tf_royale_to_camera)

    return ld
