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
    # ---------- config paths ----------
    joy_teleop_config = os.path.join(
        get_package_share_directory('launch_node'),
        'config',
        'joy_teleop.yaml'
    )

    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    zukimo_display_launch = os.path.join(
        get_package_share_directory('zukimo_car'),
        'launch',
        'display.launch.py'
    )

    # ---------- IMU-related launch args ----------
    imu_enable_la = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Start the PSoC6 IMU bridge and Madgwick filter.'
    )
    imu_port_la = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyACM0',
        description='Serial port of the PSoC6 board.'
    )
    imu_baud_la = DeclareLaunchArgument(
        'imu_baud',
        default_value='230400',
        description='Baud rate for the PSoC6 serial.'
    )
    imu_frame_la = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame id stamped in IMU messages.'
    )
    imu_raw_topic_la = DeclareLaunchArgument(
        'imu_raw_topic',
        default_value='/imu/data',
        description='Raw IMU topic published by the bridge.'
    )
    imu_oriented_topic_la = DeclareLaunchArgument(
        'imu_oriented_topic',
        default_value='/imu/data_oriented',
        description='Orientation-corrected IMU topic (Madgwick output).'
    )
    use_mag_la = DeclareLaunchArgument(
        'use_mag',
        default_value='false',
        description='Use magnetometer in Madgwick filter.'
    )
    world_frame_la = DeclareLaunchArgument(
        'world_frame',
        default_value='enu',
        description='World frame for Madgwick filter (enu/ned).'
    )

    enable_imu = LaunchConfiguration('enable_imu')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    imu_raw_topic = LaunchConfiguration('imu_raw_topic')
    imu_oriented_topic = LaunchConfiguration('imu_oriented_topic')
    use_mag = LaunchConfiguration('use_mag')
    world_frame = LaunchConfiguration('world_frame')

    # ---------- joy args ----------
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Path to the joy teleop config file'
    )

    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='',
        description='Serial port for the joy_to_steer device. Leave empty to disable joy nodes.'
    )

    device_port = LaunchConfiguration('device_port')
    joy_condition = IfCondition(PythonExpression(["'", device_port, "' != ''"]))

    # ---------- joy nodes ----------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('joy_config')],
        # condition=joy_condition
    )

    joy_to_steer_node = Node(
        package='launch_node',
        executable='joy_to_steer',
        name='joy_to_steer',
        parameters=[{'device_port': device_port}],
        condition=joy_condition
    )

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

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )

    zukimo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zukimo_display_launch)
    )

    static_tf_royale_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_royale_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'royale_optical_frame']
    )

    # ---------- PSoC6 IMU bridge node ----------
    psoc6_bridge_node = Node(
        package='psoc6_motion_bridge',
        executable='psoc6_motion_bridge',
        name='psoc6_motion_bridge',
        parameters=[{
            'port': imu_port,
            'baud': imu_baud,
            'frame_id': imu_frame_id,
            'topic': imu_raw_topic
        }],
        condition=IfCondition(enable_imu),
        output='screen'
    )

    # ---------- Madgwick filter node -----------
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag': use_mag,
            'world_frame': world_frame
        }],
        remappings=[
            ('imu/data_raw', imu_raw_topic),
            ('imu/data', imu_oriented_topic),
        ],
        condition=IfCondition(enable_imu),
        output='screen'
    )

    # # ---------- zukimo display node -----------
    # joint_state_node = Node(
    #         package='joint_state_publisher_gui',
    #         executable='joint_state_publisher_gui',
    #         name='joint_state_publisher_gui',
    #         output='screen',
    #     ),
    # robot_state_node = Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         name='robot_state_publisher',
    #         output='screen',
    #         parameters=[{'robot_description': open('/tmp/zukimo_car.urdf').read()}],
    #     ),
    # rviz_node = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         arguments=['-d', 'rviz/view_config.rviz']
    #     )

    # ---------- Assemble LaunchDescription ----------
    ld = LaunchDescription([
        # args
        joy_la, device_port_arg,
        imu_enable_la, imu_port_la, imu_baud_la, imu_frame_la,
        imu_raw_topic_la, imu_oriented_topic_la, use_mag_la, world_frame_la
    ])

    # ld.add_action(joint_state_node)
    # ld.add_action(robot_state_node)
    # ld.add_action(rviz_node)

    ld.add_action(joy_node)
    ld.add_action(joy_to_steer_node)
    ld.add_action(tof_node)
    ld.add_action(realsense)
    ld.add_action(zukimo)
    ld.add_action(static_tf_royale_to_camera)

    ld.add_action(psoc6_bridge_node)
    ld.add_action(madgwick_node)

    return ld
