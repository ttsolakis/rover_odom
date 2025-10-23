#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def _latest_bag_dir():
    base = os.path.expanduser('~/bags')
    try:
        dirs = [os.path.join(base, d)
                for d in os.listdir(base)
                if d.startswith('rover_') and os.path.isdir(os.path.join(base, d))]
        return sorted(dirs)[-1] if dirs else ''
    except Exception:
        return ''

def generate_launch_description():
    default_bag = _latest_bag_dir()
    if not default_bag:
        raise RuntimeError(
            "No bag directory found under ~/bags matching 'rover_*'. "
            "Create one or pass bag:=/full/path explicitly."
        )

    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value=default_bag,
        description='Path to rosbag2 directory (defaults to newest ~/bags/rover_*)'
    )

    rover_pkg = get_package_share_directory('rover_odom')
    urdf_path = os.path.join(rover_pkg, 'urdf', 'rover.urdf.xacro')
    rviz_config = os.path.join(rover_pkg, 'rviz', 'rover_mapping.rviz')
    xacro_exec = FindExecutable(name='xacro')

    # Publish robot_description (URDF) + /tf_static so the RobotModel shows up
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command([xacro_exec, ' ', urdf_path]),
                value_type=str
            ),
        }],
        output='screen',
    )

    # Rebuild /map from /scan during playback
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'use_odometry': False,
            'tf_buffer_duration': 10.0,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    # Delay playback a bit so TF + slam are ready
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', LaunchConfiguration('bag'),
            '--clock', '--loop',
            '--topics', '/scan', '/tf', '/tf_static'
        ],
        output='screen',
    )
    bag_play_delayed = TimerAction(period=2.0, actions=[bag_play])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        bag_arg,
        robot_state_pub,     # <-- adds URDF box
        slam,
        lifecycle_manager,
        bag_play_delayed,
        rviz,
    ])
