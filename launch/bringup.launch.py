# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue   # <-- added
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Toggle starting teleop (needs a real terminal/TTY)
    start_teleop_arg = DeclareLaunchArgument(
        'start_teleop', default_value='true',
        description='Start rover_teleop in a separate terminal (WASD).'
    )

    # 1) LiDAR driver
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '460800',
            'frame_id': 'lidar_link'
        }.items()
    )

    # 2) IMU → Odometry (include your parameterized launch)
    rover_pkg = get_package_share_directory('rover_odom')
    imu_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_pkg, 'launch', 'imu_to_odom.launch.py')
        ),
        launch_arguments={
            'http_url': 'http://192.168.4.1/js',
            'request_mode': 'param',
            'poll_json': '{"T":126}',
            'rate_hz': '50.0',
            'rpy_is_deg': 'true',
            'gyro_is_deg': 'true',
            'accel_is_mg': 'true',
            'auto_level': 'true',
            'level_samples': '100',
            'level_gyro_thresh': '0.2',
            'level_g_tolerance': '2.0',
            'publish_tf': 'true',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'topic_name': '/odom',
            'timeout_s': '1.0',
            'mounting_rpy_deg': '[0.0, 0.0, 180.0]',
            'apply_mounting_tf_in_odom': 'true',
        }.items()
    )

    # 2) slam_toolbox
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'use_odometry': False,
        }]
    )

    # 3) Lifecycle manager to auto-configure/activate slam_toolbox
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }]
    )

    # 4) Robot State Publisher for the URDF
    urdf_path  = PathJoinSubstitution([FindPackageShare('rover_odom'), 'urdf', 'rover.urdf.xacro'])
    xacro_exec = FindExecutable(name='xacro')
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            # Use ParameterValue to force string type, and build the command with an explicit space
            'robot_description': ParameterValue(
                Command([xacro_exec, ' ', urdf_path]),
                value_type=str
            )
        }],
        output='screen'
    )

    # 5) RViz
    rviz_config = os.path.join(get_package_share_directory('rover_odom'), 'rviz', 'rover_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 6) Teleop (WASD) in a separate terminal so it can read keys
    #    Requires a terminal emulator; change to xterm/konsole/kitty if you prefer.
    teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-lc',
            # You can tweak params here (http_url, rate_hz, max_apply_s, send_http)
            'ros2 run rover_odom rover_teleop '
            '--ros-args '
            '-p http_url:=http://192.168.4.1/js '
            '-p rate_hz:=50.0 '
            '-p max_apply_s:=0.6 '
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_teleop')),
    )

    return LaunchDescription([
        start_teleop_arg,
        lidar_launch,
        imu_odom_launch,
        slam,
        lifecycle_manager,
        robot_state_pub,
        rviz,
        teleop,
    ])