# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Toggle starting teleop (needs a real terminal/TTY)
    start_teleop_arg = DeclareLaunchArgument(
        'start_teleop', default_value='true',
        description='Start rover_teleop in a separate terminal (WASD).'
    )

    start_logger_arg = DeclareLaunchArgument(
        'start_logger', default_value='true',
        description='Open a separate terminal for logger_cmd_vel_yaw.'
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
            'raw_accel_debug_mode': 'false',

            'level_samples': '100',
            'level_gyro_thresh': '0.1',
            'level_accel_thresh': '0.2',
            'bias_samples': '100',
            'zupt_gyro_thresh': '0.1',
            'zupt_accel_thresh': '0.2',
            'filter_alpha': '0.0',
            'velocity_damping_lambda': '0.05',
            
            'publish_tf': 'false',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'topic_name': '/imu_odom',
            'timeout_s': '1.0',
            'apply_mounting_tf_in_odom': 'true',
        }.items()
    )

    # 3) slam_toolbox
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

    # 4) Lifecycle manager to auto-configure/activate slam_toolbox
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

    # 5) Robot State Publisher for the URDF
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

    # 6) RViz
    rviz_config = os.path.join(get_package_share_directory('rover_odom'), 'rviz', 'rover_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 7) Teleop (WASD) in a separate terminal so it can read keys
    #    Requires a terminal emulator; change to xterm/konsole/kitty if you prefer.
    teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--wait', '--', 'bash', '-lc',
            'source ~/slam_ws/install/setup.bash && '
            'ros2 run rover_odom rover_teleop '
            '--ros-args '
            '-p http_url:=http://192.168.4.1/js '
            '-p rate_hz:=50.0 '
            '-p max_apply_s:=1.0 '
            ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_teleop')),
    )

    # 8) EKF: Fuses /imu_odom and /wheel_cmd to give improved /ekf_odom 
    ekf = Node(
        package='rover_odom',
        executable='ekf_odom',
        name='ekf_odom',
        output='screen',
        parameters=[{
            'imu_odom_topic': '/imu_odom',
            'wheel_cmd_topic': '/wheel_cmd',
            'ekf_odom': '/ekf_odom',
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'log_every_n': 20,
            'cmd_buffer_size': 500,
        }],
    )

    # 9) Data Logger to identify omega=omega(cmd)
    logger_term = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--wait', '--', 'bash', '-lc',
            'source ~/slam_ws/install/setup.bash && '
            'printf "L_cmd\tR_cmd\twz_mean_twist\n"; '
            'ros2 run rover_odom logger_cmd_vel_yaw '
            '--ros-args '
            '-p imu_topic:=/imu_odom '
            '-p cmd_topic:=/wheel_cmd'
            ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_logger')),
    )


    # 10) Data Logger to identify velocities from IMU accelerations
    tools_dir = os.path.expanduser('~/slam_ws/src/rover_odom/tools/velocity_from_imu_identification')
    logger_imu_cmd_vel = Node(
        package='rover_odom',
        executable='logger_imu_cmd_vel',
        name='logger_imu_cmd_vel',
        output='screen',
        parameters=[{
            'imu_topic': '/imu_odom',
            'cmd_topic': '/wheel_cmd',
            'output_topic': '/imu_cmd_aligned',
            'queue_size': 200,
            'sync_slop_s': 0.03,
            'write_csv': True,
            'csv_dir': tools_dir,      
            'csv_basename': 'imu_cmd_aligned',
            }]
    )


    return LaunchDescription([
        start_teleop_arg,
        start_logger_arg,
        lidar_launch,
        imu_odom_launch,
        slam,
        lifecycle_manager,
        robot_state_pub,
        rviz,
        teleop,
        ekf,
        logger_term,
        logger_imu_cmd_vel,
    ])
