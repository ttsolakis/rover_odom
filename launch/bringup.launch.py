# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue   # <-- added
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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
            'rpy_is_deg': 'true',
            'gyro_is_deg': 'true',
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

    return LaunchDescription([
        lidar_launch,
        imu_odom_launch,
        slam,
        lifecycle_manager,
        robot_state_pub,
        rviz,
    ])