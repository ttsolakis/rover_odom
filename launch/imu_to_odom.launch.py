from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # expose level_samples (and any others you want overridable)
        DeclareLaunchArgument('level_samples', default_value='50'),

        Node(
            package='rover_odom',
            executable='imu_json_to_odom',
            name='imu_json_to_odom',
            output='screen',
            parameters=[{
                'http_url': 'http://192.168.4.1/js',
                'request_mode': 'param',
                'poll_json': '{"T":126}',
                'enable_once_json': '',

                'rpy_is_deg': True,
                'gyro_is_deg': True,
                'accel_is_mg': True,

                'auto_level': True,
                'level_samples': LaunchConfiguration('level_samples'),  # <-- use LC here
                'level_gyro_thresh': 0.2,
                'level_g_tolerance': 2.0,

                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'topic_name': '/odom',
                'rate_hz': 50.0,
                'timeout_s': 1.0,

                'mounting_rpy_deg': [0.0, 0.0, 180.0],
                'apply_mounting_tf_in_odom': True,
            }]
        ),

        # Static transform: base_link -> imu_link (180° yaw)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                # translation (x y z)
                '0', '0', '0',
                # quaternion (qx qy qz qw) = 180° about Z
                '0', '0', '1', '0',
                # parent frame, child frame
                'base_link', 'imu_link'
            ]
        ),
    ])
