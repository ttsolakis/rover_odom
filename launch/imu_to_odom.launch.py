from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # add this

def generate_launch_description():
    return LaunchDescription([
        # Declare everything you reference below
        DeclareLaunchArgument('level_samples', default_value='100'),
        DeclareLaunchArgument('level_gyro_thresh', default_value='0.1'),
        DeclareLaunchArgument('level_accel_thresh', default_value='0.2'),
        DeclareLaunchArgument('bias_samples', default_value='100'),
        DeclareLaunchArgument('zupt_gyro_thresh', default_value='0.1'),
        DeclareLaunchArgument('zupt_accel_thresh', default_value='0.2'),
        DeclareLaunchArgument('filter_alpha', default_value='0.0'),
        DeclareLaunchArgument('velocity_damping_lambda', default_value='0.05'),

        Node(
            package='rover_odom',
            executable='imu_reader',
            name='imu_reader',
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

                # Use ParameterValue to enforce correct types
                'level_samples':           ParameterValue(LaunchConfiguration('level_samples'), value_type=int),
                'level_gyro_thresh':       ParameterValue(LaunchConfiguration('level_gyro_thresh'), value_type=float),
                'level_accel_thresh':      ParameterValue(LaunchConfiguration('level_accel_thresh'), value_type=float),
                'bias_samples':            ParameterValue(LaunchConfiguration('bias_samples'), value_type=int),
                'zupt_gyro_thresh':        ParameterValue(LaunchConfiguration('zupt_gyro_thresh'), value_type=float),
                'zupt_accel_thresh':       ParameterValue(LaunchConfiguration('zupt_accel_thresh'), value_type=float),
                'filter_alpha':            ParameterValue(LaunchConfiguration('filter_alpha'), value_type=float),
                'velocity_damping_lambda': ParameterValue(LaunchConfiguration('velocity_damping_lambda'), value_type=float),

                'publish_tf': False,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'topic_name': '/imu_odom',
                'rate_hz': 50.0,
                'timeout_s': 1.0,

                # Keep this as a real list here
                'mounting_rpy_deg': [0.0, 0.0, 180.0],
                'apply_mounting_tf_in_odom': True,
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0','0','0','0','0','1','0','base_link','imu_link']
        ),
    ])
