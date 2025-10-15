# launch/imu_to_odom.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    args = [
        # Device / request
        DeclareLaunchArgument('http_url',        default_value='http://192.168.4.1/js'),
        DeclareLaunchArgument('request_mode',    default_value='param'),
        DeclareLaunchArgument('poll_json',       default_value='{"T":126}'),
        DeclareLaunchArgument('enable_once_json',default_value=''),

        # Units / behavior
        DeclareLaunchArgument('rpy_is_deg',           default_value='true'),
        DeclareLaunchArgument('gyro_is_deg',          default_value='true'),
        DeclareLaunchArgument('accel_is_mg',          default_value='true'),
        DeclareLaunchArgument('auto_level',           default_value='true'),
        DeclareLaunchArgument('raw_accel_debug_mode', default_value='false'),

        # Calibration / filtering
        DeclareLaunchArgument('level_samples',           default_value='100'),
        DeclareLaunchArgument('level_gyro_thresh',       default_value='0.1'),
        DeclareLaunchArgument('level_accel_thresh',      default_value='0.2'),
        DeclareLaunchArgument('bias_samples',            default_value='100'),
        DeclareLaunchArgument('zupt_gyro_thresh',        default_value='0.1'),
        DeclareLaunchArgument('zupt_accel_thresh',       default_value='0.2'),
        DeclareLaunchArgument('filter_alpha',            default_value='0.0'),
        DeclareLaunchArgument('velocity_damping_lambda', default_value='0.05'),

        # Frames / topics / rate
        DeclareLaunchArgument('publish_tf',      default_value='false'),
        DeclareLaunchArgument('odom_frame',      default_value='odom'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_frame',       default_value='imu_link'),
        DeclareLaunchArgument('topic_name',      default_value='/imu_odom'),
        DeclareLaunchArgument('rate_hz',         default_value='50.0'),
        DeclareLaunchArgument('timeout_s',       default_value='1.0'),

        # Mounting (R,P,Y in degrees) + whether to apply
        DeclareLaunchArgument('mount_roll_deg',            default_value='0.0'),
        DeclareLaunchArgument('mount_pitch_deg',           default_value='0.0'),
        DeclareLaunchArgument('mount_yaw_deg',             default_value='180.0'),
        DeclareLaunchArgument('apply_mounting_tf_in_odom', default_value='true'),
    ]

    imu_node = Node(
        package='rover_odom',
        executable='imu_reader',
        name='imu_reader',
        output='screen',
        parameters=[{
            # Device / request (force strings)
            'http_url':         ParameterValue(LaunchConfiguration('http_url'),         value_type=str),
            'request_mode':     ParameterValue(LaunchConfiguration('request_mode'),     value_type=str),
            'poll_json':        ParameterValue(LaunchConfiguration('poll_json'),        value_type=str),
            'enable_once_json': ParameterValue(LaunchConfiguration('enable_once_json'), value_type=str),

            # Units / behavior
            'rpy_is_deg':           ParameterValue(LaunchConfiguration('rpy_is_deg'),           value_type=bool),
            'gyro_is_deg':          ParameterValue(LaunchConfiguration('gyro_is_deg'),          value_type=bool),
            'accel_is_mg':          ParameterValue(LaunchConfiguration('accel_is_mg'),          value_type=bool),
            'auto_level':           ParameterValue(LaunchConfiguration('auto_level'),           value_type=bool),
            'raw_accel_debug_mode': ParameterValue(LaunchConfiguration('raw_accel_debug_mode'), value_type=bool),

            # Calibration / filtering
            'level_samples':           ParameterValue(LaunchConfiguration('level_samples'),           value_type=int),
            'level_gyro_thresh':       ParameterValue(LaunchConfiguration('level_gyro_thresh'),       value_type=float),
            'level_accel_thresh':      ParameterValue(LaunchConfiguration('level_accel_thresh'),      value_type=float),
            'bias_samples':            ParameterValue(LaunchConfiguration('bias_samples'),            value_type=int),
            'zupt_gyro_thresh':        ParameterValue(LaunchConfiguration('zupt_gyro_thresh'),        value_type=float),
            'zupt_accel_thresh':       ParameterValue(LaunchConfiguration('zupt_accel_thresh'),       value_type=float),
            'filter_alpha':            ParameterValue(LaunchConfiguration('filter_alpha'),            value_type=float),
            'velocity_damping_lambda': ParameterValue(LaunchConfiguration('velocity_damping_lambda'), value_type=float),

            # Frames / topics / rate (strings/floats/bools typed)
            'publish_tf':      ParameterValue(LaunchConfiguration('publish_tf'),      value_type=bool),
            'odom_frame':      ParameterValue(LaunchConfiguration('odom_frame'),      value_type=str),
            'base_link_frame': ParameterValue(LaunchConfiguration('base_link_frame'), value_type=str),
            'topic_name':      ParameterValue(LaunchConfiguration('topic_name'),      value_type=str),
            'rate_hz':         ParameterValue(LaunchConfiguration('rate_hz'),         value_type=float),
            'timeout_s':       ParameterValue(LaunchConfiguration('timeout_s'),       value_type=float),

            # Mounting (scalars)
            'mount_roll_deg':            ParameterValue(LaunchConfiguration('mount_roll_deg'),            value_type=float),
            'mount_pitch_deg':           ParameterValue(LaunchConfiguration('mount_pitch_deg'),           value_type=float),
            'mount_yaw_deg':             ParameterValue(LaunchConfiguration('mount_yaw_deg'),             value_type=float),
            'apply_mounting_tf_in_odom': ParameterValue(LaunchConfiguration('apply_mounting_tf_in_odom'), value_type=bool),
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '0','0','0',
            '0','0','0','1',
            LaunchConfiguration('base_link_frame'),
            LaunchConfiguration('imu_frame'),
        ]
    )

    return LaunchDescription(args + [imu_node, static_tf])