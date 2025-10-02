from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Odometry node (request-per-poll)
        Node(
            package='rover_odom',
            executable='imu_json_to_odom',
            name='imu_json_to_odom',
            output='screen',
            parameters=[{
                # Device endpoint
                'http_url': 'http://192.168.4.1/js',
                # Request-per-poll mode to always get full IMU (T=126)
                'request_mode': 'param',    # 'param' sends ?json=... each tick
                'poll_json': '{"T":126}',   # returns r,p,y + ax,ay,az + gx,gy,gz
                # Optional: already enabled streaming; not needed in param mode
                'enable_once_json': '',
                # Units based on sample:
                # r,p,y look like degrees (≈ -151.9)  → True
                # gyro values look like deg/s (small numbers) → True
                'rpy_is_deg': True,
                'gyro_is_deg': True,
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'topic_name': '/odom',
                'rate_hz': 20.0,
                'timeout_s': 1.0,

                # Apply mounting rotation in code, consistent with TF
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
