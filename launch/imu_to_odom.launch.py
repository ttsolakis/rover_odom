from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_odom',
            executable='imu_json_to_odom',
            name='imu_json_to_odom',
            output='screen',
            parameters=[{
                # Your device endpoint
                'http_url': 'http://192.168.4.1/js',

                # Request-per-poll mode to always get full IMU (T=126)
                'request_mode': 'param',          # 'param' sends ?json=... each tick
                'poll_json': '{"T":126}',         # returns r,p,y + ax,ay,az + gx,gy,gz

                # Optional: you already enabled streaming; not needed in param mode
                'enable_once_json': '',

                # Units based on your sample:
                # r,p,y look like degrees (≈ -151.9)  → True
                # gyro values look like deg/s (small numbers) → True
                'rpy_is_deg': True,
                'gyro_is_deg': True,

                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'topic_name': '/odom',
                'rate_hz': 50.0,
                'timeout_s': 1.0
            }]
        )
    ])