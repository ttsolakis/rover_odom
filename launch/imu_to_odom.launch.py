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
                'http_url': 'http://192.168.4.1/js',
                'rpy_is_deg': True,
                'gyro_is_deg': False,
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'topic_name': '/odom',
                'rate_hz': 50.0,
                'timeout_s': 1.0
            }]
        )
    ])
