from setuptools import setup
import os
from glob import glob

package_name = 'rover_odom'

data_files = [
    # Required for ament package index
    (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
     [os.path.join('resource', package_name)]),

    # Package manifest
    (os.path.join('share', package_name), ['package.xml']),

    # Launch, RVIZ, URDF
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tasos',
    maintainer_email='you@example.com',
    description='Minimal node converting IMU JSON to nav_msgs/Odometry + tooling.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_reader      = rover_odom.imu_reader:main',
            'rover_teleop    = rover_odom.rover_teleop:main',
            'ekf_odom        = rover_odom.ekf_odom:main',
            'cmd_yaw_logger  = rover_odom.cmd_yaw_logger:main',
            'imu_cmd_logger  = rover_odom.imu_cmd_logger:main',
        ],
    },
)