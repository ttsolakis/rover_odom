from setuptools import setup

package_name = 'rover_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_to_odom.launch.py', 'launch/bringup.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/rover_mapping.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/rover.urdf.xacro']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tasos',
    maintainer_email='you@example.com',
    description='Minimal node converting IMU JSON to nav_msgs/Odometry.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_json_to_odom = rover_odom.imu_json_to_odom:main',
        ],
    },
)
