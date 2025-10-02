# rover_odom

ROS 2 (Jazzy) package that polls IMU data over HTTP (JSON `T=126`) and publishes `nav_msgs/Odometry` and TF. Includes a bringup launch that starts LiDAR, `slam_toolbox` (auto-lifecycle), `robot_state_publisher` (URDF), and RViz with a ready config.

> Roadmap: add wheel-speed inputs and fuse with IMU for improved odometry.

## Features
- IMU JSON → `/odom` (`nav_msgs/Odometry`) with proper frame handling
- Optional mounting rotation compensation
- TF: `odom → base_link` (dynamic), URDF for fixed sensor frames
- Bringup: LiDAR + odom + slam_toolbox + RViz
- RViz config and minimal URDF included

## Launch
```bash
ros2 launch rover_odom bringup.launch.py

