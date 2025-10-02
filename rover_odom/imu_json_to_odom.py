#!/usr/bin/env python3
import math
import json
import urllib.request
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def euler_zyx_to_quat(roll: float, pitch: float, yaw: float):
    """
    Convert ZYX (yaw-pitch-roll) Euler angles [rad] to quaternion (x,y,z,w).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class ImuJsonToOdom(Node):
    """
    Minimal node that polls an HTTP JSON endpoint with keys:
      r,p,y (deg or rad), ax,ay,az (m/s^2), gx,gy,gz (rad/s or deg/s)
    and publishes nav_msgs/Odometry on /odom. Also broadcasts odom->base_link TF if enabled.

    Parameters (can be set in launch):
      http_url (string): e.g., "http://192.168.4.1/js"
      rpy_is_deg (bool): default True
      gyro_is_deg (bool): default False
      publish_tf (bool): default True
      odom_frame (string): default "odom"
      base_link_frame (string): default "base_link"
      topic_name (string): default "/odom"
      rate_hz (double): default 50.0
      timeout_s (double): HTTP timeout seconds (default 1.0)
    """

    def __init__(self):
        super().__init__('imu_json_to_odom')

        # Declare parameters
        self.declare_parameter('http_url', 'http://192.168.4.1/js')
        self.declare_parameter('rpy_is_deg', True)
        self.declare_parameter('gyro_is_deg', False)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('topic_name', '/odom')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('timeout_s', 1.0)

        # Read parameters
        self.http_url = self.get_parameter('http_url').get_parameter_value().string_value
        self.rpy_is_deg = self.get_parameter('rpy_is_deg').get_parameter_value().bool_value
        self.gyro_is_deg = self.get_parameter('gyro_is_deg').get_parameter_value().bool_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        # Publishers / TF
        self.odom_pub = self.create_publisher(Odometry, self.topic_name, 10)
        self.tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        self.get_logger().info(
            f'rover_odom: polling {self.http_url} at {self.rate_hz:.1f} Hz → publishing {self.topic_name}'
        )

        # Timer for polling
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self._timer_cb)

    def _fetch_json(self) -> Optional[dict]:
        try:
            with urllib.request.urlopen(self.http_url, timeout=self.timeout_s) as resp:
                data = resp.read().decode('utf-8', errors='ignore')
                return json.loads(data)
        except Exception as e:
            self.get_logger().warn(f'HTTP fetch/parse error: {e}')
            return None

    def _timer_cb(self):
        js = self._fetch_json()
        if not js:
            return

        required = ['r', 'p', 'y', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
        if not all(k in js for k in required):
            self.get_logger().warn(f'Missing keys in JSON. Got: {list(js.keys())}')
            return

        try:
            r = float(js['r'])
            p = float(js['p'])
            y = float(js['y'])
            if self.rpy_is_deg:
                r = math.radians(r)
                p = math.radians(p)
                y = math.radians(y)

            gx = float(js['gx'])
            gy = float(js['gy'])
            gz = float(js['gz'])
            if self.gyro_is_deg:
                gx = math.radians(gx)
                gy = math.radians(gy)
                gz = math.radians(gz)

            # Accels parsed but not used for pose here (kept for future fusion)
            # ax = float(js['ax']); ay = float(js['ay']); az = float(js['az'])

            qx, qy, qz, qw = euler_zyx_to_quat(r, p, y)
        except Exception as e:
            self.get_logger().warn(f'Conversion error: {e}')
            return

        now = self.get_clock().now().to_msg()

        # Build Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link

        # Position unknown (keep 0 with very large covariance)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [
            1e6, 0,   0,   0,   0,   0,
            0,   1e6, 0,   0,   0,   0,
            0,   0,   1e6, 0,   0,   0,
            0,   0,   0,   1e-2, 0,   0,
            0,   0,   0,   0,   1e-2, 0,
            0,   0,   0,   0,   0,   1e-2
        ]

        # Velocity: no linear estimate; pass gyro as angular
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = gx
        odom.twist.twist.angular.y = gy
        odom.twist.twist.angular.z = gz
        odom.twist.covariance = [
            1e6, 0,   0,   0,   0,   0,
            0,   1e6, 0,   0,   0,   0,
            0,   0,   1e6, 0,   0,   0,
            0,   0,   0,   1e-3, 0,   0,
            0,   0,   0,   0,   1e-3, 0,
            0,   0,   0,   0,   0,   1e-3
        ]

        self.odom_pub.publish(odom)

        # Optional TF odom -> base_link
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_link
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ImuJsonToOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if _name_ == '_main_':
    main()
