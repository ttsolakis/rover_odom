#!/usr/bin/env python3
import math
import json
import re
import urllib.request
from urllib.parse import urlencode
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def euler_zyx_to_quat(roll: float, pitch: float, yaw: float):
    """Convert ZYX (yaw-pitch-roll) Euler angles [rad] to quaternion (x,y,z,w)."""
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
    Node that polls an HTTP JSON endpoint and publishes nav_msgs/Odometry.

    Parameters:
      http_url (string): base URL (default: http://192.168.4.1/js)
      request_mode (string): 'plain' or 'param'
      poll_json (string): JSON payload to send each poll (only in 'param' mode)
      enable_once_json (string): JSON payload sent once at startup
      rpy_is_deg (bool): whether r,p,y are in degrees (default True)
      gyro_is_deg (bool): whether gx,gy,gz are in deg/s (default False)
      publish_tf (bool): whether to also broadcast odom->base_link TF
      odom_frame (string): name of odom frame
      base_link_frame (string): name of base_link frame
      topic_name (string): odometry topic name
      rate_hz (float): polling rate
      timeout_s (float): HTTP timeout
    """

    def __init__(self):
        super().__init__('imu_json_to_odom')

        # Declare parameters
        self.declare_parameter('http_url', 'http://192.168.4.1/js')
        self.declare_parameter('request_mode', 'plain')    # plain or param
        self.declare_parameter('poll_json', '{"T":126}')
        self.declare_parameter('enable_once_json', '')
        self.declare_parameter('rpy_is_deg', True)
        self.declare_parameter('gyro_is_deg', False)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('topic_name', '/odom')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('timeout_s', 1.0)

        # Read parameters
        self.http_url = self.get_parameter('http_url').value
        self.request_mode = self.get_parameter('request_mode').value
        self.poll_json = self.get_parameter('poll_json').value
        self.enable_once_json = self.get_parameter('enable_once_json').value
        self.rpy_is_deg = self.get_parameter('rpy_is_deg').value
        self.gyro_is_deg = self.get_parameter('gyro_is_deg').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link = self.get_parameter('base_link_frame').value
        self.topic_name = self.get_parameter('topic_name').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        # Publisher / TF
        self.odom_pub = self.create_publisher(Odometry, self.topic_name, 10)
        self.tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        # Optionally send enable_once_json to device
        if self.enable_once_json:
            try:
                url = f"{self.http_url}?{urlencode({'json': self.enable_once_json})}"
                urllib.request.urlopen(url, timeout=self.timeout_s).read()
                self.get_logger().info(f"Sent enable_once_json: {self.enable_once_json}")
            except Exception as e:
                self.get_logger().warn(f"Failed enable_once_json: {e}")

        self.get_logger().info(
            f"Polling {self.http_url} at {self.rate_hz:.1f} Hz (mode={self.request_mode})"
        )

        # Poll timer
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self._timer_cb)

    def _fetch_json(self) -> Optional[dict]:
        try:
            if self.request_mode == 'param':
                url = f"{self.http_url}?{urlencode({'json': self.poll_json})}"
            else:
                url = self.http_url

            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                data = resp.read().decode('utf-8', errors='ignore')
                try:
                    return json.loads(data)
                except json.JSONDecodeError:
                    # Fix unquoted keys {r:1} → {"r":1}
                    data_fixed = re.sub(r'([{\[,]\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:', r'\1"\2":', data)
                    return json.loads(data_fixed)
        except Exception as e:
            self.get_logger().warn(f"HTTP fetch/parse error: {e}")
            return None

    def _timer_cb(self):
        js = self._fetch_json()
        if not js:
            return

        required = ['r', 'p', 'y', 'gx', 'gy', 'gz']
        if not all(k in js for k in required):
            self.get_logger().warn(f"Missing keys: {list(js.keys())}")
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

            qx, qy, qz, qw = euler_zyx_to_quat(r, p, y)
        except Exception as e:
            self.get_logger().warn(f"Conversion error: {e}")
            return

        now = self.get_clock().now().to_msg()

        # Build Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.angular.x = gx
        odom.twist.twist.angular.y = gy
        odom.twist.twist.angular.z = gz

        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
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


if __name__ == '__main__':
    main()
    