#!/usr/bin/env python3
import math
from collections import deque
from typing import Deque, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rad) from quaternion (x,y,z,w) using ZYX convention."""
    # yaw (z-axis rotation)
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)


class EkfOdomNode(Node):
    """
    STEP 1: Subscribe to IMU odometry (/odom for now) and wheel commands
    (/wheel_cmd_units_stamped). No filtering yet, just validate data flow.

    Parameters:
      imu_odom_topic (string): default '/odom' (later '/imu_odom')
      wheel_cmd_topic (string): default '/wheel_cmd_units_stamped'
      log_every_n (int): print every N received IMU odom msgs
      cmd_buffer_size (int): ring buffer length for wheel commands (for later sync)
    """

    def __init__(self):
        super().__init__('ekf_odom')

        # --- Parameters ---
        self.declare_parameter('imu_odom_topic', '/odom')  # will rename to /imu_odom later
        self.declare_parameter('wheel_cmd_topic', '/wheel_cmd_units_stamped')
        self.declare_parameter('log_every_n', 20)
        self.declare_parameter('cmd_buffer_size', 500)

        self.imu_odom_topic: str = self.get_parameter('imu_odom_topic').value
        self.wheel_cmd_topic: str = self.get_parameter('wheel_cmd_topic').value
        self.log_every_n: int = int(self.get_parameter('log_every_n').value)
        self.cmd_buffer_size: int = int(self.get_parameter('cmd_buffer_size').value)

        # --- QoS (sensor-like for odom, reliable for cmds is fine) ---
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscriptions ---
        self.sub_imu_odom = self.create_subscription(
            Odometry, self.imu_odom_topic, self._cb_imu_odom, odom_qos
        )
        self.sub_wheel_cmd = self.create_subscription(
            Vector3Stamped, self.wheel_cmd_topic, self._cb_wheel_cmd, cmd_qos
        )

        # --- State for validation/logging ---
        self._imu_count = 0
        self._last_imu_time: Optional[Time] = None
        self._last_cmd_time: Optional[Time] = None

        # Store last messages (for later EKF use)
        self.last_imu_odom: Optional[Odometry] = None
        # Ring buffer of (stamp_sec, L, R, duration_s)
        self.cmd_buf: Deque[Tuple[float, float, float, float]] = deque(maxlen=self.cmd_buffer_size)

        self.get_logger().info(
            f"EKF step-1 node ready. Subscribing to:\n"
            f"  • IMU odom: {self.imu_odom_topic}\n"
            f"  • Wheel cmds: {self.wheel_cmd_topic}"
        )

    # ========= Callbacks =========
    def _cb_imu_odom(self, msg: Odometry):
        self._imu_count += 1
        self.last_imu_odom = msg

        t = Time.from_msg(msg.header.stamp)
        dt = float('nan') if self._last_imu_time is None else (t - self._last_imu_time).nanoseconds * 1e-9
        self._last_imu_time = t

        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        yaw = yaw_from_quat(qx, qy, qz, qw)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        u_mag = math.hypot(vx, vy)

        if self._imu_count % max(1, self.log_every_n) == 0:
            self.get_logger().info(
                f"[IMU ODOM] t={t.nanoseconds*1e-9:.3f}s  dt={0.0 if math.isnan(dt) else dt:.3f}s | "
                f"yaw={yaw:+.3f} rad | v=({vx:+.3f},{vy:+.3f}) m/s | |v|={u_mag:.3f} | r={wz:+.3f} rad/s"
            )

    def _cb_wheel_cmd(self, msg: Vector3Stamped):
        t = Time.from_msg(msg.header.stamp)
        dt = float('nan') if self._last_cmd_time is None else (t - self._last_cmd_time).nanoseconds * 1e-9
        self._last_cmd_time = t

        L = float(msg.vector.x)
        R = float(msg.vector.y)
        dur = float(msg.vector.z)  # “duration since last change” from teleop

        self.cmd_buf.append((t.nanoseconds * 1e-9, L, R, dur))

        # Light logging (only when we also log IMU to avoid spam)
        if self._imu_count % max(1, self.log_every_n) == 0:
            self.get_logger().info(
                f"[WHEEL CMD] t={t.nanoseconds*1e-9:.3f}s  dt={0.0 if math.isnan(dt) else dt:.3f}s | "
                f"L={L:+.3f}  R={R:+.3f}  (dur_since_change={dur:.2f}s)  buf={len(self.cmd_buf)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = EkfOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
