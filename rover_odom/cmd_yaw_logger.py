#!/usr/bin/env python3
import math
from typing import Optional, Tuple
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

class CmdYawLogger(Node):
    def __init__(self):
        super().__init__('cmd_yaw_logger')

        # Params
        self.declare_parameter('imu_topic', '/imu_odom')
        self.declare_parameter('cmd_topic', '/wheel_cmd_units_stamped')
        self.declare_parameter('change_epsilon', 1e-3)   # detect L/R change
        self.declare_parameter('min_duration', 0.15)     # ignore too-short segments

        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.eps = float(self.get_parameter('change_epsilon').value)
        self.min_dur = float(self.get_parameter('min_duration').value)

        # Subs
        self.create_subscription(Vector3Stamped, self.cmd_topic, self._on_cmd, 10)
        self.create_subscription(Odometry, self.imu_topic, self._on_imu, 50)

        # Segment state
        self._active: bool = False
        self._cmd: Tuple[float, float] = (0.0, 0.0)
        self._start_t_ros: Optional[Time] = None
        self._t_start_sec: Optional[float] = None
        self._t_last_sec: Optional[float] = None
        self._sum_wz: float = 0.0
        self._sum_wz2: float = 0.0
        self._n: int = 0

        # Header for easy copy-paste
        print("t_start_iso\tL_cmd\tR_cmd\tcmd_mag\tcmd_sign\tduration_s\twz_mean\twz_std\tn_samples", flush=True)

    @staticmethod
    def _neq(a: float, b: float, eps: float) -> bool:
        return abs(a - b) > eps

    @staticmethod
    def _sign(x: float, eps: float = 1e-9) -> int:
        return 1 if x > eps else (-1 if x < -eps else 0)

    def _reset_accums(self):
        self._t_start_sec = None
        self._t_last_sec = None
        self._sum_wz = 0.0
        self._sum_wz2 = 0.0
        self._n = 0

    def _start_segment(self, t_ros: Time, L: float, R: float):
        self._active = True
        self._cmd = (float(L), float(R))
        self._start_t_ros = t_ros
        self._reset_accums()

    def _finish_segment(self, t_ros: Time):
        self._active = False
        if self._t_start_sec is None or self._t_last_sec is None or self._n == 0:
            return

        duration = max(0.0, self._t_last_sec - self._t_start_sec)
        if duration < self.min_dur:
            return

        wz_mean = self._sum_wz / self._n
        var = max(0.0, (self._sum_wz2 / self._n) - (wz_mean ** 2))
        wz_std = math.sqrt(var)

        L, R = self._cmd
        cmd_mag = 0.5 * (abs(L) + abs(R))
        cmd_sign = self._sign(R - L)  # +1 ~ CCW(A), -1 ~ CW(D), 0 ~ straight/zero

        # Start time (UTC ISO) from the command stamp
        start_sec = (self._start_t_ros.nanoseconds * 1e-9) if self._start_t_ros else 0.0
        t_iso = datetime.fromtimestamp(start_sec, tz=timezone.utc).isoformat()

        print(
            f"{t_iso}\t{L:+.3f}\t{R:+.3f}\t{cmd_mag:.3f}\t{cmd_sign:+d}\t"
            f"{duration:.3f}\t{wz_mean:+.6f}\t{wz_std:.6f}\t{self._n}",
            flush=True
        )

        self._reset_accums()

    # --- callbacks ---
    def _on_cmd(self, msg: Vector3Stamped):
        L, R = float(msg.vector.x), float(msg.vector.y)
        t_ros = Time.from_msg(msg.header.stamp)

        if not self._active:
            if self._neq(L, 0.0, self.eps) or self._neq(R, 0.0, self.eps):
                self._start_segment(t_ros, L, R)
            return

        # command changed (including to zero) → close current, maybe start new
        if self._neq(L, self._cmd[0], self.eps) or self._neq(R, self._cmd[1], self.eps):
            self._finish_segment(t_ros)
            if self._neq(L, 0.0, self.eps) or self._neq(R, 0.0, self.eps):
                self._start_segment(t_ros, L, R)

    def _on_imu(self, msg: Odometry):
        if not self._active:
            return

        t_ros = Time.from_msg(msg.header.stamp)
        t = t_ros.nanoseconds * 1e-9
        wz = float(msg.twist.twist.angular.z)

        if self._t_start_sec is None:
            self._t_start_sec = t

        self._sum_wz += wz
        self._sum_wz2 += wz * wz
        self._n += 1
        self._t_last_sec = t

    def destroy_node(self):
        try:
            if self._active:
                self._finish_segment(self.get_clock().now())
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdYawLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
