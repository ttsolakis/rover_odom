#!/usr/bin/env python3
import sys
import termios
import tty
import time
import threading
from threading import Lock
import requests

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3Stamped


HELP = """
WASD control:
  W : forward     (L=+F, R=+F)
  S : reverse     (L=-F, R=-F)
  A : spin left   (L=-T, R=+T)
  D : spin right  (L=+T, R=-T)
  SPACE : STOP
  h : help
  q : quit

This node:
  • resends the last command at 'rate_hz'
  • auto-expires it after 'max_apply_s' → publishes zeros
Vector3Stamped on /wheel_cmd_units_stamped:
  x=L, y=R, z=duration_seconds_since_last_change
"""


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class RoverTeleop(Node):
    """
    WASD teleop with continuous publishing + command expiry.
    Key reader runs in a background thread; ROS executor spins the timers.
    """

    def __init__(self):
        super().__init__('rover_teleop')

        # --- Parameters ---
        self.declare_parameter('http_url', 'http://192.168.4.1/js')
        self.declare_parameter('rate_hz', 20.0)            # resend/publish rate
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('send_http', True)          # set False to test without rover

        # command magnitudes (device units, spec ~ ±0.5)
        self.declare_parameter('forward_unit', 0.2)
        self.declare_parameter('reverse_unit', 0.2)
        self.declare_parameter('turn_unit', 0.1)
        self.declare_parameter('max_unit', 0.5)

        # how long a pressed command is considered "applied"
        self.declare_parameter('max_apply_s', 0.6)         # tune to match rover hold time

        # For cmd_vel estimation (optional)
        self.declare_parameter('unit_to_mps', 0.5)         # 1.0 unit ≈ 0.5 m/s (tune later)
        self.declare_parameter('track_width', 0.20)        # meters

        # --- Read params ---
        self.http_url     = self.get_parameter('http_url').value
        self.rate_hz      = float(self.get_parameter('rate_hz').value)
        self.timeout_s    = float(self.get_parameter('timeout_s').value)
        self.send_http    = bool(self.get_parameter('send_http').value)
        self.f_unit       = float(self.get_parameter('forward_unit').value)
        self.r_unit       = float(self.get_parameter('reverse_unit').value)
        self.t_unit       = float(self.get_parameter('turn_unit').value)
        self.max_unit     = float(self.get_parameter('max_unit').value)
        self.max_apply_s  = float(self.get_parameter('max_apply_s').value)
        self.unit_to_mps  = float(self.get_parameter('unit_to_mps').value)
        self.track_width  = float(self.get_parameter('track_width').value)

        # --- Publishers ---
        self.pub_units        = self.create_publisher(Float32MultiArray, '/wheel_cmd_units', 10)
        self.pub_units_stamp  = self.create_publisher(Vector3Stamped, '/wheel_cmd_units_stamped', 10)
        self.pub_cmdvel       = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Command state (protected by lock) ---
        self._lock = Lock()
        self._L = 0.0
        self._R = 0.0
        self._last_change_steady = time.monotonic()

        # --- Timer heartbeat ---
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)

        # --- Start key reader thread ---
        self._stop_keys = threading.Event()
        self._keys_thread = threading.Thread(target=self._key_loop_thread, daemon=True)
        self._keys_thread.start()

        self.get_logger().info("WASD teleop ready. Focus terminal and use keys. Press 'h' for help.\n")
        print(HELP)

    # ========== Timer heartbeat ==========
    def _tick(self):
        with self._lock:
            elapsed = time.monotonic() - self._last_change_steady
            if elapsed >= self.max_apply_s:
                L_pub, R_pub = 0.0, 0.0
            else:
                L_pub, R_pub = self._L, self._R

        # HTTP heartbeat
        self._send_http(L_pub, R_pub)
        # ROS topics
        self._publish_ros(L_pub, R_pub, elapsed)

    # ========== I/O helpers ==========
    def _send_http(self, L, R):
        if not self.send_http:
            return
        try:
            payload = {"T": 1, "L": round(L, 3), "R": round(R, 3)}
            requests.get(self.http_url, params={"json": str(payload)}, timeout=self.timeout_s)
        except Exception as e:
            # keep low-noise; promote to warn if needed
            self.get_logger().debug(f"HTTP error: {e}")

    def _publish_ros(self, L, R, duration_s):
        msg_units = Float32MultiArray()
        msg_units.data = [float(L), float(R)]
        self.pub_units.publish(msg_units)

        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg()
        v.vector.x = float(L)
        v.vector.y = float(R)
        v.vector.z = float(duration_s)  # since last change
        self.pub_units_stamp.publish(v)

        # estimated Twist (very rough)
        vL = L * self.unit_to_mps
        vR = R * self.unit_to_mps
        vx = 0.5 * (vL + vR)
        wz = (vR - vL) / max(1e-6, self.track_width)
        tw = Twist()
        tw.linear.x  = float(vx)
        tw.angular.z = float(wz)
        self.pub_cmdvel.publish(tw)

    def _set_cmd(self, L, R):
        L = clamp(L, -self.max_unit, self.max_unit)
        R = clamp(R, -self.max_unit, self.max_unit)
        with self._lock:
            changed = (L != self._L) or (R != self._R)
            self._L, self._R = L, R
            self._last_change_steady = time.monotonic()
        if changed:
            self.get_logger().info(f"Cmd L={L:+.3f} R={R:+.3f}")

    # ========== Key loop (background thread) ==========
    def _key_loop_thread(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while not self._stop_keys.is_set():
                ch = sys.stdin.read(1)
                if ch in ('h', 'H'):
                    print(HELP)
                elif ch in ('q', 'Q'):
                    self._set_cmd(0.0, 0.0)
                    break
                elif ch == ' ':
                    self._set_cmd(0.0, 0.0)
                elif ch in ('w', 'W'):
                    self._set_cmd(+self.f_unit, +self.f_unit)
                elif ch in ('s', 'S'):
                    self._set_cmd(-self.r_unit, -self.r_unit)
                elif ch in ('a', 'A'):
                    self._set_cmd(-self.t_unit, +self.t_unit)
                elif ch in ('d', 'D'):
                    self._set_cmd(+self.t_unit, -self.t_unit)
                # else ignore
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ========== Clean shutdown ==========
    def destroy_node(self):
        # stop key thread
        self._stop_keys.set()
        try:
            self._send_http(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleop()
    try:
        # Spin the executor so timers fire while key thread runs
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
