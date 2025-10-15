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
  1-5 : speed levels (1=slow, 5=fast)

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
        self.declare_parameter('rate_hz', 50.0)            # resend/publish rate
        self.declare_parameter('timeout_s', 0.5)           # HTTP request timeout: if an HTTP call takes longer than this, it errors and is ignored.
        self.declare_parameter('send_http', True)          # set False to test without rover

        # Command paramters (device units, spec ~ ±0.5)
        self.declare_parameter('max_unit', 0.5)           # clamp max command to this         
        self.declare_parameter('max_apply_s', 1.0)        # how long a pressed command is applied: after this send zeros

        # --- Read params ---
        self.http_url     = self.get_parameter('http_url').value
        self.rate_hz      = float(self.get_parameter('rate_hz').value)
        self.timeout_s    = float(self.get_parameter('timeout_s').value)
        self.send_http    = bool(self.get_parameter('send_http').value)
        self.max_unit     = float(self.get_parameter('max_unit').value)
        self.max_apply_s  = float(self.get_parameter('max_apply_s').value)

        # --- Fixed base magnitudes (used only to define ratios) ---
        self._base_f = 0.5
        self._base_r = 0.5
        self._base_t = 0.5

        # Preserve proportions among F/R/T and scale by selected level
        _max_base = max(abs(self._base_f), abs(self._base_r), abs(self._base_t), 1e-6)
        self._ratio_f = self._base_f / _max_base
        self._ratio_r = self._base_r / _max_base
        self._ratio_t = self._base_t / _max_base

        # Number keys pick absolute magnitudes (0.1 .. 0.5)
        self._levels_abs = {'1': 0.1, '2': 0.2, '3': 0.3, '4': 0.4, '5': 0.5}
        self._current_level = '3'  # default

        # Active units (initialized by level)
        self.f_unit = 0.0
        self.r_unit = 0.0
        self.t_unit = 0.0
        self._apply_level(self._current_level)

        # --- Empirical cmd→ω fits (from your experiments) ---
        # Straight line (both wheels same sign)
        self.m_pos = 42.6750; self.b_pos = -2.3062   # cmd > 0
        self.m_neg = 41.4667; self.b_neg = +2.2750   # cmd < 0

        # Spin-in-place (opposite signs, use magnitude fit then assign signs)
        self.m_left  = 25.1242; self.b_left  = -9.2430   # left turn: (L<0, R>0)
        self.m_right = -24.2625; self.b_right = +8.5188  # right turn: (L>0, R<0)

        # --- Publishers ---
        self.pub_wheel_cmd  = self.create_publisher(Vector3Stamped, '/wheel_cmd', 10)

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

    # ========== Apply level of cmd ==========
    def _apply_level(self, key: str):
        """Set units so that the largest of (f,r,t) equals the chosen absolute level, preserving ratios."""
        L = self._levels_abs[key]  # absolute target in [0, 0.5]
        # preserve F/R/T proportions from the base units
        f = L * self._ratio_f
        r = L * self._ratio_r
        t = L * self._ratio_t

        # clamp to max_unit (symmetric)
        self.f_unit = clamp(f, -self.max_unit, self.max_unit)
        self.r_unit = clamp(r, -self.max_unit, self.max_unit)
        self.t_unit = clamp(t, -self.max_unit, self.max_unit)

        self._current_level = key
        self.get_logger().info(
            f"[level {key}] target={L:.3f} → forward={self.f_unit:.3f}, reverse={self.r_unit:.3f}, turn={self.t_unit:.3f}"
        )

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

    def _straight_map(self, u: float) -> float:
        """Per-wheel ω for straight motion (cmd u for that wheel)."""
        if u > 0.0:
            return self.m_pos * u + self.b_pos
        elif u < 0.0:
            return self.m_neg * u + self.b_neg
        else:
            return 0.0

    def _cmd_to_wheel_omega(self, L: float, R: float) -> tuple[float, float]:
        """
        Map (L,R) command units → (ω_L, ω_R) in rad/s using piecewise linear fits.
        - Straight (same sign): use straight-line fits per wheel/sign.
        - Spin (opposite sign): use spin fits by magnitude, then apply wheel signs.
        """
        eps = 1e-6
        if abs(L) < eps and abs(R) < eps:
            return 0.0, 0.0

        # Same sign → straight (forward/back)
        if L * R >= 0.0:
            return self._straight_map(L), self._straight_map(R)

        # Opposite signs → spin-in-place
        cmd_mag = max(abs(L), abs(R))

        # Left turn: L<0, R>0
        if (L < 0.0 and R > 0.0):
            mag = self.m_left * cmd_mag + self.b_left
            mag = max(0.0, abs(mag))  # ensure non-negative magnitude
            return -mag, +mag

        # Right turn: L>0, R<0
        mag = self.m_right * cmd_mag + self.b_right
        mag = max(0.0, abs(mag))
        return +mag, -mag

    def _publish_ros(self, L, R, duration_s):
        # Convert commands → wheel speeds (rad/s)
        omega_L, omega_R = self._cmd_to_wheel_omega(L, R)

        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg()
        v.vector.x = float(omega_L)    # rad/s
        v.vector.y = float(omega_R)    # rad/s
        v.vector.z = float(duration_s) # since last change
        self.pub_wheel_cmd.publish(v)  # in rad/s

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
                elif ch in ('1','2','3','4','5'):
                    self._apply_level(ch)
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
