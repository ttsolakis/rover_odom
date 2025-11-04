#!/usr/bin/env python3
import math
import json
import re
import urllib.request
import numpy as np
from urllib.parse import urlencode
from typing import Optional
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
ready_qos = QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)

def quat_to_rpy_deg(qx: float, qy: float, qz: float, qw: float):
    """
    Return (roll_deg, pitch_deg, yaw_deg) from quaternion (x,y,z,w) using ZYX (yaw-pitch-roll) convention.
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    # clamp to handle numerical drift
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll/2); sr = math.sin(roll/2)
    cp = math.cos(pitch/2); sp = math.sin(pitch/2)
    cy = math.cos(yaw/2); sy = math.sin(yaw/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)

def quat_conj(q):
    x,y,z,w = q
    return (-x, -y, -z, w)

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

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

def rotate_vec_by_quat(v, q):
    """
    Rotate 3D vector v by quaternion q=(x,y,z,w): v' = q * v * q_conj
    Assumes q rotates from imu_link to base_link when you pass self.q_imu_to_base.
    """
    vx, vy, vz = v
    qx, qy, qz, qw = q
    # t = 2 * (q_vec x v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w*t + q_vec x t
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)
    return (vpx, vpy, vpz)

def quat_normalize(q):
    x,y,z,w = q
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    return (x/n, y/n, z/n, w/n)

def quat_from_two_vectors(a, b):
    """
    Return quaternion q that rotates unit vector a -> unit vector b.
    a,b arbitrary 3D; handles near-opposite safely.
    """
    ax,ay,az = a
    bx,by,bz = b
    # normalize inputs
    an = math.sqrt(ax*ax+ay*ay+az*az) or 1.0
    bn = math.sqrt(bx*bx+by*by+bz*bz) or 1.0
    ax,ay,az = ax/an, ay/an, az/an
    bx,by,bz = bx/bn, by/bn, bz/bn

    # cross and dot
    cx = ay*bz - az*by
    cy = az*bx - ax*bz
    cz = ax*by - ay*bx
    d = ax*bx + ay*by + az*bz

    if d < -0.999999:  # 180deg: pick any orthogonal axis
        # choose axis orthogonal to a
        if abs(ax) < 0.1 and abs(ay) < 0.9:
            rx, ry, rz = 0.0, -az, ay
        else:
            rx, ry, rz = -az, 0.0, ax
        rn = math.sqrt(rx*rx+ry*ry+rz*rz) or 1.0
        rx, ry, rz = rx/rn, ry/rn, rz/rn
        return (rx*1.0, ry*1.0, rz*1.0, 0.0)  # 180deg
    # general case
    qx = cx
    qy = cy
    qz = cz
    qw = 1.0 + d
    return quat_normalize((qx, qy, qz, qw))

def causal_savitzky_golay_coeffs(length, poly_order, dt):

    # Indices of the past samples: k = 0..L-1  (0=now, 1=1 step back, ...)
    k = np.arange(length, dtype=float)

    # Times of those samples relative to "now": t_k = -k * dt (non-positive)
    t = -k * dt

    # Vandermonde-like design matrix A (L x (p+1)):
    #   row k is [1, t_k, t_k^2, ..., t_k^p]
    # We stack columns t^0, t^1, ..., t^p and then transpose to get L rows.
    A = np.vstack([t**p for p in range(poly_order + 1)]).T

    # Vector that selects the constant term a0 (value at t=0) from the polynomial coefficients
    e0 = np.zeros(poly_order + 1)
    e0[0] = 1.0

    # Compute (A^T A)^{-1} using pseudoinverse for numerical robustness
    ATA_inv = np.linalg.pinv(A.T @ A)

    # Endpoint SG weights:
    #   h = A (A^T A)^{-1} e0
    # This yields a length-L vector of FIR weights for y_hat[i] = sum h[k] * y[i-k]
    h = A @ (ATA_inv @ e0)

    return h

def filter_and_unbias_acceleration(time_imu_seconds, accel_x_mps2, window_length=15, poly_order=2, acceleration_filtering=True):
    
        # --- pass-through until we have enough samples for SG + bias learning ---
        time_imu_seconds = np.asarray(time_imu_seconds, dtype=float)
        accel_x_mps2     = np.asarray(accel_x_mps2, dtype=float)
        N = accel_x_mps2.size
        if N == 0:
            return 0.0
        if N < window_length:
            return float(accel_x_mps2[-1])
        
        # --- Savitzky-Golay filter setup ---
        # Idea: Smooth the raw acceleration to reduce noise before integration with a causal SG filter.
        # This is a low-pass FIR filter that preserves low-degree polynomials up to `poly_order`.
        # It has no phase distortion (zero phase on polynomials up to `poly_order`), so it doesn't
        # bias ramps/curvature, just smooths high-frequency noise.
        dt_nominal = float(np.median(np.diff(time_imu_seconds)))
        assert window_length >= poly_order + 1, "window_length must be >= poly_order + 1"
        savitzky_golay_weights = causal_savitzky_golay_coeffs(window_length, poly_order, dt_nominal)

        # ---- Bias learning (windowed mean while stationary) ----
        hard_zero_when_stationary = True
        stationary_abs_threshold_mps2 = 0.1
        stationary_min_consecutive_samples = 10
        bias_window_samples = stationary_min_consecutive_samples
        zupt_window_values = deque(maxlen=bias_window_samples)

        # Main loop
        bias_estimate = 0.0
        consecutive_stationary_count = 0
        bias_corrected_acceleration = float(accel_x_mps2[-1])  # fallback

        start_i = 0 if not acceleration_filtering else (window_length - 1)
        for i in range(start_i, N):

            # --- 1) Filter raw acceleratation data ---
            if acceleration_filtering:
                acceleration_buffer = accel_x_mps2[i - window_length + 1 : i + 1][::-1]  # newest→oldest
                filtered_value = float(np.dot(savitzky_golay_weights, acceleration_buffer))
            else:
                filtered_value = float(accel_x_mps2[i])

            # --- 2) Remove acceleration bias (windowed mean while stationary) ---
            # Logic:
            #  • If |filtered_value| is small, increment a “quiet” counter and push it into a small deque.
            #  • If not quiet, reset the counter and clear the deque (don’t learn from motion).
            #  • Once we’re confidently stationary (counter ≥ threshold) and the window is full,
            #    set bias_estimate = mean(window). Otherwise keep the last bias_estimate.
            #  • Subtract bias everywhere; optionally hard-zero while stationary.
            if abs(filtered_value) < stationary_abs_threshold_mps2:
                consecutive_stationary_count += 1
                zupt_window_values.append(filtered_value)
            else:
                consecutive_stationary_count = 0
                zupt_window_values.clear()

            is_stationary = (consecutive_stationary_count >= stationary_min_consecutive_samples)

            if is_stationary and len(zupt_window_values) == bias_window_samples:
                bias_estimate = float(np.mean(zupt_window_values))

            if is_stationary and hard_zero_when_stationary:
                bias_corrected_acceleration = 0.0
            else:
                bias_corrected_acceleration = filtered_value - bias_estimate

        # Return the latest sample's bias-corrected acceleration as a scalar
        return float(bias_corrected_acceleration)


def estimate_velocity_from_acceleration(time_imu_seconds, filtered_acceleration, cmd_wheel_rate_rad_s, previous_velocity_estimate_mps, wheel_radius_m, constant_velocity_threshold, zupt_speed_threshold_mps):

    # Need at least two samples everywhere we take last-two
    if len(time_imu_seconds) < 2 or len(filtered_acceleration) < 2 or len(cmd_wheel_rate_rad_s) < 2:
        return float(previous_velocity_estimate_mps)
    
    previous_time_imu_seconds, current_time_imu_seconds = float(time_imu_seconds[-2]), float(time_imu_seconds[-1])
    previous_filtered_acceleration, current_filtered_acceleration = float(filtered_acceleration[-2]), float(filtered_acceleration[-1])
    previous_cmd_wheel_rate_rad_s, current_cmd_wheel_rate_rad_s = float(cmd_wheel_rate_rad_s[-2]), float(cmd_wheel_rate_rad_s[-1])
    current_velocity_cmd_linear_mps = wheel_radius_m * current_cmd_wheel_rate_rad_s
    previous_velocity_cmd_linear_mps = wheel_radius_m * previous_cmd_wheel_rate_rad_s
    
    # --- 1) ZUPT gating ---
    near_zero_cmd = (abs(current_velocity_cmd_linear_mps) < zupt_speed_threshold_mps)
    sign_flip_cmd = (current_velocity_cmd_linear_mps * previous_velocity_cmd_linear_mps) < 0.0
    if near_zero_cmd or sign_flip_cmd:
        return 0.0

    # --- 2) CUPT gating ---
    # Only run this when the buffer is exactly the chosen window size.
    if len(cmd_wheel_rate_rad_s) == int(constant_velocity_threshold):
        v_cmd_window = wheel_radius_m * np.asarray(cmd_wheel_rate_rad_s, dtype=float)
        diffs = np.diff(v_cmd_window)
        if np.all(np.abs(diffs) <= 0.01):
            return float(current_velocity_cmd_linear_mps)

    # --- 3) Trapezoidal Integration ---
    dt = current_time_imu_seconds - previous_time_imu_seconds
    if not np.isfinite(dt) or dt <= 0.0:
        return float(previous_velocity_estimate_mps)
    
    current_velocity_estimate_mps =  previous_velocity_estimate_mps + 0.5*(current_filtered_acceleration + previous_filtered_acceleration) * dt

    return current_velocity_estimate_mps


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
      imu_topic (string): odometry topic name
      rate_hz (float): polling rate
      timeout_s (float): HTTP timeout
    """

    def __init__(self):
        super().__init__('imu_reader')

        # Declare parameters
        self.declare_parameter('http_url', 'http://192.168.4.1/js')
        self.declare_parameter('request_mode', 'plain')    # plain or param
        self.declare_parameter('poll_json', '{"T":126}')
        self.declare_parameter('enable_once_json', '')
        self.declare_parameter('rpy_is_deg', True)
        self.declare_parameter('gyro_is_deg', True)
        self.declare_parameter('accel_is_mg', True)
        self.declare_parameter('auto_level', True)
        self.declare_parameter('raw_accel_debug_mode', False)
        self.declare_parameter('level_samples', 100)
        self.declare_parameter('level_gyro_thresh', 0.1)  
        self.declare_parameter('level_accel_thresh', 0.2)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('imu_topic', '/imu_odom')
        self.declare_parameter('wheel_cmd_topic', '/wheel_cmd')  # Vector3Stamped: vector.x=Left, vector.y=Right
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('timeout_s', 1.0)
        self.declare_parameter('mount_roll_deg', 0.0)
        self.declare_parameter('mount_pitch_deg', 0.0)
        self.declare_parameter('mount_yaw_deg', 180.0)
        self.declare_parameter('apply_mounting_tf_in_odom', True)
        self.declare_parameter('accel_filtering', True)          # enable SG smoothing
        self.declare_parameter('sg_window_length', 10)           # causal window length
        self.declare_parameter('sg_poly_order', 2)               # polynomial order
        self.declare_parameter('wheel_radius_m', 0.04)
        self.declare_parameter('zupt_speed_threshold_mps', 0.02)
        self.declare_parameter('constant_velocity_threshold', 10)

        # Read parameters
        self.http_url = self.get_parameter('http_url').value
        self.request_mode = self.get_parameter('request_mode').value
        self.poll_json = self.get_parameter('poll_json').value
        self.enable_once_json = self.get_parameter('enable_once_json').value
        self.rpy_is_deg = self.get_parameter('rpy_is_deg').value
        self.gyro_is_deg = self.get_parameter('gyro_is_deg').value
        self.accel_is_mg = self.get_parameter('accel_is_mg').value
        self.auto_level = bool(self.get_parameter('auto_level').value)
        self.raw_accel_debug_mode = bool(self.get_parameter('raw_accel_debug_mode').value)
        self.level_samples = int(self.get_parameter('level_samples').value)
        self.level_gyro_thresh = float(self.get_parameter('level_gyro_thresh').value)
        self.level_accel_thresh = float(self.get_parameter('level_accel_thresh').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link = self.get_parameter('base_link_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.wheel_cmd_topic = self.get_parameter('wheel_cmd_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.mount_roll_deg = float(self.get_parameter('mount_roll_deg').value)
        self.mount_pitch_deg = float(self.get_parameter('mount_pitch_deg').value)
        self.mount_yaw_deg = float(self.get_parameter('mount_yaw_deg').value)
        self.mounting_rpy_deg = [self.mount_roll_deg, self.mount_pitch_deg, self.mount_yaw_deg] 
        self.apply_mounting_tf = bool(self.get_parameter('apply_mounting_tf_in_odom').value)
        self.accel_filtering = bool(self.get_parameter('accel_filtering').value)
        self.sg_window_length = int(self.get_parameter('sg_window_length').value)
        self.sg_poly_order = int(self.get_parameter('sg_poly_order').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.zupt_speed_threshold_mps = float(self.get_parameter('zupt_speed_threshold_mps').value)
        self.constant_velocity_threshold = int(self.get_parameter('constant_velocity_threshold').value)

        # calibration buffers/state
        self.g = 9.80665
        self._calib_count = 0
        self._acc_sum = [0.0, 0.0, 0.0]  # sum of raw accel (before any rotation) in m/s^2
        self._calib_done = (not self.auto_level)
        self.q_align = (0.0, 0.0, 0.0, 1.0)  # imu_link -> "ideal imu" (Z up) tilt correction
        self.accel_residual_thresh = 0.05  # m/s² threshold for large residual warning

        # Latest wheel command (Vector3Stamped). We use x->left, y->right by default.
        self.cmd_left = 0.0
        self.cmd_right = 0.0
        self.last_cmd_time = None  # float seconds

        # Rolling buffers used to call the offline function on a sliding window
        self._time_buf = deque(maxlen=self.sg_window_length)
        self._accel_buf = deque(maxlen=self.sg_window_length)
        self._cmd_vel_buf = deque(maxlen=self.constant_velocity_threshold)
        self._ax_filtered_buf = deque(maxlen=2)

        # Integrated velocity state
        self.vx_estimated = 0.0
        self.prev_vx_estimated = 0.0

        # Pre-compute mounting quaternion: base_link <- imu_link
        mr, mp, my = [math.radians(v) for v in self.mounting_rpy_deg]
        # static TF publishes base->imu; we need imu->base for orientation correction → inverse (conjugate)
        q_base_from_imu = rpy_to_quat(mr, mp, my)
        self.q_imu_to_base = quat_conj(q_base_from_imu)
        self.q_imu_to_base = quat_normalize(self.q_imu_to_base)

        # Publisher for IMU readiness
        self.pub_auto_level_ready = self.create_publisher(Bool, '/auto_level_ready', ready_qos)
        self._ready_published = False

        if not self.auto_level:
            self.pub_auto_level_ready.publish(Bool(data=True))
            self._ready_published = True
            self.q_align = (0.0, 0.0, 0.0, 1.0) 
            self.get_logger().info("Auto-level disabled → publishing /auto_level_ready immediately.")

        # Publisher / TF
        self.odom_pub = self.create_publisher(Odometry, self.imu_topic, 10)
        self.tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        # Subscriber to wheel commands (Vector3Stamped)
        self.wheel_sub = self.create_subscription(
            Vector3Stamped,
            self.wheel_cmd_topic,
            self._wheel_cmd_cb,
            10
        )
        self.get_logger().info(f"Subscribed to wheel commands at '{self.wheel_cmd_topic}' (Vector3Stamped: x=L, y=R)")


        # Optionally send enable_once_json to device
        if self.enable_once_json:
            try:
                url = f"{self.http_url}?{urlencode({'json': self.enable_once_json})}"
                urllib.request.urlopen(url, timeout=self.timeout_s).read()
                self.get_logger().info(f"Sent enable_once_json: {self.enable_once_json}")
            except Exception as e:
                self.get_logger().warning(f"Failed enable_once_json: {e}")

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
            self.get_logger().warning(f"HTTP fetch/parse error: {e}")
            return None
        
    def _wheel_cmd_cb(self, msg: Vector3Stamped):
        """
        Cache latest wheel command.
        Convention here: msg.vector.x = Left, msg.vector.y = Right (units as published, e.g., rad/s).
        """
        try:
            self.cmd_left = float(msg.vector.x)
            self.cmd_right = float(msg.vector.y)
            self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
        except Exception as e:
            self.get_logger().warning(f"wheel_cmd parse error: {e}")

    def _timer_cb(self):
        js = self._fetch_json()
        if not js:
            return

        required = ['r', 'p', 'y', 'gx', 'gy', 'gz', 'ax', 'ay', 'az']
        if not all(k in js for k in required):
            self.get_logger().warning(f"Missing keys: {list(js.keys())}")
            return

        try:
            r = float(js['r'])
            p = float(js['p'])
            y = float(js['y'])
            if self.rpy_is_deg:
                r = math.radians(r)  # convert deg to rad
                p = math.radians(p)  # convert deg to rad
                y = math.radians(y)  # convert deg to rad

            gx = float(js['gx'])
            gy = float(js['gy'])
            gz = float(js['gz'])
            if self.gyro_is_deg:
                gx = math.radians(gx)  # convert deg/s to rad/s
                gy = math.radians(gy)  # convert deg/s to rad/s
                gz = math.radians(gz)  # convert deg/s to rad/s

            ax = float(js['ax'])
            ay = float(js['ay'])
            az = float(js['az'])
            if self.accel_is_mg:
                ax = 9.80665e-3 * ax  # convert mg to m/s²
                ay = 9.80665e-3 * ay  # convert mg to m/s²
                az = 9.80665e-3 * az  # convert mg to m/s²

            # --- AUTO-LEVEL: estimate q_align from avg gravity vector (IMU must be stationary) ---
            if not self._calib_done and self.auto_level:
                # require small gyro magnitude to assume "still"
                if self._calib_count == 0:
                    self.get_logger().info(f"Auto-level: hold still for ~{1/(self.rate_hz)*self.level_samples} sec...")
                    
                gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
                a_norm = math.sqrt(ax*ax + ay*ay + az*az)
                is_still = (gyro_norm < self.level_gyro_thresh and abs(a_norm - self.g) < self.level_accel_thresh)

                if is_still:
                    self._acc_sum[0] += ax
                    self._acc_sum[1] += ay
                    self._acc_sum[2] += az
                    self._calib_count += 1

                if self._calib_count >= self.level_samples:
                    # average gravity direction in IMU frame
                    ax_mean = self._acc_sum[0] / self._calib_count
                    ay_mean = self._acc_sum[1] / self._calib_count
                    az_mean = self._acc_sum[2] / self._calib_count
                    # we want rotation that maps measured gravity -> +Z axis
                    # NOTE: accel measures specific force; at rest it's ~ +g along +Z in the sensor frame
                    q_tilt = quat_from_two_vectors((ax_mean, ay_mean, az_mean), (0.0, 0.0, self.g))
                    self.q_align = quat_normalize(q_tilt)
                    self._calib_done = True
                    
                if self._calib_done and not self._ready_published:
                    self.get_logger().info(f"Auto-level complete (samples={self._calib_count}).")
                    self.pub_auto_level_ready.publish(Bool(data=True))
                    self._ready_published = True
                    

            # Quaternion as reported by the IMU (in imu_link frame)
            qx, qy, qz, qw = euler_zyx_to_quat(r, p, y)

            # Compose rotation: imu_link --(q_align)--> ideal_imu --(q_imu_to_base)--> base_link
            q_total = self.q_imu_to_base
            if self._calib_done and self.auto_level:
                q_total = quat_mul(self.q_imu_to_base, self.q_align)
            q_total = quat_normalize(q_total)

            # Apply IMU mounting correction (180° yaw): base_link using imu->base mounting quaternion
            q_imu = quat_normalize((qx, qy, qz, qw))
            if self.apply_mounting_tf:      
                qx, qy, qz, qw = quat_mul(q_total, q_imu)  
                # qx, qy, qz, qw = quat_mul(q_imu, q_total)  TODO: Check which one is right!!!
                gx, gy, gz = rotate_vec_by_quat((gx, gy, gz), q_total)
                ax, ay, az = rotate_vec_by_quat((ax, ay, az), q_total)
            else:
                qx, qy, qz, qw = q_imu

            # Store raw acceleration data for debugging
            ax_raw = ax
            ay_raw = ay
            az_raw = az

            # Warning that ax has a big residual:
            if not self._calib_done and self.auto_level and abs(ax_raw) > self.accel_residual_thresh:
                 self.get_logger().warning(f"Large residual detected in ax: {ax_raw:.4f} m/s²")

            # --- INTEGRATION: get linear velocity from acceleration (simple approx, no drift correction) ---
            # --- Collect into rolling buffers ---
            t_sec_now = self.get_clock().now().nanoseconds * 1e-9  # timestamp aligned with this IMU sample
            self._time_buf.append(t_sec_now)
            self._accel_buf.append(ax)  # longitudinal accel in base_link
            self._cmd_vel_buf.append( 0.5 * (self.cmd_left + self.cmd_right) )  # average wheel command

            # --- Filter and unbias acceleration
            ax_filtered = filter_and_unbias_acceleration(self._time_buf, self._accel_buf, window_length=self.sg_window_length, poly_order=self.sg_poly_order, acceleration_filtering=self.accel_filtering)
            self._ax_filtered_buf.append(ax_filtered)

            # --- Velocity from IMU acceleration via simple integration ---
            self.vx_estimated = estimate_velocity_from_acceleration(self._time_buf, self._ax_filtered_buf, self._cmd_vel_buf, self.prev_vx_estimated, wheel_radius_m=self.wheel_radius_m, constant_velocity_threshold=self.constant_velocity_threshold, zupt_speed_threshold_mps=self.zupt_speed_threshold_mps)
            self.prev_vx_estimated = self.vx_estimated

        except Exception as e:
            self.get_logger().warning(f"Conversion error: {e}")
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

          # --- DEBUG: print RPY in degrees ---
        roll_deg, pitch_deg, yaw_deg = quat_to_rpy_deg(qx, qy, qz, qw)
        # self.get_logger().info(f"RPY_deg: roll={roll_deg:+6.2f}  pitch={pitch_deg:+6.2f}  yaw={yaw_deg:+6.2f}")

        if self.raw_accel_debug_mode:
            odom.twist.twist.linear.x = ax_raw
            odom.twist.twist.linear.y = ay_raw
            odom.twist.twist.linear.z = az_raw
        else:   
            odom.twist.twist.linear.x = self.vx_estimated
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = gx
        odom.twist.twist.angular.y = gy
        odom.twist.twist.angular.z = gz

        self.odom_pub.publish(odom)

        # if self.tf_broadcaster:
        #     t = TransformStamped()
        #     t.header.stamp = now
        #     t.header.frame_id = self.odom_frame
        #     t.child_frame_id = self.base_link
        #     t.transform.translation.x = 0.0
        #     t.transform.translation.y = 0.0
        #     t.transform.translation.z = 0.0
        #     t.transform.rotation.x = qx
        #     t.transform.rotation.y = qy
        #     t.transform.rotation.z = qz
        #     t.transform.rotation.w = qw
        #     self.tf_broadcaster.sendTransform(t)


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
    