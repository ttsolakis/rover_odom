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
        self.declare_parameter('level_samples', 50)
        self.declare_parameter('level_gyro_thresh', 0.1)  
        self.declare_parameter('level_accel_thresh', 0.2)
        self.declare_parameter('bias_samples', 50)
        self.declare_parameter('zupt_gyro_thresh', 0.1)
        self.declare_parameter('zupt_accel_thresh', 0.2)
        self.declare_parameter('filter_alpha', 0.1) 
        self.declare_parameter('velocity_damping_lambda', 0.05) 
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('topic_name', '/imu_odom')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('timeout_s', 1.0)
        self.declare_parameter('mounting_rpy_deg', [0.0, 0.0, 180.0])
        self.declare_parameter('apply_mounting_tf_in_odom', True)

        # Read parameters
        self.http_url = self.get_parameter('http_url').value
        self.request_mode = self.get_parameter('request_mode').value
        self.poll_json = self.get_parameter('poll_json').value
        self.enable_once_json = self.get_parameter('enable_once_json').value
        self.rpy_is_deg = self.get_parameter('rpy_is_deg').value
        self.gyro_is_deg = self.get_parameter('gyro_is_deg').value
        self.accel_is_mg = self.get_parameter('accel_is_mg').value
        self.auto_level = bool(self.get_parameter('auto_level').value)
        self.level_samples = int(self.get_parameter('level_samples').value)
        self.level_gyro_thresh = float(self.get_parameter('level_gyro_thresh').value)
        self.level_accel_thresh = float(self.get_parameter('level_accel_thresh').value)
        self.bias_samples = int(self.get_parameter('bias_samples').value)
        self.zupt_gyro_thresh = float(self.get_parameter('zupt_gyro_thresh').value)
        self.zupt_accel_thresh = float(self.get_parameter('zupt_accel_thresh').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.velocity_damping_lambda = float(self.get_parameter('velocity_damping_lambda').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link = self.get_parameter('base_link_frame').value
        self.topic_name = self.get_parameter('topic_name').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.mounting_rpy_deg = list(self.get_parameter('mounting_rpy_deg').value)
        self.apply_mounting_tf = bool(self.get_parameter('apply_mounting_tf_in_odom').value)

        # calibration buffers/state
        self.g = 9.80665
        self._calib_count = 0
        self._acc_sum = [0.0, 0.0, 0.0]  # sum of raw accel (before any rotation) in m/s^2
        self._calib_done = (not self.auto_level)
        self.q_align = (0.0, 0.0, 0.0, 1.0)  # imu_link -> "ideal imu" (Z up) tilt correction

        # Integration state (planar)
        self.prev_time_sec = None
        self.vx = 0.0
        self.vy = 0.0
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.ax_filtered = 0.0
        self.ay_filtered = 0.0
        self._bias_ax_sum = 0.0
        self._bias_ay_sum = 0.0
        self._bias_count = 0
        self._bias_done = False

        # Pre-compute mounting quaternion: base_link <- imu_link
        mr, mp, my = [math.radians(v) for v in self.mounting_rpy_deg]
        # static TF publishes base->imu; we need imu->base for orientation correction → inverse (conjugate)
        q_base_from_imu = rpy_to_quat(mr, mp, my)
        self.q_imu_to_base = quat_conj(q_base_from_imu)
        self.q_imu_to_base = quat_normalize(self.q_imu_to_base)

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
                    self.get_logger().info("Auto-level: hold still for ~1–2s...")
                    
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
                    self.get_logger().info(f"Auto-level complete (samples={self._calib_count}).")

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
                gx, gy, gz = rotate_vec_by_quat((gx, gy, gz), q_total)
                ax, ay, az = rotate_vec_by_quat((ax, ay, az), q_total)
            else:
                qx, qy, qz, qw = q_imu

            # --- INTEGRATION: get linear velocity from acceleration (simple approx, no drift correction) ---

            # Learn accel bias on x,y while still (reduces integration drift)
            if self._calib_done and not self._bias_done and self.auto_level:
                az_lin = az - self.g  # gravity-compensated vertical
                gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
                alin_norm = math.sqrt(ax*ax + ay*ay + az_lin*az_lin)
                is_still_zupt = (gyro_norm < self.zupt_gyro_thresh) and (alin_norm < self.zupt_accel_thresh)
                if self._bias_count == 0:
                    self.get_logger().info("Bias computation: hold still for ~1–2s...")

                if is_still_zupt:
                    self._bias_ax_sum += ax
                    self._bias_ay_sum += ay
                    self._bias_count += 1

                if is_still_zupt and self._bias_count >= self.bias_samples:
                    self.ax_bias = self._bias_ax_sum / self._bias_count
                    self.ay_bias = self._bias_ay_sum / self._bias_count
                    self._bias_done = True
                    self.get_logger().info(f"Accel bias completed: ax_bias={self.ax_bias:.4f} m/s², ay_bias={self.ay_bias:.4f} m/s² (N={self._bias_count})")

            # dt from node clock (clamped to avoid spikes)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if self.prev_time_sec is None:
                dt = 1.0 / max(1.0, self.rate_hz)
            else:
                dt = max(0.0, min(0.2, now_sec - self.prev_time_sec))
            self.prev_time_sec = now_sec

            # Unbias + EMA prefilter (planar)
            ax_unbias = ax - self.ax_bias
            ay_unbias = ay - self.ay_bias
            alpha = max(0.0, min(1.0, self.filter_alpha))
            # Larger alpha -> more smoothing (more “memory” of the past), smaller alpha -> snappier.
            self.ax_filtered = alpha * self.ax_filtered + (1.0 - alpha) * ax_unbias
            self.ay_filtered = alpha * self.ay_filtered + (1.0 - alpha) * ay_unbias

            # Integrate velocity with ZUPT
            az_lin = az - self.g  # gravity-compensated vertical
            gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
            alin_norm = math.sqrt(ax*ax + ay*ay + az_lin*az_lin)
            is_still = (gyro_norm < self.zupt_gyro_thresh) and (alin_norm < self.zupt_accel_thresh)
            if is_still:
                self.vx = 0.0
                self.vy = 0.0
            else:
                self.vx += self.ax_filtered * dt
                self.vy += self.ay_filtered * dt

            # Anti-drift (leak) toward zero
            damp = math.exp(-self.velocity_damping_lambda * dt)
            self.vx *= damp
            self.vy *= damp

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

        # (STEP 3 will compute linear twist from integrated accel)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0

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
    