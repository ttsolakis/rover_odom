#!/usr/bin/env python3
import math
from collections import deque
from typing import Deque, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from tf2_ros import TransformBroadcaster  

ready_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

def quat_from_yaw(yaw: float):
    """Quaternion (x,y,z,w) for a pure yaw."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))

def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rad) from quaternion (x,y,z,w) using ZYX convention."""
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

def wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


class EkfOdomNode(Node):
    """
    STEP 1: Subscribe to IMU odometry (/imu_odom) and wheel commands
    (/wheel_cmd). No filtering yet, just validate data flow.
    Subscriptions:
      • IMU odometry '/imu_odom' → measurement y_k = [phi, u, r]
      • Wheel commands '/wheel_cmd' → control τ_k (units→rad/s via gain)

    STEP 2: Keep I/O callbacks as data sources only; run EKF in a separate timer callback.

    STEP 3: Fuse data with EKF.

    STEP 4: Publish /ekf_odom (nav_msgs/Odometry).
    """

    def __init__(self):
        super().__init__('ekf_odom')

        # --- I/O Parameters (unchanged) ---
        self.declare_parameter('imu_odom_topic', '/imu_odom')
        self.declare_parameter('wheel_cmd_topic', '/wheel_cmd')
        self.declare_parameter('log_every_n', 20)
        self.declare_parameter('cmd_buffer_size', 500)

        self.imu_odom_topic: str = self.get_parameter('imu_odom_topic').value
        self.wheel_cmd_topic: str = self.get_parameter('wheel_cmd_topic').value
        self.log_every_n: int = int(self.get_parameter('log_every_n').value)
        self.cmd_buffer_size: int = int(self.get_parameter('cmd_buffer_size').value)

        # --- EKF + model parameters ---
        # Timing for the EKF runner (just a poller; we step only when a new IMU stamp appears)
        self.declare_parameter('ekf_rate_hz', 200.0)

        # Diff-drive model params
        self.declare_parameter('rho', 0.040)             # wheel radius [m]
        self.declare_parameter('half_track_l', 0.065)    # half track [m]

        # Process covariance diag (x,y,phi,u,r)
        self.declare_parameter('q_x',   1e-4)
        self.declare_parameter('q_y',   1e-4)
        self.declare_parameter('q_phi', 5e-4)
        self.declare_parameter('q_u',   1e-2)
        self.declare_parameter('q_r',   5e-3)

        # Measurement covariance diag (phi,u,r)
        self.declare_parameter('r_phi', 2e-3)
        self.declare_parameter('r_u',   1e-2)
        self.declare_parameter('r_r',   5e-3)

        # Initial state/covariance
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('phi0', 0.0)
        self.declare_parameter('u0', 0.0)
        self.declare_parameter('r0', 0.0)
        self.declare_parameter('p0_x',   1e-2)
        self.declare_parameter('p0_y',   1e-2)
        self.declare_parameter('p0_phi', 1e-1)
        self.declare_parameter('p0_u',   1e-1)
        self.declare_parameter('p0_r',   1e-1)
        self.declare_parameter('auto_init_from_first_measure', True)

        # --- Publishing params ---
        self.declare_parameter('ekf_odom', '/ekf_odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')

        # Read EKF params
        self.ekf_rate_hz = float(self.get_parameter('ekf_rate_hz').value)
        self.rho         = float(self.get_parameter('rho').value)
        self.l           = float(self.get_parameter('half_track_l').value)

        self.Q = np.diag([
            float(self.get_parameter('q_x').value),
            float(self.get_parameter('q_y').value),
            float(self.get_parameter('q_phi').value),
            float(self.get_parameter('q_u').value),
            float(self.get_parameter('q_r').value),
        ])
        self.R = np.diag([
            float(self.get_parameter('r_phi').value),
            float(self.get_parameter('r_u').value),
            float(self.get_parameter('r_r').value),
        ])

        self.x0   = float(self.get_parameter('x0').value)
        self.y0   = float(self.get_parameter('y0').value)
        self.phi0 = float(self.get_parameter('phi0').value)
        self.u0   = float(self.get_parameter('u0').value)
        self.r0   = float(self.get_parameter('r0').value)
        self.P    = np.diag([
            float(self.get_parameter('p0_x').value),
            float(self.get_parameter('p0_y').value),
            float(self.get_parameter('p0_phi').value),
            float(self.get_parameter('p0_u').value),
            float(self.get_parameter('p0_r').value),
        ]).astype(float)
        self.auto_init = bool(self.get_parameter('auto_init_from_first_measure').value)

        self.ekf_odom: str = self.get_parameter('ekf_odom').value
        self.publish_tf: bool = bool(self.get_parameter('publish_tf').value)
        self.odom_frame: str = self.get_parameter('odom_frame').value
        self.base_link_frame: str = self.get_parameter('base_link_frame').value

        # Publisher
        self.ekf_odom_pub = self.create_publisher(Odometry, self.ekf_odom, 10)

        # Optional TF
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # --- QoS ---
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=50,
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

        self._ready_sub = self.create_subscription(
            Bool, "/auto_level_ready", self._cb_ready, ready_qos
        )

        # --- State for validation/logging ---
        self._imu_count = 0
        self._last_imu_time: Optional[Time] = None
        self._last_cmd_time: Optional[Time] = None

        # Store last messages (for the EKF timer to consume)
        self.last_imu_odom: Optional[Odometry] = None

        # Ring buffer of (stamp_sec, L, R, duration_s)
        self.cmd_buf: Deque[Tuple[float, float, float, float]] = deque(maxlen=self.cmd_buffer_size)

        # --- Auto-level state ---
        self._auto_level_ready = False
        self._waiting_logged_once = False

        # --- EKF runtime state ---
        self.z = np.zeros((5, 1), dtype=float)   # state estimate: [x,y,phi,u,r]^T
        self.initialized = False
        self._last_processed_imu_sec: Optional[float] = None
        self._ekf_steps = 0

        # Timer: run EKF step when a new IMU odom timestamp appears
        period = 1.0 / max(1.0, self.ekf_rate_hz)
        self._ekf_timer = self.create_timer(period, self._ekf_tick)

        self.get_logger().info(
            f"EKF node ready. Subscribing to:\n"
            f"  • IMU odom: {self.imu_odom_topic}\n"
            f"  • Wheel cmds: {self.wheel_cmd_topic}\n"
            f"EKF polling at {self.ekf_rate_hz:.1f} Hz (steps only on new IMU data)."
        )


    # ========= Store-only callbacks (unchanged behavior) =========
    def _cb_ready(self, msg: Bool):
        if msg.data and not self._auto_level_ready:
            self._auto_level_ready = True
            self.get_logger().info(f"Auto-level is complete: EKF starts processing. . .")

    def _cb_imu_odom(self, msg: Odometry):
        self._imu_count += 1
        self.last_imu_odom = msg

        t = Time.from_msg(msg.header.stamp)
        dt = float('nan') if self._last_imu_time is None else (t - self._last_imu_time).nanoseconds * 1e-9
        self._last_imu_time = t

        # Light periodic logging for sanity
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        yaw = yaw_from_quat(qx, qy, qz, qw)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        # if self._imu_count % max(1, self.log_every_n) == 0:
        #     self.get_logger().info(
        #         f"[IMU ODOM] t={t.nanoseconds*1e-9:.3f}s  dt={0.0 if math.isnan(dt) else dt:.3f}s | "
        #         f"yaw={yaw:+.3f} rad | u,v=({vx:+.3f},{vy:+.3f}) m/s | r={wz:+.3f} rad/s"
        #     )

    def _cb_wheel_cmd(self, msg: Vector3Stamped):
        t = Time.from_msg(msg.header.stamp)
        dt = float('nan') if self._last_cmd_time is None else (t - self._last_cmd_time).nanoseconds * 1e-9
        self._last_cmd_time = t

        L = float(msg.vector.x)
        R = float(msg.vector.y)
        dur = float(msg.vector.z)  # “duration since last change” from teleop

        self.cmd_buf.append((t.nanoseconds * 1e-9, L, R, dur))

        # if self._imu_count % max(1, self.log_every_n) == 0:
        #     self.get_logger().info(
        #         f"[WHEEL CMD] t={t.nanoseconds*1e-9:.3f}s  dt={0.0 if math.isnan(dt) else dt:.3f}s | "
        #         f"L={L:+.3f} rad/s  R={R:+.3f} rad/s  (dur_since_change={dur:.2f}s)  buf={len(self.cmd_buf)}"
        #     )

    # ========= EKF helpers =========
    def _lookup_cmd_at(self, t_sec: float) -> Tuple[float, float]:
        """Return (omega_L, omega_R) [rad/s] for the interval ending at t_sec.
        Strategy: last entry with stamp <= t_sec; if none, zeros.
        """
        omega_L = omega_R = 0.0
        if not self.cmd_buf:
            return omega_L, omega_R

        idx = -1
        for i in range(len(self.cmd_buf)-1, -1, -1):
            if self.cmd_buf[i][0] <= t_sec:
                idx = i
                break
        if idx == -1:
            return 0.0, 0.0

        _, L, R, _ = self.cmd_buf[idx]
        return float(L), float(R)

    def _predict(self, z: np.ndarray, P: np.ndarray, dt: float, omega_L: float, omega_R: float):
        x, y, phi, u, r = z.flatten()

        # Motion model
        x_p   = x + math.cos(phi) * u * dt
        y_p   = y + math.sin(phi) * u * dt
        phi_p = wrap_pi(phi + r * dt)
        u_p   = 0.5 * self.rho * (omega_L + omega_R)
        r_p   = (self.rho / (2.0 * self.l)) * (omega_R - omega_L)
        z_pred = np.array([[x_p], [y_p], [phi_p], [u_p], [r_p]], dtype=float)

        # Jacobian wrt z
        F = np.zeros((5, 5), dtype=float)
        F[0, 0] = 1.0
        F[1, 1] = 1.0
        F[2, 2] = 1.0
        F[0, 2] = -math.sin(phi) * u * dt
        F[0, 3] =  math.cos(phi) * dt
        F[1, 2] =  math.cos(phi) * u * dt
        F[1, 3] =  math.sin(phi) * dt
        F[2, 4] =  dt

        P_pred = F @ P @ F.T + self.Q
        return z_pred, P_pred

    def _update(self, z_pred: np.ndarray, P_pred: np.ndarray, phi_meas: float, u_meas: float, r_meas: float):
        # y = [phi, u, r], h(z) = [phi, u, r]
        H = np.zeros((3, 5), dtype=float)
        H[0, 2] = 1.0
        H[1, 3] = 1.0
        H[2, 4] = 1.0

        y_meas = np.array([[phi_meas], [u_meas], [r_meas]], dtype=float)
        h_pred = np.array([[z_pred[2, 0]], [z_pred[3, 0]], [z_pred[4, 0]]], dtype=float)

        innov = y_meas - h_pred
        innov[0, 0] = wrap_pi(innov[0, 0])

        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T @ np.linalg.inv(S)

        z_upd = z_pred + K @ innov
        z_upd[2, 0] = wrap_pi(z_upd[2, 0])

        I = np.eye(5)
        P_upd = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ self.R @ K.T
        return z_upd, P_upd, innov

    # ========= EKF timer (does the actual work) =========
    def _ekf_tick(self):
        """Run EKF when a NEW IMU odom stamp is available. Otherwise do nothing."""

        if not self._auto_level_ready:
            if not self._waiting_logged_once:
                self.get_logger().info("Waiting for auto-level to complete...")
                self._waiting_logged_once = True
            return

        if self.last_imu_odom is None:
            return

        # Extract IMU stamp and measurement
        msg = self.last_imu_odom
        t_ros = Time.from_msg(msg.header.stamp)
        t_sec = t_ros.nanoseconds * 1e-9

        # Only step once per unique IMU stamp
        if self._last_processed_imu_sec is not None and t_sec <= self._last_processed_imu_sec:
            return

        # Compute dt based on last processed IMU time
        if self._last_processed_imu_sec is None:
            dt = 0.0
        else:
            dt = t_sec - self._last_processed_imu_sec

        # Clamp dt
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0
        dt = max(0.0, min(0.2, dt))

        # Measurement from IMU odom
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        phi_meas = yaw_from_quat(qx, qy, qz, qw)
        u_meas   = float(msg.twist.twist.linear.x)   # forward (body-x) speed from your pipeline
        r_meas   = float(msg.twist.twist.angular.z)  # gyro-z

        # Initialize if needed (from first measurement)
        if not self.initialized:
            if self.auto_init:
                self.z[:, 0] = np.array([self.x0, self.y0, phi_meas, u_meas, r_meas], dtype=float)
            else:
                self.z[:, 0] = np.array([self.x0, self.y0, self.phi0, self.u0, self.r0], dtype=float)
            self.z[2, 0] = wrap_pi(self.z[2, 0])
            self.initialized = True
            self._last_processed_imu_sec = t_sec
            self.get_logger().info(
                f"EKF initialized at:  x={self.z[0,0]:+.3f} m, y={self.z[1,0]:+.3f} m, "
                f"φ={self.z[2,0]:+.3f} rad, u={self.z[3,0]:+.3f} m/s, r={self.z[4,0]:+.3f} rad/s"
            )
            return

        # Look up control for the interval ending at this IMU time
        omega_L, omega_R = self._lookup_cmd_at(t_sec)

        # Predict & Update
        z_pred, P_pred = self._predict(self.z, self.P, dt, omega_L, omega_R)          #\hat{z}_{k|k-1}, P_{k|k-1}
        z_upd, P_upd, innov = self._update(z_pred, P_pred, phi_meas, u_meas, r_meas)  #\hat{z}_{k|k}, P_{k|k}

        # Commit
        self.z, self.P = z_upd, P_upd         #\hat{z}_{k|k-1} ← \hat{z}_{k|k}, P_{k|k-1} ← P_{k|k}
        self._last_processed_imu_sec = t_sec
        self._ekf_steps += 1

        # Periodic logging
        if self._ekf_steps % max(1, self.log_every_n) == 0:

            # self.get_logger().info(
            #     "EKF | dt={:.3f}s | "
            #     "ctrl: omega_L={:+.3f} rad/s, omega_R={:+.3f} rad/s | "
            #     "innov: dphi={:+.3f} rad, du={:+.3f} m/s, dr={:+.3f} rad/s | "
            #     "ẑ: x={:+.3f} m, y={:+.3f} m, φ={:+.3f} rad, u={:+.3f} m/s, r={:+.3f} rad/s".format(
            #         dt, omega_L, omega_R,
            #         innov[0, 0], innov[1, 0], innov[2, 0],
            #         self.z[0, 0], self.z[1, 0], self.z[2, 0], self.z[3, 0], self.z[4, 0]
            #     )

            self.get_logger().info(
                "EKF | dt={:.3f}s | "
                "ẑ: x={:+.3f} m, y={:+.3f} m, φ={:+.3f} rad, u={:+.3f} m/s, r={:+.3f} rad/s".format(
                    dt, omega_L, omega_R,
                    innov[0, 0], innov[1, 0], innov[2, 0],
                    self.z[0, 0], self.z[1, 0], self.z[2, 0], self.z[3, 0], self.z[4, 0]
                )

            )

        # ---------- Publish EKF odometry ----------
        stamp = t_ros.to_msg()

        # Pose in odom frame
        x = float(self.z[0, 0]); y = float(self.z[1, 0]); phi = float(self.z[2, 0])
        qx, qy, qz, qw = quat_from_yaw(phi)

        # Twist in child (base_link) frame
        u = float(self.z[3, 0])   # forward (body-x)
        r = float(self.z[4, 0])   # yaw rate

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (6x6, row-major): fill x,y,yaw from P; others large
        pose_cov = [0.0]*36
        # indices: (i,j) -> i*6+j
        P = self.P
        pose_cov[0]  = float(P[0,0])             # var(x)
        pose_cov[1]  = float(P[0,1]); pose_cov[6]  = float(P[1,0])  # cov(x,y)
        pose_cov[7]  = float(P[1,1])             # var(y)
        pose_cov[5]  = float(P[0,2]); pose_cov[30] = float(P[2,0])  # cov(x,yaw)
        pose_cov[11] = float(P[1,2]); pose_cov[31] = float(P[2,1])  # cov(y,yaw)
        pose_cov[35] = float(P[2,2])             # var(yaw)
        # large uncertainty for z, roll, pitch
        big = 1e3
        pose_cov[14] = big   # var(z)
        pose_cov[21] = big   # var(roll)
        pose_cov[28] = big   # var(pitch)
        odom.pose.covariance = pose_cov

        # Twist (base_link frame): we only estimate forward speed u and yaw rate r
        odom.twist.twist.linear.x  = u
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = r

        # Twist covariance (6x6): map var(u) to v_x, var(r) to yaw rate; others large
        twist_cov = [0.0]*36
        twist_cov[0]  = float(P[3,3])   # var(v_x) from P(u,u)
        twist_cov[35] = float(P[4,4])   # var(yaw rate) from P(r,r)
        twist_cov[7]  = big   # var(v_y)
        twist_cov[14] = big   # var(v_z)
        twist_cov[21] = big   # var(roll rate)
        twist_cov[28] = big   # var(pitch rate)
        odom.twist.covariance = twist_cov

        # Publish
        self.ekf_odom_pub.publish(odom)

        # TF: odom -> base_link
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_link_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


# ========= Main =========
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
