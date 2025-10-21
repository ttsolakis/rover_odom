#!/usr/bin/env python3
"""
Animate /ekf_odom from a ROS 2 bag (MCAP/SQLite) and save an MP4 next to this script.

Draws:
  • dashed "progress path" from the start (0) to the robot's current center
  • optional short trailing segment (solid) near the robot
  • center dot at (x,y)
  • a square (4 lines) centered at (x,y), rotated by yaw, side = --box_side

Usage:
  python3 animate_ekf_odom_dashed.py --bag ~/bags/ekf_drive_20251021_134240 --box_side 0.22

Requires:
  sudo apt install ros-jazzy-rosbag2* ffmpeg
"""
import os, math, argparse, yaml, shutil
import numpy as np
from datetime import datetime

# Headless-safe backend (no GUI window)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FFMpegWriter

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

try:
    import rosbag2_py
except Exception:
    raise SystemExit("rosbag2_py not found. Install: sudo apt install ros-jazzy-rosbag2*")

# ---------------------------- helpers ---------------------------- #

def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)

def read_series(bag_dir: str, topic: str):
    meta = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(meta):
        raise FileNotFoundError(f"metadata.yaml not found in: {bag_dir}")

    with open(meta, "r") as f:
        storage_id = yaml.safe_load(f).get("storage_identifier", "mcap")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise SystemExit(
            f"Topic '{topic}' not found in bag.\nAvailable topics:\n" +
            "\n".join(sorted(topics))
        )

    T, X, Y, YAW = [], [], [], []
    t0 = None

    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name != topic:
            continue

        msg: Odometry = deserialize_message(data, Odometry)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        t = t_ns * 1e-9
        if t0 is None:
            t0 = t

        T.append(t - t0)
        X.append(x)
        Y.append(y)
        YAW.append(yaw)

    if not T:
        raise SystemExit(f"No messages on topic '{topic}' in the bag.")

    return np.array(T), np.array(X), np.array(Y), np.array(YAW)

# ---------------------------- main ---------------------------- #

def main():
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg not found in PATH. Install it: sudo apt install ffmpeg")

    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to bag directory (contains metadata.yaml)")
    ap.add_argument("--topic", default="/ekf_odom", help="Odometry topic to read")
    ap.add_argument("--box_side", type=float, default=0.22, help="Side length of the robot square [m]")
    ap.add_argument("--trail", type=int, default=0, help="If >0, draw a short solid trail of this many points near the robot")
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed scaling for FPS")
    ap.add_argument("--dpi", type=int, default=120, help="DPI for rendering")
    args = ap.parse_args()

    bag_dir = os.path.expanduser(args.bag.rstrip("/"))
    T, X, Y, YAW = read_series(bag_dir, args.topic)

    # Figure & axes
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")

    # Bounds with margins
    xm, xM = float(X.min()), float(X.max())
    ym, yM = float(Y.min()), float(Y.max())
    pad = max(0.5, 0.1 * max(xM - xm, yM - ym, 1.0))
    ax.set_xlim(xm - pad, xM + pad)
    ax.set_ylim(ym - pad, yM + pad)
    ax.grid(True, linestyle=":", linewidth=0.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("/ekf_odom path")

    # Crosshairs at origin
    ax.axhline(0, linestyle="--", linewidth=0.8)
    ax.axvline(0, linestyle="--", linewidth=0.8)

    # Artists
    (progress_path_line,) = ax.plot([], [], linestyle="--", lw=2.0, alpha=0.9)  # dashed path from start to current
    (center_dot,) = ax.plot([], [], "o", ms=5)                                  # robot center
    trail_line = None
    if args.trail > 0:
        (trail_line,) = ax.plot([], [], lw=1.5, alpha=0.9)                       # optional short solid trail
    square_edges = [ax.plot([], [], lw=2.0)[0] for _ in range(4)]                # robot body square
    t_txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    # Frame timing → interval (ms) and FPS
    if len(T) > 1:
        median_dt = float(np.median(np.diff(T)))
    else:
        median_dt = 1.0 / 30.0  # fallback

    interval_ms = max(10.0, (median_dt / max(args.speed, 1e-6)) * 1000.0)
    fps = max(10, int(1000.0 / interval_ms))

    # Local (centered) square corners: TL, TR, BR, BL
    s = args.box_side * 0.5
    local = np.array([[-s, +s],
                      [+s, +s],
                      [+s, -s],
                      [-s, -s]], dtype=float)

    def init():
        progress_path_line.set_data([], [])
        center_dot.set_data([], [])
        if trail_line is not None:
            trail_line.set_data([], [])
        for ln in square_edges:
            ln.set_data([], [])
        t_txt.set_text("")
        return (progress_path_line, center_dot, *(square_edges), t_txt) if trail_line is None \
               else (progress_path_line, center_dot, trail_line, *(square_edges), t_txt)

    def update(i):
        xi, yi, yaw = X[i], Y[i], YAW[i]

        # Dashed path from start to current center
        progress_path_line.set_data(X[:i+1], Y[:i+1])

        # Optional short solid trail near the robot
        if trail_line is not None:
            j0 = max(0, i - args.trail)
            trail_line.set_data(X[j0:i+1], Y[j0:i+1])

        # Center dot
        center_dot.set_data([xi], [yi])

        # Square corners: rotate + translate
        c, s_ = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s_], [s_, c]], dtype=float)
        world = (local @ R.T) + np.array([xi, yi])

        # 4 edges (TL->TR, TR->BR, BR->BL, BL->TL)
        for k in range(4):
            p0 = world[k]
            p1 = world[(k + 1) % 4]
            square_edges[k].set_data([p0[0], p1[0]], [p0[1], p1[1]])

        t_txt.set_text(f"t = {T[i]:.2f}s")
        return (progress_path_line, center_dot, *(square_edges), t_txt) if trail_line is None \
               else (progress_path_line, center_dot, trail_line, *(square_edges), t_txt)

    anim = animation.FuncAnimation(
        fig, update, init_func=init,
        frames=len(T), interval=int(interval_ms),
        blit=False,           # robust saving
        repeat=False
    )

    # Save MP4 next to this .py file with broadly compatible settings
    print("Saving animation...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fname = f"ekf_odom_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
    out_path = os.path.join(script_dir, fname)

    writer = FFMpegWriter(
        fps=fps,
        codec="libx264",
        bitrate=1800,
        extra_args=["-pix_fmt", "yuv420p",
                    "-profile:v", "baseline", "-level", "3.0",
                    "-movflags", "+faststart"]
    )
    anim.save(out_path, writer=writer, dpi=args.dpi)
    plt.close(fig)

    try:
        sz = os.path.getsize(out_path)
        print(f"Saved animation to {out_path} ({sz/1e6:.2f} MB, {fps} fps)")
    except Exception:
        print(f"Saved animation to {out_path}")

if __name__ == "__main__":
    main()
