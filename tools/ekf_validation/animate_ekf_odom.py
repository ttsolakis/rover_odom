#!/usr/bin/env python3
"""
Animate the LATEST exported CSV (t,x,y,yaw) from ~/bags and save an MP4 next to this script.

It expects CSVs created by bag_to_csv_ekf_odom.py, named like:
  ~/bags/ekf_drive_YYYYMMDD_HHMMSS.csv
with columns: t,x,y,yaw

Draws:
  • dashed "progress path" from the start (0) to the robot's current center
  • optional short trailing segment (solid) near the robot
  • center dot at (x,y)
  • a square (4 lines) centered at (x,y), rotated by yaw, side = --box_side

Usage (no args; auto-picks latest CSV):
  python3 ~/slam_ws/src/rover_odom/tools/ekf_validation/animate_ekf_odom.py

Optional:
  --csv /path/to/file.csv
  --bags_dir ~/bags
  --box_side 0.22
  --trail 200
  --speed 1.0
  --dpi 120

Requires:
  sudo apt install ffmpeg
"""
import os, sys, glob, math, argparse, shutil
import numpy as np
from datetime import datetime

# Headless-safe backend (no GUI window)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FFMpegWriter

DEF_BAGS_DIR = os.path.expanduser('~/bags')
DEF_PREFIX   = 'ekf_drive_'

# ---------------------------- helpers ---------------------------- #

def find_latest_csv(bags_dir: str, prefix: str) -> str:
    """
    Find the newest CSV by lexicographic order matching <bags_dir>/<prefix>*.csv
    (Works with names like ekf_drive_YYYYMMDD_HHMMSS.csv)
    """
    pattern = os.path.join(bags_dir, f"{prefix}*.csv")
    candidates = [p for p in glob.glob(pattern) if os.path.isfile(p)]
    if not candidates:
        raise FileNotFoundError(f"No CSVs matching {pattern}")
    # Sort by basename descending (timestamp is lexicographically sortable)
    return sorted(candidates, key=lambda p: os.path.basename(p), reverse=True)[0]

def load_csv(csv_path: str):
    """
    Load CSV with columns t,x,y,yaw. Supports header or no header.
    Returns arrays T, X, Y, YAW (float).
    """
    csv_path = os.path.expanduser(csv_path)
    if not os.path.exists(csv_path):
        raise FileNotFoundError(csv_path)

    # Try names=True first (handles header "t,x,y,yaw")
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=float)
        if {'t','x','y','yaw'}.issubset(set(data.dtype.names or [])):
            T   = np.asarray(data['t'],   dtype=float)
            X   = np.asarray(data['x'],   dtype=float)
            Y   = np.asarray(data['y'],   dtype=float)
            YAW = np.asarray(data['yaw'], dtype=float)
            return T, X, Y, YAW
        # Fall through to no-header parse
    except Exception:
        pass

    # Fallback: no header (four columns)
    arr = np.loadtxt(csv_path, delimiter=",", dtype=float)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 4:
        raise ValueError(f"CSV must have at least 4 columns (t,x,y,yaw). Got shape {arr.shape}")
    T, X, Y, YAW = arr[:,0], arr[:,1], arr[:,2], arr[:,3]
    return T, X, Y, YAW

# ---------------------------- main ---------------------------- #

def main():
    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg not found in PATH. Install it: sudo apt install ffmpeg")

    ap = argparse.ArgumentParser(description="Animate latest ekf_drive_*.csv from ~/bags and save MP4.")
    ap.add_argument("--csv", default=None, help="Explicit CSV path (default: latest ~/bags/ekf_drive_*.csv)")
    ap.add_argument("--bags_dir", default=DEF_BAGS_DIR, help="Directory to search for ekf_drive_*.csv (default: ~/bags)")
    ap.add_argument("--prefix", default=DEF_PREFIX, help="CSV prefix (default: ekf_drive_)")
    ap.add_argument("--box_side", type=float, default=0.22, help="Square side length [m]")
    ap.add_argument("--trail", type=int, default=0, help="If >0, draw a short solid trail of this many points near the robot")
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed scaling → affects FPS")
    ap.add_argument("--dpi", type=int, default=120, help="DPI for rendering")
    args = ap.parse_args()

    # Resolve CSV
    if args.csv:
        csv_path = os.path.expanduser(args.csv)
    else:
        csv_path = find_latest_csv(os.path.expanduser(args.bags_dir), args.prefix)

    print(f"Reading CSV: {csv_path}")
    T, X, Y, YAW = load_csv(csv_path)
    N = len(T)
    if N == 0:
        sys.exit("CSV contains no rows.")

    # Figure & axes
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")

    # Bounds with margins
    xm, xM = float(np.nanmin(X)), float(np.nanmax(X))
    ym, yM = float(np.nanmin(Y)), float(np.nanmax(Y))
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

    # Frame timing → FPS (estimate from median dt and scale by speed)
    if N > 1:
        median_dt = float(np.median(np.diff(T)))
    else:
        median_dt = 1.0 / 30.0
    fps = max(10, min(60, int((1.0 / max(median_dt, 1e-6)) * max(args.speed, 1e-6))))
    interval_ms = 1000.0 / fps

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
        frames=N, interval=int(interval_ms),
        blit=False, repeat=False
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
