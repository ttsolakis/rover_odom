#!/usr/bin/env python3
"""
velocity_identification.py

Reads an imu_cmd_aligned_*.csv (from tools/), exposes the data as NumPy vectors,
computes velocity two ways:
  1) From wheel command: v_cmd = R * cmd_vx
  2) From IMU accel:     v_imu = ∫ a_x dt  (simple Euler integration)

…and plots them together for a quick comparison.

Usage:
  python3 tools/velocity_identification.py [optional_path_to_csv] [--save_npz]

Notes:
- Assumes CSV columns produced by imu_cmd_logger.py:
  t_out_sec,t_out_nanosec,t_imu_sec,t_imu_nanosec,t_cmd_sec,t_cmd_nanosec,
  imu_ax,imu_ay,imu_az,cmd_vx,cmd_vy,cmd_vz
- R is hardcoded to 0.040 m (wheel radius).
"""

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

R_WHEEL = 0.040  # meters

def find_latest_csv(tools_dir: str) -> str:
    pattern = os.path.join(tools_dir, "imu_cmd_aligned_*.csv")
    matches = sorted(glob.glob(pattern))
    if not matches:
        raise FileNotFoundError(f"No CSV files found matching {pattern}")
    return matches[-1]

def to_time_vector(sec_col, nsec_col):
    sec = sec_col.to_numpy(dtype=np.float64)
    nsec = nsec_col.to_numpy(dtype=np.float64)
    return sec + nsec * 1e-9

def main():
    # Resolve repo tools dir (works whether you run from repo root or anywhere)
    # This script lives in slam_ws/src/rover_odom/tools/
    here = os.path.abspath(os.path.dirname(_file_))
    tools_dir = here

    # Parse args
    save_npz = False
    csv_path = None
    for a in sys.argv[1:]:
        if a == "--save_npz":
            save_npz = True
        else:
            csv_path = a

    if csv_path is None:
        csv_path = find_latest_csv(tools_dir)

    print(f"[info] Reading: {csv_path}")
    df = pd.read_csv(csv_path, sep=",|\t", engine="python")  # allow tab or comma

    # --- Build time vectors (seconds) ---
    # We'll use the IMU timestamp for dt; plot against t0 = first out-stamp for nicer x axis.
    t_out = to_time_vector(df["t_out_sec"], df["t_out_nanosec"])
    t_imu = to_time_vector(df["t_imu_sec"], df["t_imu_nanosec"])
    t_cmd = to_time_vector(df["t_cmd_sec"], df["t_cmd_nanosec"])

    # Normalize times to start at zero for plotting
    t0 = float(t_out[0])
    t_plot = t_out - t0

    # --- Signals as vectors ---
    imu_ax = df["imu_ax"].to_numpy(dtype=np.float64)
    imu_ay = df["imu_ay"].to_numpy(dtype=np.float64)
    imu_az = df["imu_az"].to_numpy(dtype=np.float64)
    cmd_vx = df["cmd_vx"].to_numpy(dtype=np.float64)
    cmd_vy = df["cmd_vy"].to_numpy(dtype=np.float64)
    cmd_vz = df["cmd_vz"].to_numpy(dtype=np.float64)

    # Print brief summary so you can poke around later if needed
    print(f"[info] N = {len(imu_ax)} samples")
    print(f"[info] t range: {t_plot[0]:.3f} .. {t_plot[-1]:.3f} s")
    print(f"[info] imu_ax mean={imu_ax.mean():.6f} std={imu_ax.std():.6f}")

    # --- 1) Velocity from wheel command (assume cmd_vx is angular rate [rad/s]) ---
    v_cmd = R_WHEEL * cmd_vx  # m/s

    # --- 2) Velocity from IMU acceleration via simple integration ---
    # dt from IMU timestamps (guard against non-monotonic noise)
    dt = np.diff(t_imu, prepend=t_imu[0])
    # Clamp any tiny negative/huge spikes
    dt = np.clip(dt, 0.0, 0.2)

    # Naive Euler integration
    v_imu = np.cumsum(imu_ax * dt)

    # (Optional) remove initial bias drift by subtracting mean of first ~0.5s accel
    # n0 = max(1, int(0.5 / np.median(dt[1:]) if len(dt) > 1 else 1))
    # bias = imu_ax[:n0].mean()
    # v_imu = np.cumsum((imu_ax - bias) * dt)

    # --- Plot ---
    plt.figure()
    plt.plot(t_plot, v_cmd, label="v_cmd = R * cmd_vx (m/s)")
    plt.plot(t_plot, v_imu, label="v_imu = ∫ a_x dt (m/s)")
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.title("Velocity: wheel cmd vs IMU integration")
    plt.grid(True)
    plt.legend()
    out_png = os.path.join(tools_dir, "vel_compare.png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[info] Saved plot -> {out_png}")

    # --- Optionally save vectors for quick reuse ---
    if save_npz:
        out_npz = os.path.join(tools_dir, "vel_compare_data.npz")
        np.savez_compressed(
            out_npz,
            t_out=t_out, t_imu=t_imu, t_cmd=t_cmd, t_plot=t_plot,
            imu_ax=imu_ax, imu_ay=imu_ay, imu_az=imu_az,
            cmd_vx=cmd_vx, cmd_vy=cmd_vy, cmd_vz=cmd_vz,
            v_cmd=v_cmd, v_imu=v_imu, dt=dt
        )
        print(f"[info] Saved vectors -> {out_npz}")

if _name_ == "_main_":
    main()