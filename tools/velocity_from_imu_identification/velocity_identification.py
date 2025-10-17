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
import matplotlib.colors as mcolors

R_WHEEL = 0.040  # meters

def darken(color_hex, factor=0.6):
    """Return a darker shade of color_hex (0<f<1)."""
    r, g, b = mcolors.to_rgb(color_hex)
    return (r*factor, g*factor, b*factor)

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

import numpy as np

import numpy as np
from collections import deque

import numpy as np

def causal_savitzky_golay_coeffs(length, poly_order, dt):
    """
    Compute **causal Savitzky–Golay endpoint weights** for a *uniformly sampled* signal.

    Goal
    ----
    Build a length-L FIR filter (weights h[0..L-1]) that, at time index i,
    approximates the signal value ŷ[i] by fitting a polynomial of degree `poly_order`
    to the **past L samples only** {y[i], y[i-1], ..., y[i-(L-1)]} (causal, one-sided),
    then evaluating that polynomial at the current time (t = 0).

        ŷ[i] = sum_{k=0}^{L-1} h[k] * y[i - k]

    Inputs
    ------
    length : int
        Number of past samples to use (L). Typical: 7–15. Must satisfy L >= poly_order + 1.
        Larger L → smoother but more lag; smaller L → less smoothing, more noise.
    poly_order : int
        Degree of the fitted polynomial (p). Typical: 2 or 3. Must satisfy p <= L - 1.
        p=2 preserves ramps/curvature well while smoothing noise.
    dt : float
        Uniform sampling interval (seconds). Used to build the time design matrix.
        (For non-uniform sampling, this endpoint formula is not valid.)

    Output
    ------
    h : ndarray, shape (L,)
        FIR weights to convolve with [y[i], y[i-1], ..., y[i-(L-1)]] to get the
        **smoothed value at the current time**. The filter is DC preserving:
        for p >= 0, sum(h) == 1 (within numerical precision).

    Method (least squares derivation)
    ---------------------------------
    We index the past L samples by k = 0..L-1, where k=0 is "now", k=1 is one step back, etc.
    Their corresponding times (relative to now) are t_k = -k * dt (non-positive).

    We seek polynomial coefficients a = [a0, a1, ..., ap]^T minimizing:
        min_a || A a - y ||^2
    where rows of A are [1, t_k, t_k^2, ..., t_k^p], and y = [y[i], y[i-1], ..., y[i-(L-1)]]^T.

    The LS solution is a* = (A^T A)^{-1} A^T y (using pseudoinverse for stability).
    The smoothed value at t=0 is just the polynomial’s value at 0:
        ŷ[i] = a0  (since t=0 → [1, 0, 0, ..., 0], i.e., selects the constant term)

    We want a **linear operator** ŷ[i] = h^T y, so:
        h^T = e0^T (A^T A)^{-1} A^T
    where e0 = [1, 0, ..., 0]^T picks the constant term.

    Rewriting (and transposing) gives a convenient expression for h:
        h = A (A^T A)^{-1} e0

    Notes
    -----
    • This is an **endpoint**, **causal** (one-sided) SG filter, so it behaves well at the
      “right edge” of a streaming time series. It preserves low-degree polynomials up to `p`
      (i.e., zero phase and minimal bias on trends with degree ≤ p), while reducing noise.

    • If L is small or p is high, (A^T A) can be ill-conditioned; we use `pinv` for stability.

    • The returned weights assume you multiply them against the vector
      [y[i], y[i-1], ..., y[i-(L-1)]] in that exact order.

    • Derivatives: if you wanted the **first derivative** at t=0 instead of the value,
      you would replace e0 with e1 = [0, 1, 0, ...]^T and then divide by 1!, etc.
      (For k-th derivative: use ek and divide by k!.)

    Complexity
    ----------
    Building A: O(L * p).  Forming A^T A: O(L * p^2).  Inverting (p+1)x(p+1): negligible for small p.
    You typically precompute h once and reuse it for many samples.

    """
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

def ema_update(previous_value: float, new_value: float, dt: float, time_constant_s: float) -> float:
    """
    Exponential moving average with dt-aware alpha.
    alpha = dt / (tau + dt);   tau <= 0 means 'no smoothing' (just take new_value).
    """
    if time_constant_s <= 0.0:
        return new_value
    alpha = dt / (time_constant_s + dt)
    return (1.0 - alpha) * previous_value + alpha * new_value


def filter_and_unbias_acceleration(time_imu_seconds, accel_x_mps2, window_length=15, poly_order=2, dt_nominal=0.01,  acceleration_filtering=True):
    
    # Preallocate output
    filtered_acceleration = np.zeros(len(accel_x_mps2))
    bias_corrected_acceleration = np.zeros(len(filtered_acceleration))
    
    # --- Savitzky-Golay filter setup ---
    # Idea: Smooth the raw acceleration to reduce noise before integration with a causal SG filter.
    # This is a low-pass FIR filter that preserves low-degree polynomials up to `poly_order`.
    # It has no phase distortion (zero phase on polynomials up to `poly_order`), so it doesn't
    # bias ramps/curvature, just smooths high-frequency noise.
    dt_nominal = float(np.median(np.diff(time_imu_seconds)))
    window_length = 10    # must satisfy window_length >= poly_order+1
    poly_order = 2        # quadratic fit usually works well
    assert window_length >= poly_order + 1, "window_length must be >= poly_order + 1"
    savitzky_golay_weights_full = causal_savitzky_golay_coeffs(window_length, poly_order, dt_nominal)
    print(f"Savitzky-Golay weights (L={window_length}, p={poly_order}): {savitzky_golay_weights_full}")
    acceleration_buffer = deque(maxlen=window_length)


    # ---- Bias learning (windowed mean while stationary) ----
    hard_zero_when_stationary = True                 # force accel to 0 during ZUPT after bias removal
    stationary_abs_threshold_mps2 = 0.1              # |a_filtered| must be below this to consider “quiet”
    stationary_min_consecutive_samples = 10          # need this many quiet samples to declare ZUPT

    # Size of the averaging window in *seconds*; convert to samples using nominal dt
    bias_window_samples = stationary_min_consecutive_samples
    zupt_window_values = deque(maxlen=bias_window_samples)

    # Main loop
    bias_estimate = 0.0
    consecutive_stationary_count = 0
    for i in range(1, len(accel_x_mps2)):
        dt = time_imu_seconds[i] - time_imu_seconds[i-1]

        # --- 1) Filter raw acceleratation data ---
        if acceleration_filtering:
            acceleration_buffer.append(float(accel_x_mps2[i]))
            samples_new_to_old = np.fromiter(acceleration_buffer, float)[::-1]
            if len(acceleration_buffer) < window_length:
                # Not enough samples yet to fill the SG window; use smaller window
                window_length_small = len(acceleration_buffer)
                poly_order_small = min(poly_order, window_length_small - 1)
                savitzky_golay_weights = causal_savitzky_golay_coeffs(window_length_small, poly_order_small, dt_nominal)
                filtered_value = float(np.dot(savitzky_golay_weights, samples_new_to_old))
            else:
                savitzky_golay_weights = savitzky_golay_weights_full
            filtered_value = float(np.dot(savitzky_golay_weights, samples_new_to_old))
        else:
            filtered_value = float(accel_x_mps2[i])

        filtered_acceleration[i] = filtered_value

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

        # Update bias only when confidently stationary and we have a full window of quiet samples
        if is_stationary and len(zupt_window_values) == bias_window_samples:
            bias_estimate = float(np.mean(zupt_window_values))

        # Apply bias removal
        if is_stationary and hard_zero_when_stationary:
            # While stationary, force the corrected accel to exact zero to avoid tiny residuals
            bias_corrected_acceleration[i] = 0.0
        else:
            # During motion (or if you prefer not to hard-zero), subtract the learned bias
            bias_corrected_acceleration[i] = filtered_value - bias_estimate

    return filtered_acceleration, bias_corrected_acceleration

def estimate_velocity_from_acceleration(time_imu_seconds, filtered_acceleration, cmd_wheel_rate_rad_s, wheel_radius_m=R_WHEEL):

    # Preallocate output
    velocity_estimate_mps = np.zeros(len(filtered_acceleration))

    # Main loop: integrate acceleration to get velocity
    constant_velocity_counter = 0
    for i in range(1, len(filtered_acceleration)):
        
        # --- 1) ZUPT gating ---
        zupt_speed_threshold_mps=0.01
        velocity_cmd_linear_mps = wheel_radius_m * cmd_wheel_rate_rad_s[i]
        previous_velocity_cmd_linear_mps = wheel_radius_m * cmd_wheel_rate_rad_s[i-1]
        near_zero_cmd = (abs(velocity_cmd_linear_mps) < zupt_speed_threshold_mps)
        sign_flip_cmd = (velocity_cmd_linear_mps * previous_velocity_cmd_linear_mps) < 0.0
        stationary = near_zero_cmd or sign_flip_cmd

        if stationary:
            velocity_estimate_mps[i-1] = 0.0
            velocity_estimate_mps[i]   = 0.0
            constant_velocity_counter = 0
            continue  # freeze integrator this sample

        # --- 2) CUPT gating ---
        constant_velocity_threshold = 10

        constant_velocity_command = (abs(wheel_radius_m * cmd_wheel_rate_rad_s[i] - wheel_radius_m * cmd_wheel_rate_rad_s[i-1]) <= 0.01)
        if constant_velocity_command:
            constant_velocity_counter += 1
        else:
            constant_velocity_counter = 0

        if constant_velocity_counter >= constant_velocity_threshold:
            velocity_estimate_mps[i-1] = velocity_cmd_linear_mps
            velocity_estimate_mps[i] = velocity_cmd_linear_mps
            constant_velocity_counter = 0
            continue

        # --- 3) Trapezoidal Integration ---
        dt = time_imu_seconds[i] - time_imu_seconds[i-1]     
        velocity_estimate_mps[i] =  velocity_estimate_mps[i-1] + 0.5*(filtered_acceleration[i] + filtered_acceleration[i-1]) * dt

    return velocity_estimate_mps

def main():
    # Resolve repo tools dir (works whether you run from repo root or anywhere)
    # This script lives in slam_ws/src/rover_odom/tools/
    here = os.path.abspath(os.path.dirname(__file__))
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

    # --- 2) Acceleration post-processing
    filtered_acceleration, bias_corrected_acceleration = filter_and_unbias_acceleration(t_imu, imu_ax, acceleration_filtering=True)
    ma_ax = pd.Series(imu_ax).rolling(int(round(2.0/np.median(np.diff(t_imu)))), center=True, min_periods=1).mean().to_numpy()
    ma_ax_filtered = pd.Series(filtered_acceleration).rolling(int(round(2.0/np.median(np.diff(t_imu)))), center=True, min_periods=1).mean().to_numpy()
    ma_ax_unbiased = pd.Series(bias_corrected_acceleration).rolling(int(round(2.0/np.median(np.diff(t_imu)))), center=True, min_periods=1).mean().to_numpy()

    # --- 3) Velocity from IMU acceleration via simple integration ---
    v_imu_filtered = estimate_velocity_from_acceleration(t_imu, filtered_acceleration, cmd_vx, wheel_radius_m=R_WHEEL)
    v_imu_unbiased = estimate_velocity_from_acceleration(t_imu, bias_corrected_acceleration, cmd_vx, wheel_radius_m=R_WHEEL)

    # --- Plot ---
    plt.figure()
    plt.plot(t_plot, imu_ax, color="#1f77b4", alpha=0.7, linewidth=0.2, label="raw acceleration")
    plt.plot(t_plot, ma_ax, color=darken("#1f77b4", factor=0.6), linewidth=0.2)
    plt.plot(t_plot, filtered_acceleration, color="#ff7f0e", alpha=0.7, linewidth=0.2, label="filtered acceleration")
    plt.plot(t_plot, ma_ax_filtered, color=darken("#ff7f0e", factor=0.6), linewidth=0.2)
    plt.plot(t_plot, bias_corrected_acceleration, color="#2ca02c", alpha=0.7, linewidth=0.2, label="unbiased acceleration", linestyle='--')
    plt.plot(t_plot, ma_ax_unbiased, color=darken("#2ca02c", factor=0.6), linewidth=0.2, linestyle='--')
    plt.xlabel("time (s)")
    plt.ylabel("acceleration:x (m/s²)")
    plt.title("Longitudinal Acceleration: IMU")
    plt.grid(True)
    plt.legend()
    out_base_acc = os.path.join(tools_dir, "accel_imu")
    plt.savefig(out_base_acc + ".png", dpi=300, bbox_inches="tight")  # high-res raster
    plt.savefig(out_base_acc + ".pdf",            bbox_inches="tight")  # vector (zoom!)
    print(f"[info] Saved plots -> {out_base_acc}.png/.pdf")

    plt.figure()
    plt.plot(t_plot, v_cmd, label="v_cmd = R * cmd_vx (m/s)")
    plt.plot(t_plot, v_imu_filtered, label="v_imu = ∫ a_x dt (m/s) - a_x filtered", linewidth=0.2)
    plt.plot(t_plot, v_imu_unbiased, label="v_imu = ∫ a_x dt (m/s) - a_x unbiased", linewidth=0.2, linestyle='--')
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.title("Velocity: wheel cmd vs IMU integration")
    plt.grid(True)
    plt.legend()
    out_base_vel = os.path.join(tools_dir, "vel_compare")
    plt.savefig(out_base_vel + ".png", dpi=300, bbox_inches="tight")  # high-res raster
    plt.savefig(out_base_vel + ".pdf",            bbox_inches="tight")  # vector (zoom!)
    print(f"[info] Saved plots -> {out_base_vel}.png/.pdf")

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

if __name__ == "__main__":
    main()