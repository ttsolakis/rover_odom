#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt

# --- constants ---
R = 0.040           # wheel radius [m]
DT = 1.0            # straight-line sample duration [s]
HALF_TRACK = 0.064  # half track [m] (given)
YAW_TO_WHEEL = HALF_TRACK / R  # omega_wheel = (l/R) * yaw_rate

# --- STRAIGHT-LINE measurements (distance in meters over 1 s) ---
# CMD applied to both L and R; six repeats per row
data_straight = {
    +0.1: [0.077, 0.075, 0.076, 0.078, 0.075, 0.075],
    +0.2: [0.250, 0.251, 0.251, 0.253, 0.250, 0.252],
    +0.3: [0.422, 0.421, 0.425, 0.424, 0.424, 0.423],
    +0.4: [0.588, 0.585, 0.590, 0.588, 0.587, 0.588],
    -0.1: [0.070, 0.070, 0.073, 0.073, 0.071, 0.074],
    -0.2: [0.248, 0.244, 0.244, 0.245, 0.244, 0.243],
    -0.3: [0.408, 0.410, 0.405, 0.410, 0.406, 0.408],
    -0.4: [0.572, 0.575, 0.570, 0.569, 0.567, 0.569],
}

# Flatten into (cmd, omega) samples, using omega = l / (R * DT), sign from cmd
xs, ys = [], []
for cmd, dists in data_straight.items():
    for l in dists:
        omega_mag = l / (R * DT)                 # rad/s, magnitude
        omega = math.copysign(omega_mag, cmd)    # sign by cmd
        xs.append(cmd)
        ys.append(omega)

x = np.array(xs)
y = np.array(ys)

# Split and fit straight-line pos/neg subsets: y = m*x + b
pos_mask = x > 0
neg_mask = x < 0
x_pos, y_pos = x[pos_mask], y[pos_mask]
x_neg, y_neg = x[neg_mask], y[neg_mask]
m_pos, b_pos = np.polyfit(x_pos, y_pos, 1)
m_neg, b_neg = np.polyfit(x_neg, y_neg, 1)

print("STRAIGHT LINE fits (ω = m * cmd + b):")
print(f"  Positive cmds: m = {m_pos:.4f} rad/s per unit, b = {b_pos:+.4f} rad/s")
print(f"  Negative cmds: m = {m_neg:.4f} rad/s per unit, b = {b_neg:+.4f} rad/s")
print()

# --- TURN-IN-PLACE measurements (from IMU yaw rate) ---
# For pure spin: ω_z = (R/ℓ) * ω_wheel  →  ω_wheel = (ℓ/R) * ω_z
# We'll treat x-axis as the command magnitude (0.4, 0.5) and y as signed per-wheel ω.

# Left turns (cmd_sign = +1): (L=-u, R=+u) → wz_mean is positive in your logs
spin_left_yaw = {
    0.5: [2.489983, 2.115595, 2.025874, 1.666194],
    0.4: [0.810316, 0.599074, 0.272914, 0.334295],
}

# Right turns (cmd_sign = -1): (L=+u, R=-u) → wz_mean is negative in your logs
spin_right_yaw = {
    0.5: [-2.689828, -2.323768, -2.195728, -1.821701],
    0.4: [-0.968887, -0.897814, -0.599232, -0.499477],
}

def flatten_spin_dict(yaw_dict):
    """Return arrays (u_cmd, omega_wheel) with u_cmd>0 and signed wheel-omega."""
    xs_cmd, ys_omega = [], []
    for u, wz_list in yaw_dict.items():
        for wz in wz_list:
            xs_cmd.append(u)                 # x: command magnitude (positive)
            ys_omega.append(YAW_TO_WHEEL * wz)  # y: signed per-wheel ω
    return np.array(xs_cmd), np.array(ys_omega)

# Left-turn plot & fit
x_left, y_left = flatten_spin_dict(spin_left_yaw)
m_left, b_left = np.polyfit(x_left, y_left, 1)

# Right-turn plot & fit
x_right, y_right = flatten_spin_dict(spin_right_yaw)
m_right, b_right = np.polyfit(x_right, y_right, 1)

print("TURN-IN-PLACE fits (per-wheel ω = m * cmd_mag + b):")
print(f"  Left turns:  m = {m_left:.4f} rad/s per unit, b = {b_left:+.4f} rad/s")
print(f"  Right turns: m = {m_right:.4f} rad/s per unit, b = {b_right:+.4f} rad/s")
print(f"(Used HALF_TRACK={HALF_TRACK:.3f} m, R={R:.3f} m → ω_wheel = {(HALF_TRACK/R):.3f} * yaw_rate)")
print()

# --- PLOTS ---
# 1) Straight-line mapping
plt.figure()
plt.scatter(x_pos, y_pos, label="Samples (cmd>0)", marker='o')
plt.scatter(x_neg, y_neg, label="Samples (cmd<0)", marker='x')
xpos_line = np.array([x_pos.min(), x_pos.max()])
xneg_line = np.array([x_neg.min(), x_neg.max()])
plt.plot(xpos_line, m_pos * xpos_line + b_pos, label="Fit (pos)")
plt.plot(xneg_line, m_neg * xneg_line + b_neg, label="Fit (neg)")
plt.xlabel("command (unit)")
plt.ylabel("wheel angular speed ω (rad/s)")
plt.title("Wheel speed vs command — straight-line tests")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("omega_vs_cmd_straight.png", dpi=150)

# 2) Turn-in-place — LEFT
plt.figure()
plt.scatter(x_left, y_left, label="Left-turn samples", marker='o')
x_left_line = np.array([x_left.min(), x_left.max()])
plt.plot(x_left_line, m_left * x_left_line + b_left, label="Fit (left)")
plt.xlabel("command magnitude (unit)")
plt.ylabel("per-wheel angular speed ω (rad/s)")
plt.title("Wheel ω vs command — turn-in-place (LEFT)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("omega_vs_cmd_spin_left.png", dpi=150)

# 3) Turn-in-place — RIGHT
plt.figure()
plt.scatter(x_right, y_right, label="Right-turn samples", marker='x')
x_right_line = np.array([x_right.min(), x_right.max()])
plt.plot(x_right_line, m_right * x_right_line + b_right, label="Fit (right)")
plt.xlabel("command magnitude (unit)")
plt.ylabel("per-wheel angular speed ω (rad/s)")
plt.title("Wheel ω vs command — turn-in-place (RIGHT)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("omega_vs_cmd_spin_right.png", dpi=150)

plt.show()
