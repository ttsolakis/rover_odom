# rover_odom

ROS 2 (Jazzy) package for **low-cost odometry and mapping**. It polls an IMU over HTTP (`T=126` JSON), publishes `nav_msgs/Odometry`, fuses wheel speeds with an EKF, and can bring up LiDAR + SLAM.

**Pipeline**
```
IMU JSON  ──>  imu_reader ──── /imu_odom ─┐
    ────────────┘                         ├─>  ekf_odom  ── /ekf_odom (+ TF odom→base_link by default)
Teleop     ──>  rover_teleop ─ /wheel_cmd ┘
LiDAR      ──>  sllidar_ros2  ─────────── /scan ─→ slam_toolbox ─→ map
```

---

## Features

- **IMU → Odometry**: converts IMU RPY/gyro/accel (HTTP JSON) to `/imu_odom` in the robot’s frame, handling IMU mounting + optional auto-level.
- **Bias & filtering**: causal Savitzky–Golay smoothing + simple bias learning and ZUPT/CUPT gating for drift-limited forward speed.
- **EKF odometry**: fuses `/imu_odom` (yaw, forward speed, yaw rate) with **wheel speeds** to publish `/ekf_odom` (+ TF `odom→base_link`).
- **Bringup**: LiDAR driver, slam_toolbox (with lifecycle), robot_state_publisher (URDF), optional WASD teleop.

---

## Quick start

```bash
# Dependencies (Ubuntu 24.04 + ROS 2 Jazzy)
sudo apt update
sudo apt install   ros-jazzy-desktop ros-jazzy-robot-state-publisher ros-jazzy-xacro   ros-jazzy-slam-toolbox ros-jazzy-nav2-lifecycle-manager ros-jazzy-ros2bag   ros-jazzy-rviz2 python3-numpy python3-requests gnome-terminal

# Workspace
mkdir -p ~/slam_ws/src && cd ~/slam_ws/src
git clone https://github.com/ttsolakis/rover_odom
git clone https://github.com/Slamtec/sllidar_ros2   # LiDAR driver from source
cd ~/slam_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash

# Full bringup (LiDAR + IMU → /imu_odom + EKF + SLAM; teleop in a separate terminal)
ros2 launch rover_odom bringup.launch.py
```

> Teleop uses `gnome-terminal`. If you don’t have it, install it or run with `start_teleop:=false`.

---

## Nodes (I/O cards)

### 1) `rover_teleop`  — WASD keyboard → wheel **speeds**
**Inputs:** keyboard (W/A/S/D, SPACE, 1-5)  
**Outputs:** `geometry_msgs/Vector3Stamped` on **`/wheel_cmd`**  
- `vector.x = ω_L` (rad/s), `vector.y = ω_R` (rad/s), `vector.z = seconds since last change`  
- Also (optionally) sends HTTP heartbeat to the rover: `GET http://…/js?json={"T":1,"L":…,"R":…}`

**Key params**
- `http_url` (default `http://192.168.4.1/js`), `send_http` (bool), `timeout_s`
- `rate_hz` (publish/heartbeat), `max_unit`, `max_apply_s`
- Internally maps WASD “units” → per-wheel **rad/s** using your fitted linear models.

---

### 2) `imu_reader`  — IMU HTTP JSON → `/imu_odom`
**Inputs:** HTTP endpoint providing `{r,p,y,gx,gy,gz,ax,ay,az}` (your `T=126`)  
**Optional input:** `/wheel_cmd` (for ZUPT/CUPT gating of velocity integration)  
**Outputs:** `nav_msgs/Odometry` on **`/imu_odom`** (+ optional TF `odom→base_link`)

- Orientation is corrected for **IMU mounting** and optional **auto-level** (gravity alignment).
- Twist:
  - `linear.x` = drift-limited forward speed (from accel + gating) **or** raw accel in debug mode
  - `angular.z` = yaw rate (rad/s) from gyro

**Key params**
- Connection & units:  
  `http_url`, `request_mode` (`plain|param`), `poll_json` (default `{"T":126}`), `timeout_s`, `rate_hz`,  
  `rpy_is_deg` (True), `gyro_is_deg` (True), `accel_is_mg` (True)
- Mounting & leveling:  
  `mount_roll_deg`, `mount_pitch_deg`, `mount_yaw_deg` (default 180 yaw),  
  `apply_mounting_tf_in_odom` (True), `auto_level` (True), `level_*` thresholds/samples
- Filtering & velocity:  
  `accel_filtering` (True), `sg_window_length`, `sg_poly_order`,  
  `wheel_radius_m`, `zupt_speed_threshold_mps`, `constant_velocity_threshold`
- Topics & frames:  
  `imu_topic` (`/imu_odom`), `wheel_cmd_topic` (`/wheel_cmd`),  
  `publish_tf` (False), `odom_frame` (`odom`), `base_link_frame` (`base_link`)
- Debug: `raw_accel_debug_mode` (publish raw accel into twist)

---

### 3) `ekf_odom`  — fuse `/imu_odom` + `/wheel_cmd` → `/ekf_odom`
**Inputs:**  
- `nav_msgs/Odometry` on **`/imu_odom`** → measurements `φ̃, ũ, r̃` (yaw, forward speed, yaw-rate)  
- `geometry_msgs/Vector3Stamped` on **`/wheel_cmd`** → `ω_L, ω_R` (rad/s)

**Outputs:**  
- `nav_msgs/Odometry` on **`/ekf_odom`** (pose x,y,φ and twist u,r)  
- **TF** `odom→base_link` (enabled by default)

**State/model**
- State `z = [x, y, φ, u, r]ᵀ`  
- Controls from wheel speeds: `u = ½ρ(ω_L+ω_R)`, `r = (ρ/2l)(ω_R-ω_L)`  
- Measurements `y = [φ̃, ũ, r̃]ᵀ` from `/imu_odom`

**Key params**
- Topics & frames: `imu_odom_topic` (`/imu_odom`), `wheel_cmd_topic` (`/wheel_cmd`),  
  `ekf_odom` (`/ekf_odom`), `publish_tf` (True), `odom_frame`, `base_link_frame`
- Model: `rho` (wheel radius, m), `half_track_l` (m)  
- Covariances: `q_*` (process diag for x,y,φ,u,r), `r_*` (meas diag for φ,u,r)  
- Init: `x0,y0,phi0,u0,r0`, `p0_*`, `auto_init_from_first_measure`  
- Runtime: `ekf_rate_hz`, `log_every_n`, `cmd_buffer_size`

---

## Bringup

**Everything on one machine**
```bash
ros2 launch rover_odom bringup.launch.py start_teleop:=true
```
Starts:
1) `sllidar_ros2` driver → `/scan`  
2) `imu_reader` → `/imu_odom`  
3) `ekf_odom` → `/ekf_odom` (+ TF)  
4) `slam_toolbox` (sync) + `nav2_lifecycle_manager`  
5) `robot_state_publisher` with `urdf/rover.urdf.xacro`  
6) **Teleop** (WASD) in a separate terminal  
7) *(RViz config exists; launching is currently commented in the file — start RViz manually if you like.)*

---

## Topics (main)

- `/wheel_cmd` — `Vector3Stamped` (`x=ω_L`, `y=ω_R` in rad/s, `z=Δt_since_change`)
- `/imu_odom` — `Odometry` (IMU-derived; forward speed + yaw-rate)
- `/ekf_odom` — `Odometry` (fused)
- `/scan` — LiDAR (from `sllidar_ros2`)
- `/tf`, `/tf_static`

---

## Frames

- `map` — from SLAM (dynamic `map→odom`)  
- `odom` — EKF “world” frame (drift-limited)  
- `base_link` — robot body frame

> Ensure **exactly one** `odom→base_link` TF source. By default, **EKF** publishes it; keep `imu_reader.publish_tf:=false`.

---

## Tuning notes

- Set your wheel geometry (`rho`, `half_track_l`) and measurement/process diagonals.  
- If heading drifts: adjust `r_phi` (measurement) and/or `q_phi` (process).  
- If position feels sticky: increase `q_x`, `q_y`.  
- Teleop speed mapping is already in **rad/s**; adjust the fitted lines in `rover_teleop` if your hardware changes.

---

## Dependencies

- ROS 2 Jazzy
- `slam_toolbox`, `nav2_lifecycle_manager`, `robot_state_publisher`, `tf2_ros`, `xacro`
- **LiDAR driver**: `sllidar_ros2` (build from source)
- Python libs: `numpy`, **`requests`** (used by teleop)

> **Note:** Because `rover_teleop` imports `requests`, keep `python3-requests` in your `package.xml` (`<exec_depend>python3-requests</exec_depend>`).

---

## Known limitations

- IMU forward speed is drift-limited (simple integration with smoothing & gating).  
- EKF assumes diff-drive kinematics (no lateral velocity state).  
- EKF steps on new IMU timestamps; wheel commands are aligned to the latest command at/before that time.

---

## License

MIT
