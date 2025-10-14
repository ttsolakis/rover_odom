# rover_odom

ROS 2 (Jazzy) package for low-cost odometry and mapping. It polls an IMU over HTTP (JSON `T=126`), publishes `nav_msgs/Odometry`, fuses wheel commands with an EKF for improved state estimates, and brings up LiDAR + SLAM + RViz in one go.

**Pipeline**
```
IMU JSON  ──>  imu_reader ──── /imu_odom ─┐
                                          ├─>  ekf_odom  ── /ekf_odom (+ optional TF odom→base_link)
Wheel cmds ─>  rover_teleop ─ /wheel_cmd ─┘
```

---

## Features

- **IMU → Odometry**: converts IMU RPY/gyro/accel (via HTTP JSON) to `/imu_odom` with proper mounting and frame handling.
- **Auto-level & biasing**: optional tilt calibration from gravity and planar accel bias estimation (ZUPT).
- **EKF odometry**: fuses `/imu_odom` (yaw, fwd speed, yaw rate) with wheel commands to publish `/ekf_odom`.
- **Bringup**: LiDAR driver, `slam_toolbox` (auto lifecycle), `robot_state_publisher` (URDF), RViz config, optional WASD teleop.
- **TF**: single source of `odom→base_link` TF (by default from EKF to avoid conflicts).

---

## Quick start

```bash
# Build
colcon build --packages-select rover_odom
source install/setup.bash

# Full bringup (LiDAR + IMU odom + EKF + SLAM + RViz + optional teleop)
ros2 launch rover_odom bringup.launch.py start_teleop:=true
```

**Wheel teleop** opens in a separate terminal (WASD). Disable it with `start_teleop:=false`.

---

## Nodes

### 1) `imu_reader`
Polls an HTTP endpoint (your IMU) and publishes `nav_msgs/Odometry` on **`/imu_odom`**.  
Orientation is corrected for IMU mounting; gyro/accel are rotated into `base_link`.

**Publishes**
- `nav_msgs/Odometry` on **`/imu_odom`**  
  - `pose.orientation`: IMU attitude in `odom` frame (no integrated position here)  
  - `twist.linear.x/y`: planar velocity from accel integration (+ damping & ZUPT)  
  - `twist.angular.z`: yaw rate from gyro
- *(optional)* TF `odom → base_link` (disabled by default—EKF is preferred TF source)

**Important parameters**
- Connection & format:
  - `http_url` (default `http://192.168.4.1/js`)
  - `request_mode`: `plain` or `param`
  - `poll_json`: JSON body when `param` (default `{"T":126}`)
  - `timeout_s`, `rate_hz`
- Units:
  - `rpy_is_deg` (default `True`)
  - `gyro_is_deg` (default `True`)
  - `accel_is_mg` (default `True`)
- Mounting & calibration:
  - `mounting_rpy_deg` (default `[0,0,180]`) — IMU → base_link static rotation
  - `apply_mounting_tf_in_odom` (default `True`)
  - `auto_level` (default `True`), `level_*` thresholds/samples
  - `bias_samples`, `zupt_*` thresholds
  - `filter_alpha` (EMA prefilter), `velocity_damping_lambda` (leak to zero)
- Frames & topics:
  - `topic_name` (default `"/imu_odom"`)
  - `publish_tf` (default `False`)
  - `odom_frame` (default `"odom"`), `base_link_frame` (default `"base_link"`)

**Notes**
- We rotate IMU data into `base_link` **inside the message**, so consumers don’t need TF to interpret `/imu_odom`.
- Keep `publish_tf:=False` here if EKF is broadcasting TF to avoid duplicate `odom→base_link`.

---

### 2) `ekf_odom`
Discrete-time EKF for a diff-drive rover:

- **State**: `z = [x, y, φ, u, r]ᵀ`
- **Inputs**: wheel speeds (left/right), converted from teleop “units” → rad/s
- **Measurements**: `y = [φ̃, ũ, r̃]ᵀ` from `/imu_odom`
- **Outputs**: `nav_msgs/Odometry` on **`/ekf_odom`**, optional TF `odom→base_link`

**Subscriptions**
- `nav_msgs/Odometry` on **`/imu_odom`** (yaw, forward speed, yaw rate)
- `geometry_msgs/Vector3Stamped` on **`/wheel_cmd_units_stamped`**  
  - `vector.x = L`, `vector.y = R` (teleop units), `vector.z = duration_since_change [s]`

**Publishes**
- `nav_msgs/Odometry` on **`/ekf_odom`** with pose (x,y,φ) and twist (u, r)
- *(optional)* TF `odom → base_link` (enabled by default)

**Key parameters**
- Topics & frames:
  - `imu_odom_topic` (default `"/imu_odom"`)
  - `wheel_cmd_topic` (default `"/wheel_cmd_units_stamped"`)
  - `ekf_odom` (default `"/ekf_odom"`)
  - `publish_tf` (default `True`)
  - `odom_frame` / `base_link_frame`
- Model:
  - `rho` (wheel radius, m), `half_track_l` (m), `unit_to_radps` (teleop unit → rad/s)
- Covariances:
  - Process diag: `q_x,q_y,q_phi,q_u,q_r`
  - Measurement diag: `r_phi,r_u,r_r`
- Init:
  - `x0,y0,phi0,u0,r0`
  - `p0_x,p0_y,p0_phi,p0_u,p0_r`
  - `auto_init_from_first_measure` (default `True`)
- Runtime:
  - `ekf_rate_hz` (timer poll; steps only when new IMU stamp arrives)
  - `log_every_n`, `cmd_buffer_size`

**Odometry fields**
- Pose `(x,y,φ)` in `odom`
- Twist: `linear.x = u`, `angular.z = r` in `base_link`
- Covariances: relevant entries mapped from EKF `P`; unused axes set large (e.g., 1e3)

**TF**
- Prefer **only EKF** to publish `odom→base_link`. Ensure `publish_tf:=False` in `imu_reader`.

---

## Bringup

Launch everything together:

```bash
ros2 launch rover_odom bringup.launch.py start_teleop:=true
```

What it starts:

1. **SLLidar** driver (`sllidar_ros2`)
2. **IMU → `/imu_odom`** (`rover_odom/imu_reader.py`)
3. **EKF** (`rover_odom/ekf_odom.py`) → `/ekf_odom` (+ TF by default)
4. **slam_toolbox** (sync node) + **nav2_lifecycle_manager** (auto-start)
5. **robot_state_publisher** with `urdf/rover.urdf.xacro`
6. **RViz** with `rviz/rover_mapping.rviz`
7. **WASD teleop** (separate terminal) producing `/wheel_cmd_units_stamped`

**Useful args**
- `start_teleop:=false` — don’t spawn the teleop window
- Change IMU HTTP settings via `imu_to_odom.launch.py` parameters (e.g., `http_url`, `poll_json`, `rate_hz`)

---

## Topics

- `/imu_odom` — `nav_msgs/Odometry` (from IMU, planar velocities + orientation only)
- `/wheel_cmd_units_stamped` — `geometry_msgs/Vector3Stamped` (L,R units; `z` = duration since change)
- `/ekf_odom` — `nav_msgs/Odometry` (fused pose & twist)
- `/scan` — LiDAR (from SLLidar)
- `/tf`, `/tf_static`

---

## Frames

- `map` — provided by SLAM (dynamic `map→odom`)
- `odom` — EKF world frame (drift-limited)
- `base_link` — robot body frame

Ensure **exactly one** broadcaster for `odom→base_link` (EKF by default).

---

## Tuning tips

- **`unit_to_radps`**: set so a known wheel command produces the expected linear speed.
- Start with conservative **`R`** (measurement) and increase **`Q`** entries for states you want to be more driven by the model (e.g., `q_u`, `q_r` if wheel commands are trustworthy).
- If EKF position seems too “sticky,” raise `q_x`, `q_y`. If heading drifts, adjust `r_phi` and `q_phi`.
- IMU velocity integration is intentionally damped—EKF will rely on wheel-derived `u,r` for prediction while measurements stabilize heading and short-term speed.

---

## Example JSON (T=126)

```json
{ "r": 0.5, "p": -0.3, "y": 12.0, "gx": 0.1, "gy": -0.2, "gz": 0.8, "ax": 12.3, "ay": -3.1, "az": 1005.0 }
```

Units controlled via params: `rpy_is_deg`, `gyro_is_deg`, `accel_is_mg`.

---

## Dependencies

- ROS 2 Jazzy
- `slam_toolbox`, `nav2_lifecycle_manager`, `robot_state_publisher`, `tf2_ros`
- LiDAR driver: `sllidar_ros2`

---

## Known limitations

- IMU linear velocities are planar and drift-limited (simple integration with damping/ZUPT).
- EKF currently ignores lateral velocity; model assumes diff-drive kinematics.
- Timestamps: EKF steps on new IMU stamps and aligns to the latest wheel command at or before that time.

---

## License

MIT (or your project’s license).
