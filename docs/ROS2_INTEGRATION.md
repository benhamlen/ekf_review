# ROS 2 integration: wrapping this EKF in nodes

How the standalone architecture maps onto ROS 2 (Jazzy, `rclpy`), what already exists in Ben's local rover workspace, and a step-by-step practice plan. The `rclpy` boilerplate is in `REFERENCE.md` §8.

---

## 1. Standalone → ROS mapping

| Standalone (`sim.py`) | ROS 2 equivalent |
|---|---|
| `World` + `ControlSource` | Gazebo sim + `/cmd_vel` (teleop or a script) |
| `TruthState` | `/odometry/ground_truth` (Gazebo OdometryPublisher), for **evaluation only** |
| `GyroSensor` → `ProcessInput` | `/imu` (`sensor_msgs/Imu`) callback → `ProcessInput(t, msg.angular_velocity.z)` |
| `StarTrackerSensor` → `Measurement` | not simulated: write a **fake star-tracker node** (ground-truth yaw + noise → `sensor_msgs/Imu` orientation, or a custom topic) |
| wheel speed `Measurement` | `/diff_drive_controller/odom` twist → `wheel_speed_measurement(t, twist.linear.x, σ)` |
| `EKF.snapshot` | publish `nav_msgs/Odometry` + TF `odom → base_footprint` |
| `evaluation.py` | an eval node (or offline from a `ros2 bag`) comparing to ground truth |
| `ViserViz` | RViz2 (Odometry arrows, TF) or keep Viser/Rerun alongside |

**Timing rule:** the filter clock is **message timestamps** (`header.stamp`, with `use_sim_time: true`), never the wall clock. `dt` = current IMU stamp − previous IMU stamp. Before applying a measurement, predict up to the measurement's stamp, or accept a small timing error and say so.

---

## 2. What exists in `~/rover_ws` (inspected Oct 12)

- **Packages:** `rover_description` (xacro URDF), `rover_bringup` (Gazebo launch, worlds `terrain_world.sdf` and `sensor_world.sdf`), `rover_control` (ros2_control `diff_drive_controller`), `rover_localization` (robot_localization configs + scripts), `rover_mapping`.
- **Sensors in the URDF:** IMU (`/imu`, frame `imu_link`), RGB camera (`/camera/image_raw`), NavSat (`/gps`). **There is no LiDAR yet.**
- **Bridge topics** (`rover_gazebo.launch.py`): `/clock`, `/imu`, `/camera/image_raw`, `/camera/camera_info`, `/gps`, and `/model/rover/odometry` remapped to `/odometry/ground_truth`.
- **Wheel odometry:** `/diff_drive_controller/odom` with `enable_odom_tf: false`. Skid-steer with `wheel_separation_multiplier: 1.5`, which is effectively a slip fudge.
- **robot_localization:** `ekf_local.yaml` (world_frame odom, two_d_mode, odom0 = wheel vx and vyaw, `publish_tf: false`), `ekf_global.yaml`, `navsat_transform.yaml`. **Base frame is `base_footprint`.**
- **`scripts/ground_truth_tf.py`** publishes **`odom → base_footprint` from ground truth**. ⚠ If your EKF node also publishes that transform, the two will conflict. Disable one, or have your EKF publish under a different name for side-by-side comparison (e.g. `odom → base_footprint_ekf`).
- `~/lunar_ws` has `lunar_worlds` and `lunar_eval`; `~/ros2_ws` has `rover_demo`.

Re-check these before relying on them:
```bash
ros2 topic list; ros2 topic hz /imu; ros2 topic echo /imu --once
ros2 run tf2_tools view_frames
```

---

## 3. Node design for this repo's EKF

```
/imu ───────────────┐
/diff_drive/odom ───┼──▶ ekf_node ──▶ /odometry/ekf (nav_msgs/Odometry)
/star_tracker ──────┘        └──────▶ TF odom → base_footprint (or *_ekf for comparison)
/odometry/ground_truth ──▶ eval_node ──▶ /eval/pos_err, /eval/nees (std_msgs/Float64) + CSV
```

**`ekf_node` responsibilities:**
- Parameters: noise σ values, initial covariance, frame names, topic names.
- Callbacks:
  - IMU → `predict`. Rotate the gyro into the base frame with the static TF if the IMU is mounted rotated.
  - Wheel odom → wheel-speed update.
  - Star tracker → heading update.
- Publish an `Odometry` message after each predict (or on a timer). Pack x, y and yaw into the pose. Covariance: `[0]=Pxx`, `[1]=[6]=Pxy`, `[7]=Pyy`, `[35]=Pθθ`, and `[5],[30],[11],[31]` for the xy–θ cross terms.
- Keep `EKF` in a separate importable module, so it's unit-testable without ROS.

**Quaternion ↔ yaw:** `q = (x=0, y=0, z=sin(ψ/2), w=cos(ψ/2))`; `ψ = atan2(2(w z + x y), 1 − 2(y² + z²))`.

---

## 4. Practice plan (about 3 h)

1. **(20 m)** Launch the sim and list topics. Confirm IMU rate, frames, and whether `/imu` angular velocity is in `imu_link` or the body frame.
2. **(20 m)** Create the package: `ros2 pkg create --build-type ament_python --dependencies rclpy sensor_msgs nav_msgs geometry_msgs tf2_ros -- ekf_loc`. Copy in `ekf_review_refined.py`. Get a node that just logs IMU dt running via `ros2 run`.
3. **(40 m)** IMU → predict, then publish `Odometry` and a TF under a non-conflicting child frame. Drive with teleop and watch it in RViz.
4. **(30 m)** Add the wheel-speed update from `/diff_drive_controller/odom`.
5. **(30 m)** A fake star-tracker node: subscribe to ground truth, extract yaw, add noise, publish at 1 Hz. Add the heading update.
6. **(30 m)** Eval: record a bag (`ros2 bag record /odometry/ekf /odometry/ground_truth`) and compute error and NEES offline with `evaluation.py`-style code. Or write a live `eval_node`.
7. **(10 m)** A launch file with a params YAML that starts all of it.

**Common pitfalls:**
- QoS mismatch: the IMU from the bridge may be best-effort. Use `qos_profile_sensor_data`.
- Forgetting `use_sim_time`.
- Two TF publishers for one transform.
- `base_link` vs `base_footprint`.
- Using `time.time()` instead of stamps.
- Ignoring the first-message `dt` (no previous stamp yet).
- Not sourcing `install/setup.bash` after `colcon build`.

---

## 5. Adding a LiDAR (for DEM-registration practice)

Add a `gpu_lidar` sensor to the xacro: about 35 m range, 360° horizontal, a handful of vertical channels. Bridge it as `sensor_msgs/msg/PointCloud2` (Gazebo `gz.msgs.PointCloudPacked`) or `LaserScan`. You'll also need a DEM: export the terrain heightmap used by `terrain_world.sdf` as a NumPy grid. Registration then becomes a node that subscribes to the cloud, uses the EKF prior as its initial guess, and publishes a pose with covariance. That pose either feeds the EKF as a `position_fix_measurement`, or drives `map → odom`.
