# ros2_template/ekf_loc

A minimal working ROS 2 Jazzy (`ament_python`) package that wraps this repo's NumPy EKF in a node. It's meant as a **known-good starting point** for copying, adapting and extending, so you aren't writing boilerplate from scratch on the day.

**Verified (Oct 12):** builds with `colcon` against `/opt/ros/jazzy`, and runs end-to-end against fake IMU, wheel and ground-truth publishers. With a gyro biased by 0.02 rad/s, the star-tracker updates held yaw at 0.797 vs 0.800 true, and wheel updates gave v = 1.00. **Not yet run against the Gazebo sim.**

## Contents
```
ekf_loc/
├── ekf_loc/ekf_core.py           # copy of ../../ekf_review_refined.py + wheel_speed_measurement (no ROS imports)
├── ekf_loc/ekf_node.py           # Imu→predict, wheel Odometry→update, star_tracker Imu→heading update; pubs Odometry + TF
├── ekf_loc/fake_star_tracker.py  # ground-truth yaw + noise @ 1 Hz → sensor_msgs/Imu (orientation only)
├── config/ekf_loc.yaml           # all parameters (frames, noise, Q, P0)
├── launch/ekf_loc.launch.py      # both nodes + topic remaps for ~/rover_ws
├── package.xml, setup.py, setup.cfg, resource/ekf_loc
```

## Use it
```bash
# copy into a workspace (or symlink) and build with SYSTEM python (not pixi)
cp -r ~/ekf_review/ros2_template/ekf_loc ~/rover_ws/src/
cd ~/rover_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ekf_loc && source install/setup.bash

# with the sim running (rover_gazebo.launch.py):
ros2 launch ekf_loc ekf_loc.launch.py
ros2 topic echo /odometry/ekf --once
ros2 run tf2_ros tf2_echo odom base_footprint_ekf
# RViz: fixed frame odom; add Odometry displays for /odometry/ekf and /odometry/ground_truth
```

## Design choices (and how to explain them)
- **The EKF core has no ROS dependency** (`ekf_core.py`), so it's unit-testable and reusable. The node only converts messages into `ProcessInput` and `Measurement`.
- **The clock is message timestamps.** `dt` comes from IMU stamps, and gaps over 0.5 s (or ≤ 0) are skipped rather than integrated.
- **The gyro is a process input; wheel speed and heading are measurements** (Q vs R).
- **The child frame is `base_footprint_ekf`**, because `rover_localization/scripts/ground_truth_tf.py` already publishes `odom → base_footprint` and two publishers for one transform conflict. Switch to `base_footprint` (with `publish_tf: true`) only once ground-truth TF is off.
- **Covariance packing:** the x, y, θ block of P goes into ROS indices (0, 1, 5) of the 6×6 pose covariance. Unestimated z, roll and pitch get a variance of 1e6.

## Known simplifications (say them out loud)
- The IMU frame is assumed aligned with the base frame. Otherwise, rotate the gyro by the static TF `base → imu_link`.
- Measurements are applied at the current filter time, without predicting up to their stamp first. That's fine at these rates, but say so.
- Q and P0 are untuned starting values. Validate with NIS on the sim, or with NEES against `/odometry/ground_truth`.
- Planar (`two_d_mode`-like). See `docs/REFERENCE_3D.md` for full 3D.
- `ekf_core.py` is a copy, so changes to `ekf_review_refined.py` don't propagate to it automatically.
