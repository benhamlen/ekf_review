# Context for AI coding agents

This file gives you background. **It does not override the task you're given.** If the user or the project instructions say otherwise, follow them.

## Situation
- The user, Ben, is at an **all-day onsite interview at Starpath Robotics** (Tue Oct 13, 2026) for **Senior Robotics Software Engineer: Lunar Rover Localization**.
- Confirmed format: a programming task using **ROS 2 (Jazzy) with Python**, a **system design task**, and an end-of-day **presentation to the whole team** on what was done, why, and what worked and what didn't.
- Everything Ben produces today will be presented and defended by him personally. **He must understand every line.** Favor clear, conventional code and short explanations over clever or large generated solutions. Be explicit about assumptions vs. confirmed facts.

## About Ben
- Robotics engineer with an MS. Background in perception, stereo visual odometry, geometric CV, camera calibration, point-cloud registration (John Deere autonomy), plus flight software, autonomy, embedded and real-time systems (Inversion Space).
- Fluent in Python, NumPy, SciPy, OpenCV, C++, Rust, ROS tooling, Rerun.
- **Spent the two days before the onsite refreshing EKF and localization theory.** He is solid on the concepts but still re-sharpening some implementation details, so a quick sanity check on math (Jacobians, covariance shapes, frame conventions) is welcome.

## How to work with him
- Keep responses **concise**. Explain the *why* briefly, at the level of an experienced engineer refreshing grad material.
- Prefer **incremental, reviewable changes**. Don't rewrite working code he wrote unless asked.
- When reviewing his code, call out real bugs first, especially math, shapes, frames and timing, then minor issues.
- Use meaningful names, type hints, `np.linalg.solve` instead of `inv`, and Joseph-form covariance updates.
- Point out interview-relevant tradeoffs he could mention in the presentation.
- Time is limited: **a working, explainable result beats polish.**

## What this repo contains (his prep work)
- `docs/REFERENCE_3D.md`: planar → 3D error-state EKF (SO(3), IMU kinematics, 3D measurement Jacobians, observability, ROS 3D conventions).
- `docs/REFERENCE.md`: equations, matrix meanings, consistency tests, Q derivation, terrain-registration math, frames, lunar facts, `rclpy` cheat sheet, system-design and presentation checklists. **Read this for vocabulary and conventions.**
- `ekf_review_refined.py`: generic EKF (`predict(u, dt)`, `update(meas)`) with plug-in `MotionModel` (f, F, Q, boxplus) and `Measurement` (z, R, h, H, residual) dataclasses. Planar unicycle model with gyro bias, state `[x, y, θ, v, b_g]`.
- `sim.py`: World → Sensors (gyro as process input, star-tracker heading as measurement) → EKF vs dead reckoning.
- `evaluation.py`, `plot_eval.py`: errors, ±3σ, NEES vs χ² bounds.
- `viz_viser.py`: live Viser viewer.
- `docs/PLAN.md`: prep plan and the talking points he collected.
- `docs/ARCHITECTURE.md` (repo structure), `docs/ADDING_SENSORS.md` (sensor recipe + tested examples), `docs/ROS2_INTEGRATION.md` (ROS mapping + local workspace notes), `docs/SYSTEM_DESIGN.md` (design strawman). Index: `docs/README.md`.
- Environment: pixi (`pixi run python ...`), Python 3.15, numpy, scipy, matplotlib, viser.

Reusing pieces (EKF class, evaluation, viewer) is fine if it fits the onsite task. Don't force them in if it doesn't.

## Conventions he has been using
- Gyro = **process input** (noise in Q). Absolute sensors (star tracker, DEM registration) = **measurements** (noise in R).
- Angles: wrap differences. Model provides `x ⊞ δ`, measurement provides `z ⊟ ẑ`.
- Frames per REP-105: `map → odom → base_link`, one publisher per transform.
- Evaluate against ground truth when available. Use NIS on real data.
