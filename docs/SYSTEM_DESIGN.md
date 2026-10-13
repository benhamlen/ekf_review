# System design strawman: lunar rover localization

A **starting point to adapt**, not an answer to memorize. On the day, the real prompt and constraints take priority. Every hardware detail here is an **assumption**. The sensor list (360° LiDAR reliable to about 35 m, IMU, sun sensors, star tracker, teleop camera) came from an interviewer's example and isn't confirmed hardware.

Aim to talk through this in about 5 minutes, then go deep wherever they probe.

---

## 1. Clarify first (ask, don't assume)
- **What accuracy, and for what?** Driving safety (decimeters, locally consistent) vs science site return (meters, globally) vs docking or sample hand-off (centimeters, relative).
- **Operating region:** polar (low sun, long shadows, permanently shadowed regions) or equatorial? Day only, or night as well?
- **Autonomy level and comms:** how often is there an Earth link? Is ground-in-the-loop re-localization allowed?
- **Compute and power:** what processor (rad-tolerant means slow)? What CPU budget for localization?
- **Prior maps:** which DEM and orthophotos, at what resolution and accuracy, in which frame (ME vs PA)?
- **Speed and driving pattern:** continuous driving or stop-and-go? (Stop-and-go allows zero-velocity updates and registration while stationary.)

## 2. Layered architecture

```
           ┌──────────────── onboard ────────────────────────────────────────┐
IMU 100–200 Hz ─┐
Wheel odom 50 Hz┼─▶ LOCAL ES-EKF (odom frame) ─▶ odom→base_link ─▶ control, hazard avoidance
Star tracker 1–10 Hz ─┘   15-state: p, v, δθ, b_g, b_a
Sun sensor + gravity ─┘   (fallback attitude)
                                 │ prior pose + cov
LiDAR 10 Hz ─▶ local DEM mosaic ─▶ GLOBAL REGISTRATION vs orbital DEM (0.1–1 Hz, or when stopped)
                                 │ pose + cov (+ degeneracy flags)
                                 ▼
                    map→odom correction (or global EKF / sliding-window smoother)
           └────────────────────────────────────────────────────────────────┘
Telemetry: pose, covariance, innovations, NIS, registration quality ─▶ ground
Ground: orbital-image re-localization, map updates, anomaly review
```

**Why two layers:** the local estimate must be **smooth** for control; the global estimate is **accurate but can jump**. The REP-105 `map → odom → base_link` split gives you both (see `REFERENCE.md` §6).

## 3. Sensor roles

| Sensor | Gives | Role | Main failure modes |
|---|---|---|---|
| IMU | angular rate, specific force | process input; roll/pitch from gravity | bias drift, temperature |
| Wheel odometry | body velocity | measurement (or input) + non-holonomic constraint | **slip and sinkage in regolith**, wheel lift |
| Star tracker | inertial attitude → local heading (via ephemeris and position) | absolute attitude measurement | sun or Earth in the field of view, dust, needs time sync |
| Sun sensor + gravity | 2 + 2 DOF → full attitude via TRIAD | backup heading | sun occluded or near zenith |
| LiDAR | local 3D geometry | scan-to-scan relative motion, **DEM registration for absolute position** | flat or repetitive terrain (degenerate), dust, range limit around 35 m |
| Camera | imagery | teleop now; future visual odometry, crater/landmark matching | lighting (polar shadows), compute |

## 4. Key design decisions (state the tradeoff)
- **Filter vs smoother:** use an EKF for the local layer (cheap, real-time). For the global layer, consider a sliding-window factor graph if delayed or relative constraints dominate, since it relinearizes and handles loop-like constraints. Tradeoff: compute and complexity.
- **Loose vs tight coupling for registration:** loose (a pose plus covariance into the EKF) is modular and testable. Tight (point residuals in the filter) is more accurate under degeneracy. Start loose, with honest covariance and degeneracy flags.
- **Bootstrapping:** start from the landing solution, then run a global DEM search over the uncertainty region (coarse-to-fine correlation, multi-hypothesis) and confirm before trusting the result.
- **Register while stopped:** better LiDAR data, no motion distortion, and the CPU is free.
- **Use attitude sensors to shrink registration to x, y, z (± yaw):** better conditioned and faster.

## 5. Fault detection and degraded modes
- **NIS gating** on every measurement, with logged rejections.
- **Degeneracy check** on registration via eigenvalues of `JᵀJ`: inflate or project weak directions, and report quality.
- **Slip detection:** wheel velocity vs IMU (and later VO) disagreement. Persistent NIS growth triggers a slow-down or stop.
- **Sensor timeouts** and covariance-growth alarms. When the global position σ exceeds a threshold, stop, re-localize, and possibly ask the ground for help.
- **Ambiguity:** multiple registration basins mean don't commit; keep multiple hypotheses or wait for more terrain.

## 6. Validation
- Simulation with ground truth: error, ±3σ, **NEES**, Monte-Carlo runs.
- Bag replay of field data: **NIS**, cross-check against a surveyed or GNSS reference at Earth analog sites.
- Sensitivity to DEM resolution and noise, LiDAR range, update rates, slip.
- Regression tests in CI; NEES/NIS thresholds as pass/fail.

## 7. What I'd build first (MVP → iterate)
1. Planar or 2.5D local EKF: IMU + wheel + star-tracker heading, with NEES validated in simulation.
2. LiDAR→DEM registration offline on recorded data, with covariance and degeneracy flags.
3. Fuse registration as a position measurement; show bounded error versus dead reckoning.
4. Then: full 3D error-state EKF, delayed-measurement handling, global initialization, factor graph if needed.

## 8. Presentation hooks (from this prep)
- "Heading-only updates bound heading and make gyro bias observable, but **nothing bounds position** without an absolute fix, so terrain registration is essential."
- "A low registration residual doesn't mean a good pose: I check the information matrix."
- "My filter is written in ⊞/⊟ form, so going from planar to SO(3) changes the model and the measurements, not the filter."
- "NEES showed my hand-tuned Q was far too conservative (mean around 0.2 against an expected 5). Next I'd derive Q from the IMU specs."
