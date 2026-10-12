# Starpath Onsite Prep — Plan & Tracker

**Onsite:** Tue Oct 13, 2026 — Senior Robotics SWE, Lunar Rover Localization (all-day project + presentation).
Exercise topics are *hypotheses*, not confirmed: EKF fusion, ROS 2 localization nodes, LiDAR↔orbital-DEM registration, drift correction, accuracy/uncertainty/failure-mode evaluation.

**The sentence to be able to say on Tuesday:**
> "I built a modular, sensor-agnostic EKF, validated it against ground truth with consistency checks, and can show interactively how each noise source and sensor affects the estimate — and how terrain registration plugs in as just another measurement."

---

## Roadmap

| Day | Focus | Status |
|---|---|---|
| Sun Oct 11 | EKF math + standalone sim → playground refactor → wheel odom / global fixes → DEM registration | 🔄 in progress |
| Mon Oct 12 | ROS 2 Jazzy integration with existing lunar rover sim, evaluation, practice explaining | ⬜ |
| Tue Oct 13 | Onsite | ⬜ |

### Phase 1 — EKF fundamentals ✅
- [x] Planar EKF, state `[x, y, θ, v, b_g]`, gyro as process input
- [x] Jacobian `F` at pre-prediction state; `P⁻ = F P Fᵀ + Q`
- [x] Heading update: wrapped innovation, `np.linalg.solve` gain, Joseph-form covariance, re-normalize θ
- [x] Verified: heading update corrects position and bias via cross-covariance

### Phase 2 — Standalone simulation ✅ (`ekf_review.py`)
- [x] Closed-form circular truth (R = v/ω = 10 m), seeded RNG
- [x] Gyro = ω_true + bias + noise (process input); star-tracker heading = θ_true + noise @ 1 Hz
- [x] EKF vs dead reckoning (same gyro, DR gets no updates); time-aligned indexing (iteration k: t_{k−1} → t_k)
- [x] Live XY plot with update markers
- [x] Sanity check: DR circle radius ≈ 1/(0.1+0.02) ≈ 8.33 m ✔
- [ ] Save history (folded into Phase 3 step 5)

### Phase 3 — EKF playground refactor 🔄 (`ekf_review_refined.py`) — *detailed below*

### Phase 4 — More sensors & evaluation ⬜
- [ ] Wheel-speed measurement + slip (new `Sensor`, zero EKF changes → proves the interface)
- [ ] Occasional global position fixes → bounded position error
- [ ] Physically derived, dt-dependent `Q` (noise-input Jacobian, gyro ARW, bias random walk)
- [ ] Compare consistency (NEES) across configurations

### Phase 5 — Terrain registration ⬜
- [ ] Synthetic DEM + simulated LiDAR scan (≤35 m reliable range)
- [ ] LiDAR→DEM translation (then + yaw) via `scipy.optimize.least_squares`
- [ ] Registration covariance `(Jᵀ Σ⁻¹ J)⁻¹`; degeneracy detection via eigenvalues (flat plane, ridge)
- [ ] Feed result into EKF as a `Measurement` (inflate/reject degenerate directions)
- [ ] Viser: DEM mesh + matched points

### Phase 6 — ROS 2 integration (Mon) ⬜
- [ ] Inspect existing workspace: packages, topics, which sensors actually exist
- [ ] Local odometry node (IMU + wheel) → `nav_msgs/Odometry`, `odom → base_link`
- [ ] Global registration node → `map → odom` correction (no conflicting TF publishers)
- [ ] Evaluation vs sim ground truth; RViz / Foxglove / Rerun
- [ ] Practice the end-of-day presentation: approach, results, tradeoffs, failure modes

---

## Phase 3 detail — Playground refactor

**Rules**
- EKF never imports truth; only `History` sees both.
- Core math runs and tests without any UI (Tuesday needs the algorithm, not the playground).
- No behavior changes during the refactor — same model, same `Q`.
- Owner key: **B** = Ben writes it, **C** = Claude scaffolds/reviews.

```
ControlSource ──u──▶ World (truth) ──truth──▶ Sensors ──Measurement──▶ EKF ──▶ History ──▶ Visualizer (Viser)
```

| # | Step | Owner | Interview payoff | Est. | Status |
|---|---|---|---|---|---|
| 0 | Verify Viser installs under pixi — viser 1.1.1 on Py 3.15 OK; server, slider, point cloud, `add_plotly`/`add_uplot` all work | C | Kill tooling risk early | 10m | ✅ |
| 1 | Dataclasses: `Control(v, ω)`, `TruthState`, `Measurement(z, R, h, H, angle_idx)` — sketch, then review | B | "Every sensor reduces to z, h(x), R" — loose coupling; DEM registration = just another sensor | 20m | ⬜ |
| 2 | `EKF` class: `predict(u, dt)`, generic `update(meas)` (innovation `z − h(x)`, wrap by `angle_idx`, H at x) | B | Nonlinear measurements; registration poses, landmark ranges | 30m | ⬜ |
| 3 | `World` (online truth integration from `ControlSource(t)`) + `Sensor`s with own rate/noise → `Measurement \| None` | B | Mirrors ROS node split; time-varying ω; truth ≠ filter integrator | 30m | ⬜ |
| 4 | Regression: port loop, confirm DR radius ≈ 8.33 m and EKF tracks truth | B | "Refactored without behavior change, and here's how I checked" | 10m | ⬜ |
| 5 | `History` + evaluation: heading err ±3σ, bias vs truth, position err, NEES | B (math), C (plots) | **Highest value:** consistency, bias observability, unbounded position error without position fixes | 45m | ⬜ |
| 6 | Viser front end: trails, heading arrows, 3σ ellipse; sliders for ω, bias, σ_gyro, σ_heading, update rate; reset | C (scaffold), B (wiring) | Live demo: "drop the update rate, watch P grow" | 1h | ⬜ |

**Cut line:** 1–5 are must-haves. 6 can shrink to sliders + 2D trail.

---

## Open questions / predictions to check
- Does EKF position error stay bounded with heading-only updates? (Prediction before plotting.)
- How fast does the bias estimate converge, and how does that depend on σ_heading and update rate?
- Is the filter consistent with the current hand-tuned `Q`? (NEES should land in its χ² bounds.)

## Interview talking points (collect as we go)
- Gyro as process input → its noise lives in `Q`, not `R`.
- Cross-covariance lets a heading measurement correct position and bias even though `H` has zeros there.
- Bias makes DR heading error grow linearly; white noise grows ~√t.
- Relative (gyro, wheel, scan-to-scan) vs absolute (star tracker, DEM registration) sensors — fusion = fast relative between slow absolute.
- Low registration residual ≠ well-constrained pose; flat terrain leaves x/y unobservable.
- Star tracker gives inertial attitude; needs frame transform to lunar-local. Sun sensor constrains 2 of 3 rotational DOF.
- Facts about Starpath's rover vs assumptions: sensor list came from an interviewer example — not confirmed hardware.
