# Starpath Onsite Prep — Plan & Tracker

**Onsite:** Tue Oct 13, 2026, arrive ~11 AM — Senior Robotics SWE, Lunar Rover Localization.

**Confirmed format (from the invite email):**
- Tour, then a day-long task "representative of a task you would do if hired."
- **A programming task using ROS 2 with Python** and **a system design task.**
- End of day: **presentation to the entire group** — what you did, why, what worked and what didn't.

Topic guesses are still *hypotheses*: EKF fusion, ROS 2 localization nodes, LiDAR↔orbital-DEM registration, drift correction, accuracy/uncertainty/failure-mode evaluation.

**The sentence to be able to say on Tuesday:**
> "I built a modular, sensor-agnostic EKF, validated it against ground truth with consistency checks, and can show interactively how each noise source and sensor affects the estimate — and how terrain registration plugs in as just another measurement."

---

## Roadmap

| Day | Focus | Status |
|---|---|---|
| Sun Oct 11 | EKF math + standalone sim → playground refactor → wheel odom / global fixes → DEM registration | 🔄 in progress |
| Mon Oct 12 | Revised after invite email — see "Monday plan" below | 🔄 |
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

## Monday plan (revised to match the invite)

The email confirms **ROS 2 + Python**, **system design**, and a **presentation**. Shift effort from playground polish toward those three. Time-boxed, in order:

| # | Block | Why (maps to invite) | Time | Status |
|---|---|---|---|---|
| 1 | Finish step 5 (`evaluation.py`) + one NEES-driven `Q` experiment | Evaluation + "what worked / what didn't" evidence | 1h | 🔄 |
| 2 | **ROS 2 Python drill**: inspect lunar-sim workspace; wrap the EKF in an `rclpy` node — sub `sensor_msgs/Imu` → predict, sub heading/other → update, pub `nav_msgs/Odometry` + TF `odom→base_link`; params YAML; launch file; compare to sim ground truth; `ros2 bag` | **Programming task** — be fast at rclpy boilerplate, QoS, TF2, launch | 3h | ⬜ |
| 3 | **System design prep**: one-page lunar-rover localization architecture (sensors, frames, rates, fault handling, compute, comms latency, bootstrapping absolute position) — practice talking it through in 5 min | **System design task** | 1.5h | ⬜ |
| 4 | DEM registration — concept-level, plus a toy `least_squares` fit + covariance/degeneracy check if time remains | Likely topic; strong design-discussion material | 1h | ⬜ |
| 5 | Presentation skeleton + one dry run | **End-of-day presentation** | 30m | ⬜ |
| 6 | Reference sheet (`REFERENCE.md`) + agent context (`AGENTS.md`) | Day-of lookup | — | ✅ (C) |

**Dropped / deferred:** Viser parameter sliders, wheel-slip sim, Phase 4 extras. Don't polish the playground further.

### Monday-night triage (energy-limited; ~75 min, then sleep)
| # | Block | Time | Status |
|---|---|---|---|
| 1 | Fix `sigmas` (`np.sqrt`, not `**2`), rerun `plot_eval.py`, commit + push | 10m | ⬜ |
| 2 | **ROS smoke test:** read `ros2_template/README.md`, copy into `~/rover_ws/src`, build, run `ros2 launch ekf_loc ekf_loc.launch.py` against the Gazebo sim, check in RViz | 30m | ⬜ (template ✅ builds + passes fake-input test) |
| 3 | Talk through `docs/SYSTEM_DESIGN.md` **out loud**, 5-min timer | 15m | ⬜ |
| 4 | Skim `docs/PRESENTATION_TEMPLATE.md`; rehearse 2–3 of the "likely questions" | 10m | ⬜ |
| 5 | Sleep — arrive 11 AM fresh | — | ⬜ |

**Skipped, with fallbacks:** Q/NEES experiment (result + explanation already in hand: NEES ≈ 0.18 → Q too conservative, mainly unobserved v); DEM-registration code (concepts in `REFERENCE.md` §5, `REFERENCE_3D.md` §7); full 3h ROS drill (replaced by template + smoke test).

**Day-of:** ask at the start whether personal notes / LLM use are OK; keep `ONSITE_LOG.md`; code freeze 60–90 min before presenting.

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
| 1 | Dataclasses: `Control(v, ω)`, `TruthState`, `ProcessInput`, `Measurement(z, R, h, H, residual)`, `EstimateSnapshot`, Protocols (`MotionModel`, `Sensor`, `ControlSource`) | B | "Every sensor reduces to z, h(x), R" — loose coupling; DEM registration = just another sensor | 20m | ✅ |
| 2 | `EKF` class: `predict(u, dt)`, generic `update(meas)`; angles via model `boxplus` (x ⊞ δ) + measurement `residual` (z ⊟ ẑ) — no angle logic in EKF; single-step regression test passes | B | Nonlinear measurements; ⊞/⊟ form extends to SO(3) star-tracker attitude | 30m | ✅ |
| 3 | `World` (online truth integration from `ControlSource(t)`) + `Sensor`s with own rate/noise → `Measurement \| None` | C | Mirrors ROS node split; time-varying ω; truth ≠ filter integrator | 30m | ✅ `sim.py` |
| 4 | Regression: port loop, confirm DR radius ≈ 8.33 m and EKF tracks truth | C | "Refactored without behavior change, and here's how I checked" | 10m | ✅ truth R=10.000, DR R=8.339 (exp. 8.333), EKF final err 0.014 m vs DR 16.3 m |
| 5 | `History` + evaluation: heading err ±3σ, bias vs truth, position err, NEES | B (math), C (plots) | **Highest value:** consistency, bias observability, unbounded position error without position fixes | 45m | ⬜ |
| 6 | Viser front end: trails, heading arrows, 3σ ellipse; sliders for ω, bias, σ_gyro, σ_heading, update rate; reset | C (scaffold), B (wiring) | Live demo: "drop the update rate, watch P grow" | 1h | 🔄 `viz_viser.py` scaffold done (trails, arrows, 3σ ellipses, update stars, playback speed); sliders for sim params pending |

**Cut line:** 1–5 are must-haves. 6 can shrink to sliders + 2D trail.

---

## Open questions / predictions to check
- Bias estimate ends at 0.0162 vs true 0.02 after 120 s — is that inside its 3σ? (check in step 5; likely Q on bias + heading-only observability)
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
