# ekf_review: Architecture

How the code in this repo is organized: what each piece does, how data flows between them, and the rules that keep the pieces separate.

```
ControlSource ──Control──▶ World ──TruthState──▶ Sensors ──ProcessInput──▶ EKF.predict
   (truth input)          (truth)                   │                          │
                                                    └──Measurement──▶ EKF.update
                                                                               │
                                       TruthState ──┐            EstimateSnapshot
                                                    ▼                          ▼
                                        SimResult (history) ──▶ evaluation.py ──▶ plot_eval.py
                                                                     (errors, σ, NEES)    (eval.png)
                                        live: ViserViz.log_step(truth, snapshots)
```

**Design rules:**
1. **The EKF never sees truth.** Only `World`, the sensors and evaluation read `TruthState`.
2. **The EKF knows nothing about rovers.** All rover-specific math lives in the `MotionModel` (f, F, Q, ⊞) and in each `Measurement` (z, R, h, H, ⊟). Angle handling lives there too.
3. **Snapshots are copies.** History entries can't be changed by later filter steps.
4. **Core math has no UI dependency.** The viewer and plots only read snapshots.

---

## Files

| File | Role |
|---|---|
| `ekf_review_refined.py` | **Core.** Data types, Protocols, `UnicycleBiasModel`, generic `EKF`, `normalize_angle`, `heading_measurement`. Its `__main__` block is a single-step regression test |
| `sim.py` | **Simulation.** `ConstantControl`, `World`, `GyroSensor`, `StarTrackerSensor`, `run_sim() -> SimResult`, regression checks. `--no-viz` runs headless |
| `evaluation.py` | **Metrics** (pure NumPy): `truth_state_vectors`, `state_errors`, `sigmas`, `nees` |
| `plot_eval.py` | **Static plots:** heading error ±3σ, bias convergence, position error (log scale), NEES vs χ² bounds → `eval.png` |
| `viz_viser.py` | **Live viewer** (Viser, http://localhost:8080): trails, heading arrows, 3σ ellipses, update markers, playback speed |
| `ekf_review.py` | The original procedural version, kept as a reference |
| `ros2_template/ekf_loc/` | ROS 2 Jazzy package wrapping the EKF (`ekf_node`, `fake_star_tracker`, params, launch). See its README |
| `docs/` | Plan, reference sheets, this document |

Run:
```bash
pixi run python ekf_review_refined.py   # single-step regression test (asserts)
pixi run python sim.py --no-viz         # headless sim + regression checks
pixi run python sim.py                  # live Viser view
pixi run python plot_eval.py            # evaluation figure
```

---

## Data types (`ekf_review_refined.py`)

All are frozen dataclasses: facts that never change after creation.

| Type | Fields | Who creates it | Who consumes it |
|---|---|---|---|
| `Control` | `v`, `omega` | `ControlSource` | `World` (truth only) |
| `TruthState` | `t, x, y, theta` (unwrapped)`, v, omega` | `World` | sensors, evaluation, viewer |
| `ProcessInput` | `t, gyro_z` | `GyroSensor` | `EKF.predict` → `MotionModel` |
| `Measurement` | `t, z (m,), R (m,m), h(x), H(x), residual(z, ẑ), source` | measurement sensors / helper builders | `EKF.update` |
| `EstimateSnapshot` | `t, x (n,), P (n,n)` | `EKF.snapshot` | history, evaluation, viewer |

`Vec = NDArray[np.float64]`. The default `residual` is `subtract_residual` (plain z − ẑ); angular measurements pass a wrapping residual instead.

## Protocols (interfaces)

Any class with these methods fits the interface; no inheritance is needed.

```python
class MotionModel(Protocol):
    def f(self, x, u: ProcessInput, dt) -> Vec        # propagate state
    def F(self, x, u, dt) -> Vec                       # ∂f/∂x at the prior x
    def Q(self, x, u, dt) -> Vec                       # process noise for this step
    def boxplus(self, x, dx) -> Vec                    # x ⊞ dx, keeps x valid (wraps θ)

class Sensor(Protocol):
    def sample(self, truth: TruthState) -> Measurement | None   # None = not this step

class ControlSource(Protocol):
    def command(self, t) -> Control
```
`GyroSensor.sample` returns a `ProcessInput` rather than a `Measurement`, so it deliberately doesn't satisfy `Sensor`. That's the input-vs-measurement distinction.

## Components

### `UnicycleBiasModel` (implements `MotionModel`)
- State `[x, y, θ, v, b_g]`. Heading is driven by `u.gyro_z − b_g`, and v is constant.
- `f` wraps θ. `boxplus` = `x + dx` with θ wrapped.
- `Q` is currently a **constant hand-tuned matrix**. NEES shows it's far too large: mean NEES ≈ 0.18, where 5 is expected. The obvious next improvement is a spec-derived, dt-dependent Q.

### `EKF`
- `__init__(model, x0, P0)` stores float copies.
- `predict(u, dt)` computes `F`, `Q` at the prior x, then `x = f(x)` and `P = FPFᵀ + Q`.
- `update(meas)`:
  1. `y = residual(z, h(x))`, `H = H(x)`
  2. `S = HPHᵀ + R`, `K = solve(S, HP)ᵀ`
  3. `x = boxplus(x, Ky)`
  4. Joseph-form P
- `snapshot(t)` returns an `EstimateSnapshot` with copies of x and P.
- Dead reckoning is just a second `EKF` that never receives `update` calls.

### Simulation (`sim.py`)
- `World` holds the true `[x, y, θ]`. `set_control` sets the true v and ω for the coming interval; `step(dt)` integrates in `TRUTH_SUBSTEPS` Euler sub-steps, so truth doesn't use the EKF's integrator.
- `GyroSensor(bias, sigma, rng)` returns `ProcessInput` with `ω + bias + N(0, σ²)`.
- `StarTrackerSensor(sigma, period_steps, rng)` fires every Nth call and returns `heading_measurement(...)` with a wrapped θ.
- **Loop timing:** set control, sample the gyro at t_{k−1}, step truth to t_k, predict both filters, maybe update the EKF with the measurement at t_k, then snapshot.
- `SimResult(truth, ekf, dr, ekf_updated, bias_true)`.

### Evaluation and plots
- `state_errors` = estimate − truth (truth gets `bias_true` appended), with the θ column wrapped.
- `sigmas` = √diag(P).
- `nees` = eₖᵀPₖ⁻¹eₖ via `solve`.
- `plot_eval` uses `scipy.stats.chi2` for the 95% bounds at dof = 5.

### Viewer (`viz_viser.py`)
- `ViserViz.log_step(truth, {name: snapshot}, updated=[names])`. It redraws every `redraw_every` steps and sleeps according to the playback-speed slider.
- `hold()` keeps the server alive after the run.
- It reads x, y and θ at indices 0, 1, 2, so it's tied to the planar layout.

---

## Extension points

| Want to… | Change |
|---|---|
| Add a sensor | New `Measurement` builder + a sensor class. **No EKF changes.** See `ADDING_SENSORS.md` |
| Add a state (e.g. wheel scale factor, z) | New `MotionModel` (f, F, Q, boxplus) + update every `H` to the new n. Evaluation needs the truth layout extended |
| Better Q | `MotionModel.Q(x, u, dt)` already gets x, u, dt; derive it as G Q_c Gᵀ dt |
| Time-varying motion | New `ControlSource` (scripted profile or slider) |
| Go 3D | New model with a nominal/error-state split; ⊞/⊟ carry over. See `REFERENCE_3D.md` |
| Run under ROS 2 | Wrap `EKF` in an `rclpy` node: callbacks build `ProcessInput` / `Measurement`. See `ROS2_INTEGRATION.md` |

## Known limitations
- Q is hand-tuned and much too conservative, as NEES shows.
- v is never measured, so position σ grows without bound.
- Index constants (0, 1, 2, 4) are hard-coded in the viewer, plots and `heading_measurement`. A `UnicycleState(IntEnum)` would centralize them.
- `Measurement` is `frozen=True` but not `eq=False`, so comparing two measurements with `==` would raise.
- There's no handling for delayed or out-of-order measurements: each update is applied at the current filter time.
