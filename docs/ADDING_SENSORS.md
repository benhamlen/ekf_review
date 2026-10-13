# Adding a new sensor

A recipe for adding any sensor to the EKF, with worked examples that run against `ekf_review_refined.py`. Every example below has been checked: Jacobians against central finite differences (error < 1e-9) and update steps against the EKF.

---

## Step 0: decide the sensor's role

| If the sensor… | Role | Goes into | Noise goes into |
|---|---|---|---|
| is high-rate and drives the motion (gyro, accelerometer, sometimes wheel odometry) | **process input** | `MotionModel.f` via `ProcessInput` | **Q** |
| observes the state, absolutely or relatively (star tracker, position fix, DEM registration, wheel speed as a check) | **measurement** | `EKF.update` via `Measurement` | **R** |

The same physical sensor can be either. Wheel odometry is an *input* in a "drive with wheels" model, or a *measurement* `h(x) = v` in this repo's constant-velocity model. Pick one and state the reason.

## Step 1: write the measurement model

Answer four questions:
1. **z:** what does the sensor output, with shape (m,)?
2. **h(x):** what would it read if the state were exactly x?
3. **H(x) = ∂h/∂x:** an (m, n) matrix. Derive it, then **check it numerically**.
4. **R:** the (m, m) noise covariance. Where does it come from: a datasheet, calibration, or a front-end's covariance? Is it range-dependent? Are there correlations?

Also: does **⊟** need anything special (angles, unit vectors, rotations)? If so, pass a `residual`.

## Step 2: write a builder

```python
def <name>_measurement(t, z_meas, ..., sigma) -> Measurement:
    return Measurement(t=t, z=..., R=..., h=..., H=..., residual=..., source="<name>")
```

## Step 3: write a sensor class (for simulation)

```python
class FooSensor:
    def __init__(self, sigma, period_steps, rng): ...
    def sample(self, truth: TruthState) -> Measurement | None:
        # return None when it's not this sensor's turn to fire
        # z = h(true state) + noise   (use TRUE quantities only)
```

## Step 4: add it to the loop

`meas = foo.sample(truth); if meas is not None: ekf.update(meas)`. Order doesn't matter for measurements at the same time step. Sequential updates are equivalent to one stacked update when the noises are independent.

## Step 5: check it

1. **Finite-difference H:** column i ≈ `(h(x + εeᵢ) − h(x − εeᵢ)) / 2ε`.
2. **Single-update sanity check:** with a large R, the state should barely move; with a tiny R, h(x⁺) should ≈ z.
3. **NIS** = yᵀS⁻¹y averages about m. Gate outliers at the χ²(m) 99% value (see `REFERENCE.md`).
4. **Evaluation:** the error should drop in the states the sensor observes, and NEES should stay consistent.

```python
def fd_jacobian(h, x, eps=1e-6):
    return np.column_stack([(h(x + eps*e) - h(x - eps*e)) / (2*eps) for e in np.eye(len(x))])
assert np.allclose(fd_jacobian(meas.h, x), meas.H(x), atol=1e-6)
```

---

## Worked examples (state `[x, y, θ, v, b_g]`)

### Wheel speed: linear, scalar
```python
def wheel_speed_measurement(t, v_meas, sigma):
    return Measurement(
        t=t, z=np.array([v_meas]), R=np.array([[sigma**2]]),
        h=lambda x: np.array([x[3]]),
        H=lambda x: np.array([[0.0, 0.0, 0.0, 1.0, 0.0]]),
        source="wheel_speed",
    )
```
**Slip:** skid-steer wheels report more speed than the rover actually has. Model it as a bias or scale-factor state, inflate R when commanded turn rates are high, or **NIS-gate** it. Slip shows up as a persistent positive innovation.

### Global position fix (x, y): linear, 2D, full covariance
```python
def position_fix_measurement(t, xy_meas, cov_xy):
    H_xy = np.zeros((2, 5)); H_xy[0, 0] = H_xy[1, 1] = 1.0
    return Measurement(
        t=t, z=np.asarray(xy_meas, float), R=np.asarray(cov_xy, float),   # cov can be non-diagonal
        h=lambda x: x[0:2].copy(), H=lambda x: H_xy, source="position_fix",
    )
```
This is where **DEM registration** plugs in with `cov_xy = (JᵀΣ⁻¹J)⁻¹`, inflated, and possibly elongated along a ridge. It's what bounds position error.

### Range to a known landmark: nonlinear
```python
def landmark_range_measurement(t, range_meas, landmark_xy, sigma):
    lm = np.asarray(landmark_xy, float)
    def h(x): return np.array([np.hypot(x[0] - lm[0], x[1] - lm[1])])
    def H(x):
        d = x[0:2] - lm
        J = np.zeros((1, 5)); J[0, 0:2] = d / np.linalg.norm(d)    # unit vector landmark→rover
        return J
    return Measurement(t=t, z=np.array([range_meas]), R=np.array([[sigma**2]]), h=h, H=H, source="landmark_range")
```
A single range constrains only the direction toward the landmark; the perpendicular direction is unobservable from one measurement. The Jacobian is undefined at the landmark itself, where r = 0.

### Bearing to a known landmark: nonlinear and angular (needs ⊟)
```python
def landmark_bearing_measurement(t, bearing_meas, landmark_xy, sigma):
    lm = np.asarray(landmark_xy, float)
    def h(x): return np.array([normalize_angle(np.arctan2(lm[1] - x[1], lm[0] - x[0]) - x[2])])
    def H(x):
        dx, dy = lm[0] - x[0], lm[1] - x[1]; q = dx*dx + dy*dy
        J = np.zeros((1, 5)); J[0, 0] = dy / q; J[0, 1] = -dx / q; J[0, 2] = -1.0
        return J
    return Measurement(t=t, z=np.array([bearing_meas]), R=np.array([[sigma**2]]), h=h, H=H,
                       residual=lambda z, zh: normalize_angle(z - zh), source="landmark_bearing")
```
Bearing couples position and heading. Camera features of known terrain landmarks look like this.

### Sun sensor (planar): absolute heading from an azimuth
The sensor measures the sun azimuth in the body frame: `h(x) = wrap(α_sun − θ)`, where α_sun is the sun's azimuth in the local frame, taken from ephemeris, time and approximate position. So `H = [0, 0, −1, 0, 0]`, and the residual wraps. In 3D it's a unit vector with rank-2 H; see `REFERENCE_3D.md`.

### DEM registration as a pose fix (x, y, θ)
`z = [x, y, θ]` from registration. `h(x) = x[0:3]`, `H = [I₃ | 0]`, the residual wraps component 2, and R is the registration covariance. **Degeneracy handling:** eigen-decompose R⁻¹ (the information matrix). For eigenvalues below a threshold, either set that direction's variance very large (an effectively uninformative update along it) or project y and H onto the well-constrained subspace before updating.

---

## Harder cases

| Case | Problem | Approach |
|---|---|---|
| **Relative measurement** (scan-to-scan ICP, visual odometry: Δpose between t₁ and t₂) | h depends on two states, but the filter holds only the current one | (a) convert to a velocity/yaw-rate measurement over the interval; (b) **stochastic cloning**: copy the pose into the state at t₁ and update with h = pose(t₂) ⊟ clone; (c) use it as a process input instead |
| **Delayed measurement** (registration finishes 2 s late) | the update belongs to a past state | buffer states and inputs, roll back, apply, re-propagate; or keep a cloned past pose; or do nothing if the delay is short compared with the dynamics |
| **Out-of-order arrival** | measurement timestamps aren't monotonic | timestamp-ordered queue with a small latency window |
| **Sensor needs a new state** (scale factor, extrinsic, bias) | h depends on something you don't estimate | augment the state: new `MotionModel` with a larger n and a random-walk Q for the new state; check observability |
| **Correlated noise** (same front end produces two measurements) | treating them as independent makes the filter overconfident | stack them into one `Measurement` with a full R |
| **Non-Gaussian / outliers** | multipath, wrong registration basin | NIS gating, robust (Huber) inflation of R, or multi-hypothesis tracking |
| **Rank-deficient information** (flat terrain) | low residual but unconstrained pose | eigen-check `JᵀJ`; inflate, project, or skip |

---

## Checklist (say these in a design review)
- [ ] Role chosen (input vs measurement) and justified
- [ ] z, h, H, R, ⊟ written down; H checked with finite differences
- [ ] Where R comes from, and whether it varies (range, incidence angle, terrain)
- [ ] Rate, latency, timestamp source, frame (extrinsic transform to base_link)
- [ ] Failure modes and detection (NIS gate, timeout, degeneracy)
- [ ] Observability: which states does this sensor actually improve?
- [ ] Evaluation plan: error and NEES before vs after adding it
