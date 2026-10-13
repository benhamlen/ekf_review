# Planar → 3D: what changes and why

A companion to `REFERENCE.md` (same folder). The planar filter in this repo has state `[x, y, θ, v, b_g]` and uses the gyro z-axis as its input. A real rover estimator is 3D. This sheet maps every planar concept to its 3D version.

**One-line summary:** in 3D the attitude lives on SO(3), not on a line. You keep a *nominal* state with a quaternion and run the EKF on a small 3-vector *error*. That's the **error-state EKF (ES-EKF)**, and ⊞/⊟ is the mechanism that makes it work.

Canonical reference: J. Solà, *Quaternion kinematics for the error-state Kalman filter* (2017). Lie-group background: Solà, Deray & Atchuthan, *A micro Lie theory for state estimation in robotics* (2018).

---

## 1. Planar ↔ 3D at a glance

| Concept | Planar (this repo) | 3D |
|---|---|---|
| Attitude | θ (scalar, wrap to ±π) | R ∈ SO(3) or unit quaternion q (4 params, 3 DOF) |
| Attitude error | θ̂ − θ, wrapped | δθ ∈ ℝ³, with R = R̂·Exp(δθ) |
| ⊞ | `x + dx`, wrap θ | `q ⊗ Exp(δθ)` for attitude, `+` for vectors |
| ⊟ | `wrap(z − ẑ)` | `Log(R̂ᵀ R_meas)` for attitude, `−` for vectors |
| Gyro | ω_z only | ω ∈ ℝ³ with bias b_g ∈ ℝ³ |
| Accelerometer | unused | specific force a_m ∈ ℝ³ with bias b_a ∈ ℝ³; gravity gives roll and pitch |
| Velocity | scalar v along heading | v ∈ ℝ³ in the world frame (or body frame) |
| State size | 5 | nominal 16 (p3, v3, q4, b_g3, b_a3); **error state 15** |
| P | 5×5 | **15×15**, over the *error* state |
| Heading measurement | `h = θ`, H = e₃ᵀ | star tracker: full attitude, H = [0 0 I 0 0] on δθ |
| Unobservable without help | position | position **and yaw** (gravity fixes roll and pitch only) |

---

## 2. Why an error-state filter

- A quaternion has 4 numbers for 3 DOF, plus the constraint |q| = 1. A plain EKF on q produces a **singular, constraint-violating covariance**, and additive updates `q + Kq` leave the unit sphere.
- The error δθ is **minimal (3D), unconstrained and near zero**, so the linearization is good and P is well-defined.
- The loop has three phases:
  1. **Propagate the nominal state** with the full nonlinear kinematics (no noise).
  2. **Propagate the error covariance** with the error-state Jacobian F_δ and Q_δ.
  3. **On update:** compute δx̂ = K y, **inject** it into the nominal state (p += δp, v += δv, q ← q ⊗ Exp(δθ), b += δb), then **reset** δx̂ = 0. The covariance reset Jacobian is ≈ I and is usually ignored.
- This is the same ⊞/⊟ pattern as the repo's `boxplus` and `residual`, just on SO(3).

---

## 3. Rotation toolkit

```python
import numpy as np
from scipy.spatial.transform import Rotation as Rot

def skew(w):                       # [w]×, so that skew(a) @ b == np.cross(a, b)
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

Exp = lambda phi: Rot.from_rotvec(phi).as_matrix()      # ℝ³ → SO(3) (Rodrigues)
Log = lambda R:   Rot.from_matrix(R).as_rotvec()        # SO(3) → ℝ³
```
- First-order approximations: `Exp(δθ) ≈ I + [δθ]×`, and `[a]× b = a × b = −[b]× a`.
- **Pick one perturbation convention and use it everywhere.**
  - *Local (right):* R = R̂·Exp(δθ), with δθ in the **body** frame. Solà's ES-EKF uses this, and so does this sheet.
  - *Global (left):* R = Exp(δθ)·R̂, with δθ in the **world** frame.
  - The Jacobians differ. Mixing the two conventions is a classic bug.
- **Quaternion ordering traps:**
  - ROS `geometry_msgs/Quaternion` fields are **x, y, z, w**.
  - SciPy `Rotation.from_quat` defaults to **scalar-last [x, y, z, w]**; pass `scalar_first=True` for w first.
  - Eigen's `Quaterniond(w, x, y, z)` constructor takes **w first**, but `.coeffs()` returns x, y, z, w.
  - Hamilton and JPL quaternion conventions also differ. ROS, SciPy and Eigen all use Hamilton.
- **Validate with finite differences:** perturb δθ by ε·eᵢ through `R̂ @ Exp(ε eᵢ)`, not by adding to quaternion components.

---

## 4. IMU-driven kinematics (the 3D version of `f`)

The accelerometer measures **specific force** in the body frame: `a_m = Rᵀ(a − g) + b_a + noise`. Take the world frame as ENU with **g = [0, 0, −1.62] m/s²** on the Moon. At rest, the accelerometer therefore reads about +1.62 m/s² "up."

Nominal propagation, with ω = ω_m − b_g and a = a_m − b_a:
```
p ← p + v dt + ½ (R a + g) dt²
v ← v + (R a + g) dt
q ← q ⊗ Exp(ω dt)            (R ← R · Exp(ω dt))
b_g, b_a unchanged (random walk)
```

Error-state Jacobian F_δ, using the local-δθ convention and the order [δp, δv, δθ, δb_g, δb_a]:
```
δp ← δp + δv dt
δv ← δv + (−R [a]× δθ − R δb_a) dt
δθ ← Exp(ω dt)ᵀ δθ − δb_g dt
δb ← δb
```
- **Read the δv row as a talking point:** an attitude error δθ makes you rotate the measured specific force into the wrong direction. At rest, a tilt error therefore leaks gravity into horizontal velocity. This is the 3D version of "heading error turns into position error."
- **Q_δ from IMU specs** (continuous densities, per step of length dt):
  - velocity: σ_a² dt·I
  - angle: σ_g² dt·I
  - accelerometer bias: σ_ba² dt·I
  - gyro bias: σ_bg² dt·I

  It's block-diagonal on the noise inputs. The cross-coupling into other states then builds up through F_δ.
- **Alternative if there's no good IMU model or time is short:** a constant-velocity or "odometry-driven" model, using wheel odometry for body velocity and the gyro for rotation. It's simpler, and the accelerometer is used only as a tilt measurement.

---

## 5. Measurements in 3D (residual, Jacobian, intuition)

H is with respect to the error state [δp, δv, δθ, δb_g, δb_a]. Only the nonzero blocks are listed.

| Sensor | Residual y = z ⊟ ẑ | Nonzero H blocks | Constrains |
|---|---|---|---|
| **Star tracker** (attitude) | `Log(R̂ᵀ R_meas)` | δθ: **I** | all 3 rotations, including yaw |
| **Sun sensor** (unit vector s_B) | `s_meas − R̂ᵀ s_L` | δθ: **[ŝ_B]×** (rank 2) | 2 DOF; rotation about the sun line is free |
| **Accelerometer as tilt** (static) | `a_m − R̂ᵀ(−g)` | δθ: **[ĝ_B]×**, δb_a: I | roll and pitch only (rank 2) |
| **DEM / global pose fix** | `[p_meas − p̂ ; Log(R̂ᵀ R_meas)]` | δp: I, δθ: I | position (and attitude if provided) |
| **Global position only** | `p_meas − p̂` | δp: I | position |
| **Wheel odometry** (body velocity) | `v_meas,B − R̂ᵀ v̂` | δv: R̂ᵀ, δθ: [R̂ᵀ v̂]× | body velocity; ties v to attitude |
| **Non-holonomic constraint** (pseudo) | `0 − (R̂ᵀ v̂)_{y,z}` | rows y, z of the wheel-odometry H | "no sideslip, no vertical velocity": cheap and very effective |
| **Zero-velocity update** (when stopped) | `0 − v̂` | δv: I | velocity; makes the biases observable |

**Derivation pattern** (do this on a whiteboard): substitute R = R̂·Exp(δθ), use Exp(δθ) ≈ I + [δθ]× (and Exp(−δθ) ≈ I − [δθ]× for Rᵀ), keep first-order terms, and use [a]×b = −[b]×a. For example, for the sun sensor: Rᵀs_L ≈ (I − [δθ]×)ŝ_B = ŝ_B + [ŝ_B]× δθ, so H_θ = [ŝ_B]×.

**Star-tracker details:**
- Its accuracy is anisotropic: very good across the boresight and several times worse about the boresight. Build R in the sensor frame and rotate it into the body frame: `R_body = C R_sensor Cᵀ`.
- The measurement is inertial attitude. Chain the frames: `R_local←body = R_local←Moon-fixed(position) · R_Moon-fixed←inertial(t, ephemeris) · R_inertial←sensor · R_sensor←body`. Errors in time or position leak into heading through this chain.

**Sun sensor plus gravity:** two non-parallel vectors give full attitude (TRIAD), or you can fuse them as two rank-2 updates. Near the poles the sun is low on the horizon, so it's nearly perpendicular to gravity, which conditions the problem well.

---

## 6. Observability in 3D (what you can and can't know)

- **IMU only:** everything drifts. Roll and pitch are observable from gravity when accelerations are small. **Yaw is not**, and position and velocity aren't either.
- **Horizontal accelerometer bias and tilt are confounded:** a bias looks like a small tilt. They separate only through attitude changes (maneuvers) or an independent attitude source (star tracker).
- **Gyro bias:** x and y become observable through gravity and tilt; z (the yaw axis) needs a heading reference (star tracker, sun sensor, map).
- **Absolute position** requires an absolute source: DEM registration, landmarks, or ground tracking. Wheel odometry, visual odometry and IMU only slow the drift.
- **Same lesson as the planar case:** cross-covariance lets the star tracker correct position and biases indirectly. NEES/NIS checks carry over unchanged, with dof = 15 for NEES on the error state and dof = m for NIS.

---

## 7. Terrain registration in 3D

Transform point qᵢ (body frame) into the world: `wᵢ = R qᵢ + p`. Height residual against the DEM:
```
rᵢ = wᵢ,z − h(wᵢ,x, wᵢ,y)
n̂ᵢ = [−∂h/∂x, −∂h/∂y, 1]              (unnormalized surface normal at the hit point)
∂rᵢ/∂δp = n̂ᵢᵀ
∂rᵢ/∂δθ = n̂ᵢᵀ (−R [qᵢ]×)                (local perturbation)
```
- This is point-to-plane ICP with the DEM providing the planes.
- **Use the IMU and star tracker to fix roll, pitch (and yaw).** That reduces registration to 3 or 4 DOF (x, y, z, ±yaw), which is much better conditioned and faster. It's a good design point to state out loud.
- **Degeneracy:** take the eigen-decomposition of the 6×6 (or 4×4) `JᵀΣ⁻¹J`. Flat terrain constrains only z, roll and pitch. A long valley or ridge leaves translation along it weak. Inflate R along weak eigenvectors, or project the update away from them.
- **Covariance → EKF:** `R_pose ≈ (JᵀΣ⁻¹J)⁻¹`, expressed in the same δp/δθ convention as your filter. Inflate it for DEM resolution and error.
- **Lever arm:** the LiDAR is offset from the IMU or body origin, so use the full extrinsic `T_body←lidar` from URDF or TF.

---

## 8. ROS 2 in 3D

- **`nav_msgs/Odometry`:**
  - Pose is in `header.frame_id` (e.g. `odom`); twist is in `child_frame_id` (e.g. `base_link`).
  - Covariances are 6×6 row-major over (x, y, z, rotX, rotY, rotZ).
  - **The pose covariance is expressed in the parent frame.** If your δθ is body-local, rotate the attitude block before publishing: `Σ_world = R Σ_body Rᵀ`. The position block is already in the world frame if δp is in the world frame.
- **`sensor_msgs/Imu`:**
  - `angular_velocity` (rad/s) and `linear_acceleration` (m/s², specific force including the reaction to gravity) are in `header.frame_id`, which is the IMU frame.
  - **Rotate into the body frame using the static TF.** Check whether `orientation_covariance[0] == -1`, which means "no orientation provided."
- **`tf2`:** `lookup_transform("base_link", "imu_link", Time())` for extrinsics. `tf_transformations` may not be installed on Jazzy; use `scipy.spatial.transform.Rotation` for conversions instead.
- **`robot_localization` uses a 15-state model:** (x, y, z, roll, pitch, yaw, their rates, ax, ay, az). It has `two_d_mode: true` for planar operation. It's a quick baseline to compare a custom filter against.

---

## 9. Pragmatic simplifications (say them out loud)

- **2.5D:** estimate x, y, yaw in the filter. Take z from the DEM at (x, y). Take roll and pitch directly from the accelerometer (low-pass filtered) or the star tracker. That's reasonable for a slow rover on gentle terrain.
- **Attitude from the star tracker, position from the filter:** when the star tracker is accurate and frequent, treat attitude as known and estimate only translation, velocity and biases. That decouples the problem.
- **What you lose:** the coupling terms, for example tilt error leaking gravity into velocity. Say when that matters: high accelerations, poor attitude sensors, long gaps.
- Before writing any code, decide on 2D, 2.5D or full 3D based on the task's time budget. Write down the assumption so you can present it.

---

## 10. Minimal 3D ES-EKF skeleton (shape check)

```python
# nominal: p(3), v(3), q (scipy Rotation), bg(3), ba(3)      error state: 15, P: 15x15
I_P, I_V, I_TH, I_BG, I_BA = slice(0,3), slice(3,6), slice(6,9), slice(9,12), slice(12,15)

def predict(nom, P, omega_m, acc_m, dt, Qd):
    R = nom.q.as_matrix()
    w, a = omega_m - nom.bg, acc_m - nom.ba
    F = np.eye(15)
    F[I_P, I_V]   = np.eye(3) * dt
    F[I_V, I_TH]  = -R @ skew(a) * dt
    F[I_V, I_BA]  = -R * dt
    F[I_TH, I_TH] = Exp(w * dt).T
    F[I_TH, I_BG] = -np.eye(3) * dt
    # nominal propagation (after computing F with the prior state)
    acc_world = R @ a + G_MOON
    nom.p = nom.p + nom.v * dt + 0.5 * acc_world * dt**2
    nom.v = nom.v + acc_world * dt
    nom.q = nom.q * Rot.from_rotvec(w * dt)
    return nom, F @ P @ F.T + Qd

def inject(nom, dx):                 # x ⊞ δx, then reset δx = 0
    nom.p += dx[I_P]; nom.v += dx[I_V]
    nom.q = nom.q * Rot.from_rotvec(dx[I_TH])
    nom.bg += dx[I_BG]; nom.ba += dx[I_BA]
```
The update step is the same generic EKF update as in the repo: y = z ⊟ h(nominal), H on the error state, Joseph-form P, then `inject`.

**Verified numerically** (finite differences through ⊞/⊟, in this prep session):
- F_δ matches to first order in dt. The largest deviation is the δθ/δb_g block, where the exact value is −J_r(ω dt)·dt rather than −I·dt. That error shrinks as dt², as expected.
- The sun-sensor, wheel-velocity and DEM-point δθ Jacobians match to about 1e-6.
