# Localization Reference Sheet

A quick-lookup sheet for the Starpath onsite: EKF matrices and equations, consistency checks, process noise, angles and rotations, terrain registration, frames, lunar specifics, an `rclpy` cheat sheet, and system-design and presentation checklists.

> Code in this repo: `ekf_review_refined.py` (generic EKF + unicycle model), `sim.py` (world/sensors loop), `evaluation.py` + `plot_eval.py` (errors, NEES), `viz_viser.py` (live viewer).

---

## 1. EKF: every matrix, its shape, its meaning

n = state dimension, m = measurement dimension.

| Symbol | Shape | Name | Meaning |
|---|---|---|---|
| **x** | (n,) | state estimate | Best guess of the state, e.g. `[x, y, θ, v, b_g]` |
| **P** | (n,n) | state covariance | Uncertainty of x. Diagonal = variances; off-diagonal = **cross-correlations** (what lets one sensor correct other states) |
| **u** | – | process input | Data that *drives* the motion model (gyro, wheel speed, commanded twist). Its noise belongs in **Q** |
| **f(x,u,dt)** | (n,) | motion model | Propagates the state one step |
| **F = ∂f/∂x** | (n,n) | state-transition Jacobian | Linearization of f, **evaluated at the pre-prediction x** |
| **G = ∂f/∂w** | (n,p) | noise-input Jacobian | How input/process noise w enters the state |
| **Q** | (n,n) | process noise covariance | *New* uncertainty added per step (unmodeled dynamics + noisy inputs). Symmetric PSD, depends on dt |
| **z** | (m,) | measurement | Sensor reading used in **update** |
| **h(x)** | (m,) | measurement model | What the sensor *should* read given x |
| **H = ∂h/∂x** | (m,n) | measurement Jacobian | Linearization of h at the prior x. Rows are ∂hᵢ/∂x |
| **R** | (m,m) | measurement noise covariance | Sensor noise. For a registration "pseudo-measurement" this is the **pose covariance**, not raw point noise |
| **y = z ⊟ h(x)** | (m,) | innovation (residual) | Surprise: measured minus predicted. Wrap the angle components |
| **S = HPHᵀ + R** | (m,m) | innovation covariance | Expected spread of y |
| **K = PHᵀS⁻¹** | (n,m) | Kalman gain | How much each state moves per unit of innovation. `K = solve(S, H @ P).T` (P and S are symmetric) |

### Predict
```
F = F(x⁻, u, dt);  Q = Q(x⁻, u, dt)   # BEFORE overwriting x
x = f(x, u, dt)                        # wrap angles
P = F P Fᵀ + Q
```
### Update
```
y = z ⊟ h(x);   H = H(x)               # both at the prior x
S = H P Hᵀ + R
K = P Hᵀ S⁻¹                            # via solve, never inv
x = x ⊞ K y                             # wrap angles
P = (I − K H) P (I − K H)ᵀ + K R Kᵀ     # Joseph form: stays symmetric PSD
```

### Key concepts
- **Process input vs measurement is about the sensor's role, not what it is.** The gyro is an *input* when it drives predict, so its noise goes in Q. It becomes a *measurement* if ω is a state (`h(x) = ω + b_g`).
- **Cross-covariance does the fusion.** A heading-only update (H = `[0 0 1 0 0]`) still corrects x, y and b_g through the off-diagonal entries of P built up during predict.
- **Observability.** Gyro bias becomes observable from repeated absolute heading fixes. Position is **not** observable from heading plus gyro alone; its error stays small only if v is good, and P_xy grows without bound.
- **Loose vs tight coupling.** Loose: a front end (scan matching, DEM registration, VO) produces z, R and h(x), and the EKF fuses that. Tight: raw residuals (individual points or features) go into the filter. Tight coupling is more accurate and handles degeneracy naturally; loose coupling is simpler and modular.
- **Relative vs absolute sensors.** Relative sensors (gyro, wheel, scan-to-scan) carry the estimate between absolute fixes (star tracker, DEM registration, landmarks). Relative measurements between two poses need state augmentation or cloning, or conversion to a velocity measurement.
- **Error grows differently with bias and noise.** An uncorrected bias makes heading error grow ∝ t (and position error ∝ t²). White noise makes heading error grow ∝ √t.
- **Unicycle model.** `ẋ = v cosθ`, `ẏ = v sinθ`, `θ̇ = ω`, with no sideslip. Skid-steer rovers in regolith violate the no-sideslip assumption, which motivates slip modeling and gating.

### F for this repo's model (state `[x, y, θ, v, b_g]`, input ω_m)
```
f = [x + v cosθ dt,  y + v sinθ dt,  θ + (ω_m − b_g) dt,  v,  b_g]
F = I + [[0,0,−v sinθ dt, cosθ dt, 0],
         [0,0, v cosθ dt, sinθ dt, 0],
         [0,0, 0,         0,     −dt],
         [0,0, 0,0,0], [0,0,0,0,0]]
```
**Check Jacobians numerically:** column i ≈ `(f(x + ε eᵢ) − f(x − ε eᵢ)) / 2ε`.

---

## 2. Consistency (is P honest?)

| Test | Formula | Distribution | Needs truth? |
|---|---|---|---|
| **NEES** | εₖ = eₖᵀ Pₖ⁻¹ eₖ, with e = x̂ − x | χ²(n) | yes (simulation) |
| **NIS** | νₖᵀ Sₖ⁻¹ νₖ | χ²(m) | **no**, so it works on the real robot |

- Mean NEES ≈ n means consistent. **Mean NEES ≫ n means overconfident** (Q or R too small, or an unmodeled error). **≪ n means conservative** (Q or P0 too large).
- For M Monte-Carlo runs: M·ε̄ ~ χ²(M·n), which gives tighter bounds.

**χ² values:**

| dof | 95% two-sided (single run) | 95% gate | 99% gate |
|---|---|---|---|
| 1 | [0.001, 5.02] | 3.84 | 6.63 |
| 2 | [0.051, 7.38] | 5.99 | 9.21 |
| 3 | [0.216, 9.35] | 7.81 | 11.34 |
| 5 | [0.831, 12.83] | 11.07 | 15.09 |
| 6 | [1.24, 14.45] | 12.59 | 16.81 |

- **Innovation gating (outlier rejection):** reject the update if NIS > the 99% gate. Use this for bad registrations, wheel slip and LiDAR multipath. Log every rejection.
- Also check: innovations should be zero-mean and white (no autocorrelation), and errors should stay inside ±3σ about 99% of the time.

---

## 3. Process noise Q from sensor specs

- **Gyro angle random walk (ARW)**, i.e. white rate noise with density σ_g in rad/s/√Hz. The per-sample std at sampling interval dt is σ_g/√dt. Heading variance added per step: **σ_g² · dt**.
- **Gyro bias instability or random walk** σ_bw (rad/s/√s). Bias variance added per step: **σ_bw² · dt**.
- **Velocity random walk** (if v is a state driven by unmodeled acceleration σ_a): **σ_a² · dt**.
- **General form:** `Q_d ≈ G Q_c Gᵀ dt`, where Q_c is the continuous noise density and G maps noise into the state. G depends on x (for example θ), so evaluate it at the same prior x as F.
- **Datasheet conversions:** ARW in °/√hr → rad/s/√Hz: multiply by (π/180)/60. Bias instability in °/hr → rad/s: multiply by (π/180)/3600.
- **What this repo's sim showed:** a hand-tuned constant Q left NEES far from n. Derive Q from specs first, then tune it with NEES and NIS.

---

## 4. Angles and rotations

- **Wrap differences, not necessarily states:** `wrap(a) = (a + π) mod 2π − π`. Wrap innovations and plotted errors.
- **⊞ / ⊟ (manifold) form:** the model defines `x ⊞ δ` (apply a correction) and the measurement defines `z ⊟ ẑ` (residual). In 2D that's addition with a wrap. **In 3D:** `R ⊞ δ = R · Exp(δ)` and `R₁ ⊟ R₂ = Log(R₂ᵀ R₁)`. This is the basis of error-state (multiplicative) EKFs: estimate a 3-vector attitude error, not quaternion components.
- **Quaternion ↔ yaw (planar):** `q = (w, x, y, z) = (cos(ψ/2), 0, 0, sin(ψ/2))`; `ψ = atan2(2(wz + xy), 1 − 2(y² + z²))`.
- **One direction vector constrains 2 rotational DOF** (rotation about the vector itself is unobservable). Two non-parallel vectors give full attitude via **TRIAD** or Wahba's problem (SVD or q-method).

---

## 5. Terrain registration (LiDAR → orbital DEM)

**Height-map residual** for point qᵢ, with known orientation and unknown translation p = [x, y, z]:
```
rᵢ(p) = (q_z,i + z) − h(q_x,i + x, q_y,i + y)
Jᵢ   = [ −∂h/∂x, −∂h/∂y, 1 ]
```
- **Solve:** Gauss-Newton or LM, `scipy.optimize.least_squares(fun, p0, jac=..., loss="huber", f_scale=...)`. Use a robust loss because of map error, rocks smaller than the DEM resolution, and dynamic dust.
- **Pose covariance:** `R_pose ≈ (Jᵀ Σ⁻¹ J)⁻¹ = (Fisher information)⁻¹`. Valid only if J has full rank locally and the residual noise model is right. Real error is larger because of DEM error, interpolation and ambiguity, so **inflate R**.
- **Degeneracy detection:** take the eigen-decomposition of `JᵀJ`. Small eigenvalues mark unconstrained directions.
  - Flat plane: J = [0, 0, 1], so only z is constrained (with rotation states, also roll and pitch). x, y and yaw are unobservable.
  - Ridge: constrained perpendicular to the ridge, weak along it.
  - Responses: inflate R along weak eigenvectors, project the update onto well-constrained directions ("solution remapping"), or skip the update.
- **A low residual does not mean a well-constrained pose.** Check the information matrix, not the cost.
- **Local minima and ambiguity:** repetitive terrain (crater fields) gives multiple basins. Use a coarse-to-fine search, a correlation or grid search over (x, y, ψ) seeded by the EKF prior, multi-hypothesis tracking or a particle filter for global initialization, and NIS gating on the result.
- **Data considerations:** LiDAR is reliable to about 35 m and degraded beyond. Weight by range and incidence angle. The DEM resolution (meters per pixel) limits achievable accuracy. Polar lighting doesn't affect LiDAR. Subsample points, and build a local DEM from the scan (gridded max/mean height) to match against the orbital DEM.
- **Alternatives:** point-to-plane ICP against a DEM mesh, NDT, image-based (orthophoto) matching, crater or landmark matching.

---

## 6. Frames (ROS REP-103 / REP-105)

```
map ──▶ odom ──▶ base_link ──▶ sensor frames (lidar_link, imu_link, ...)
 ▲ global localizer        ▲ local odometry (continuous, drifts)
   publishes map→odom        publishes odom→base_link
```
- **Each transform has exactly one publisher.** The global node computes `map→base_link`, then publishes `map→odom = map→base_link · (odom→base_link)⁻¹`.
- **`odom` frame:** continuous, no jumps, drifts. Use it for control and obstacle avoidance. **`map` frame:** globally accurate, may jump on corrections.
- **REP-103 conventions:** x forward, y left, z up (body); ENU for world frames. SI units, right-handed.
- **Static sensor extrinsics** go in URDF or `static_transform_publisher`.

---

## 7. Lunar specifics (sanity-check numbers)

- **No GNSS, no usable magnetic field** (no compass). Absolute position comes from orbital maps (DEM or imagery), landmarks, or ground-based radiometric tracking.
- **Moon rotation is about 2.66×10⁻⁶ rad/s** (sidereal period 27.3 days). That's far too slow for MEMS gyrocompassing, so heading must come from celestial sensors or maps.
- **Gravity is 1.62 m/s².** The accelerometer at rest gives the gravity direction, hence roll and pitch.
- **Star tracker** gives inertial attitude (body w.r.t. an inertial frame such as J2000/ICRF). Converting to local heading needs the Moon's orientation at that time (ephemeris, e.g. SPICE) plus the rover's approximate position for the local tangent frame. Small position errors cause small heading errors here, but the chain should be modeled.
- **Sun sensor** gives the sun direction in the body frame. Combined with ephemeris, time and position, it constrains 2 DOF. Pair it with the gravity vector (TRIAD) for full attitude. Near the poles the sun sits low on the horizon, so the sun vector and gravity are close to perpendicular, which makes TRIAD well-conditioned. It fails when the sun is near zenith (rare at the poles) or occluded.
- **Lunar body-fixed frames:** Mean Earth/Polar Axis (ME), used for cartography and DEMs, and Principal Axis (PA), used for dynamics. They differ by under about 1 km at the surface, so know which one your DEM uses.
- **Terrain:** regolith causes wheel slip and sinkage. Long polar shadows hurt cameras; LiDAR is lighting-independent. Crater fields can be repetitive, which makes registration ambiguous.
- **Operations:** Earth round-trip light time is about 2.6 s (about 1.3 s each way), plus ground processing, so localization must run onboard. Radiation-tolerant compute is slow, so watch CPU budgets.

---

## 8. ROS 2 Jazzy, `rclpy` cheat sheet

### Package
```bash
ros2 pkg create --build-type ament_python --dependencies rclpy sensor_msgs nav_msgs geometry_msgs tf2_ros -- my_loc
# add to setup.py entry_points: 'console_scripts': ['ekf_node = my_loc.ekf_node:main']
colcon build --symlink-install --packages-select my_loc && source install/setup.bash
ros2 run my_loc ekf_node --ros-args -p use_sim_time:=true
```

### Node skeleton
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class EkfNode(Node):
    def __init__(self):
        super().__init__("ekf_node")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.imu_sub = self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "odometry/filtered", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_stamp = None   # use message stamps for dt, not wall clock

    def on_imu(self, msg: Imu):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last_stamp is not None:
            dt = t - self.last_stamp
            # ekf.predict(ProcessInput(t, msg.angular_velocity.z), dt)
        self.last_stamp = t
        self.publish(msg.header.stamp)

    def publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame       # pose is in this frame
        odom.child_frame_id = self.base_frame        # twist is in this frame
        # odom.pose.pose.position.x = ...; orientation from yaw (see §4)
        # odom.pose.covariance: 36 floats, row-major 6x6 over (x, y, z, rotX, rotY, rotZ)
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        # tf.transform.translation.x = ...; tf.transform.rotation = ...
        self.tf_broadcaster.sendTransform(tf)

def main():
    rclpy.init()
    rclpy.spin(EkfNode())
    rclpy.shutdown()
```
- **Timers:** `self.create_timer(0.1, self.on_timer)`.
- **TF lookup:** `tf2_ros.Buffer` + `TransformListener(buffer, self)`, then `buffer.lookup_transform(target, source, rclpy.time.Time())` for the latest transform. Wrap it in try/except for `TransformException`.
- **Covariance index for yaw** in a 6×6 row-major array: `[35]`. For x: `[0]`, for y: `[7]`, for the xy cross term: `[1]` and `[6]`.
- **QoS:** sensor topics usually use best effort (`qos_profile_sensor_data`). A best-effort publisher with a reliable subscriber means **no messages arrive**. Check with `ros2 topic info -v`.
- **Sim time:** set `use_sim_time:=true` on every node, and make sure `/clock` is published.

### Launch file (`launch/loc.launch.py`)
```python
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package="my_loc", executable="ekf_node", parameters=["config/ekf.yaml", {"use_sim_time": True}]),
    ])
```
Install launch and config files via `data_files` in `setup.py`.

### CLI
```bash
ros2 topic list | echo /imu --once | hz /imu | info -v /imu
ros2 node list | info /ekf_node
ros2 param list /ekf_node | get /ekf_node odom_frame
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
ros2 bag record -a   |   ros2 bag play <bag> --clock
rqt_graph   rviz2
```
- **Off-the-shelf option:** `robot_localization` (`ekf_node`). One instance for `odom` (IMU + wheel) and one for `map` (adds global fixes), configured in YAML. The tradeoff to state: it's fast to stand up, but it's harder to add custom models like DEM-registration degeneracy handling.

---

## 9. System design checklist (localization for a lunar rover)

1. **Requirements:** accuracy needed (driving vs science vs docking), update rates, latency, compute and power budgets, autonomy level.
2. **Sensors and their roles:** IMU (propagation), wheel odometry (relative, slip-prone), LiDAR (scan matching for relative motion, DEM registration for absolute position), star tracker (absolute inertial attitude, giving heading), sun sensor plus gravity (attitude backup), camera (visual odometry later). For each: rate, noise, and failure modes.
3. **Estimator architecture:** a local high-rate EKF (`odom→base_link`) plus a low-rate global correction (`map→odom`). Error-state EKF on SO(3). Or a factor graph / sliding-window smoother when relative-pose constraints and delayed measurements dominate.
4. **Frames and time:** frame tree, time sync, timestamping at the sensor, and out-of-order or delayed measurements (buffer and replay, or augment the state).
5. **Bootstrapping:** initial position from the landing solution plus a global DEM search. Initial heading from the star tracker.
6. **Fault detection and degraded modes:** NIS gating, registration degeneracy checks, slip detection (wheel vs IMU/VO disagreement), sensor dropout timeouts, and a covariance-growth alarm that triggers "stop and re-localize."
7. **Validation:** simulation with truth (NEES), Monte-Carlo runs, field analog tests, bag replay, and regression tests in CI.
8. **Operations:** onboard autonomy because of Earth latency. Telemetry of covariance and innovations to the ground. Ground-in-the-loop re-localization as a fallback.

---

## 10. Presentation structure (end of day, whole group)

1. **Problem and assumptions:** what you were asked, and what you assumed (say which is which).
2. **Approach:** the architecture diagram, and why this design. Name the alternatives you rejected.
3. **What you built:** a demo or plots, kept short.
4. **Results:** quantitative error, ±3σ, NEES/NIS, compared with a baseline such as dead reckoning.
5. **What worked and what didn't:** honest failures and their root causes.
6. **Next steps:** what you'd do with another day, week and month.
7. Keep 1–2 backup slides or plots ready for likely questions (degeneracy, Q tuning, frames).
