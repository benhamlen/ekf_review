"""Closed-loop-free rover sim: World -> Sensors -> EKF / dead reckoning -> viewer.

Run:   pixi run python sim.py            (Viser viewer at http://localhost:8080)
       pixi run python sim.py --no-viz   (headless; prints regression checks)
"""

import argparse
from dataclasses import dataclass

import numpy as np

from ekf_review_refined import (
    EKF,
    Control,
    EstimateSnapshot,
    Measurement,
    ProcessInput,
    TruthState,
    UnicycleBiasModel,
    Vec,
    heading_measurement,
    normalize_angle,
)

# --- sim configuration (truth + sensor specs) ---
DT = 0.1  # s, predict rate 10 Hz
DURATION_S = 120.0
V_TRUE = 1.0  # m/s
OMEGA_TRUE = 0.1  # rad/s
GYRO_BIAS_TRUE = 0.02  # rad/s
SIGMA_GYRO = 0.01  # rad/s, white noise per 10 Hz sample
SIGMA_HEADING = 0.005  # rad, star-tracker-derived heading
STAR_TRACKER_PERIOD_STEPS = 10  # 1 Hz
TRUTH_SUBSTEPS = 10  # truth integrated at dt/10 so it doesn't share the EKF's integrator
SEED = 42


class ConstantControl:
    """ControlSource: constant speed and turn rate."""

    def __init__(self, v: float, omega: float) -> None:
        self._control = Control(v=v, omega=omega)

    def command(self, t: float) -> Control:
        return self._control


class World:
    """Ground truth. Integrates the unicycle with fine Euler sub-steps from the TRUE control."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, substeps: int = TRUTH_SUBSTEPS) -> None:
        self.t = 0.0
        self.x, self.y, self.theta = x, y, theta
        self.v, self.omega = 0.0, 0.0
        self.substeps = substeps

    def state(self) -> TruthState:
        return TruthState(t=self.t, x=self.x, y=self.y, theta=self.theta, v=self.v, omega=self.omega)

    def set_control(self, control: Control) -> None:
        """True rates for the coming interval (what an onboard gyro would physically sense)."""
        self.v, self.omega = control.v, control.omega

    def step(self, dt: float) -> TruthState:
        h = dt / self.substeps
        for _ in range(self.substeps):
            self.x += self.v * np.cos(self.theta) * h
            self.y += self.v * np.sin(self.theta) * h
            self.theta += self.omega * h  # kept unwrapped; wrap only errors
        self.t += dt
        return self.state()


class GyroSensor:
    """Yaw-rate gyro. Process input (drives predict), not a Measurement."""

    def __init__(self, bias: float, sigma: float, rng: np.random.Generator) -> None:
        self.bias, self.sigma, self.rng = bias, sigma, rng

    def sample(self, truth: TruthState) -> ProcessInput:
        return ProcessInput(t=truth.t, gyro_z=truth.omega + self.bias + self.rng.normal(0.0, self.sigma))


class StarTrackerSensor:
    """Absolute heading (star tracker -> lunar-local frame), fires every `period_steps` calls."""

    def __init__(self, sigma: float, period_steps: int, rng: np.random.Generator) -> None:
        self.sigma, self.period_steps, self.rng = sigma, period_steps, rng
        self._calls = 0

    def sample(self, truth: TruthState) -> Measurement | None:
        self._calls += 1
        if self._calls % self.period_steps != 0:
            return None
        theta_meas = normalize_angle(truth.theta + self.rng.normal(0.0, self.sigma))
        return heading_measurement(t=truth.t, theta_meas=theta_meas, sigma=self.sigma)


def fit_circle_radius(xy: Vec) -> float:
    """Algebraic (Kasa) least-squares circle fit: x² + y² + D x + E y + F = 0."""
    A = np.column_stack([xy[:, 0], xy[:, 1], np.ones(len(xy))])
    b = -(xy[:, 0] ** 2 + xy[:, 1] ** 2)
    D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
    return float(np.sqrt(D**2 / 4 + E**2 / 4 - F))


@dataclass(frozen=True, eq=False)
class SimResult:
    """Everything evaluation needs; one entry per step at t_1 .. t_N."""

    truth: list[TruthState]
    ekf: list[EstimateSnapshot]
    dr: list[EstimateSnapshot]
    ekf_updated: list[bool]  # True where the EKF applied a star-tracker update
    bias_true: float


def run_sim(use_viz: bool = False) -> SimResult:
    rng = np.random.default_rng(SEED)

    control_source = ConstantControl(V_TRUE, OMEGA_TRUE)
    world = World()
    gyro = GyroSensor(GYRO_BIAS_TRUE, SIGMA_GYRO, rng)
    star_tracker = StarTrackerSensor(SIGMA_HEADING, STAR_TRACKER_PERIOD_STEPS, rng)

    # same model, same initial guess for both; v initialized to the true 1 m/s, bias guess 0
    Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.00001])
    model = UnicycleBiasModel(Q)
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    P0 = np.eye(len(x0))
    ekf = EKF(model, x0, P0)
    dr = EKF(model, x0, P0)  # dead reckoning = EKF that never gets a Measurement

    viz = None
    if use_viz:
        from viz_viser import (
            ViserViz,  # imported lazily so --no-viz doesn't need a server
        )

        viz = ViserViz()

    truth = world.state()  # t = 0
    num_steps = round(DURATION_S / DT)
    truth_hist: list[TruthState] = []
    ekf_hist: list[EstimateSnapshot] = []
    dr_hist: list[EstimateSnapshot] = []
    ekf_updated: list[bool] = []

    for _ in range(num_steps):
        world.set_control(control_source.command(truth.t))

        # gyro sampled at the START of the interval drives t_{k-1} -> t_k
        gyro_input = gyro.sample(world.state())
        truth = world.step(DT)  # truth now at t_k

        ekf.predict(gyro_input, DT)
        dr.predict(gyro_input, DT)

        meas = star_tracker.sample(truth)  # measurement at t_k, applied to the estimate at t_k
        if meas is not None:
            ekf.update(meas)

        ekf_snap, dr_snap = ekf.snapshot(truth.t), dr.snapshot(truth.t)
        truth_hist.append(truth)
        ekf_hist.append(ekf_snap)
        dr_hist.append(dr_snap)
        ekf_updated.append(meas is not None)

        if viz is not None:
            viz.log_step(
                truth,
                {"EKF": ekf_snap, "dead reckoning": dr_snap},
                updated=["EKF"] if meas is not None else (),
            )

    if viz is not None:
        viz.hold()

    return SimResult(truth_hist, ekf_hist, dr_hist, ekf_updated, GYRO_BIAS_TRUE)


def print_regression_checks(result: SimResult) -> None:
    """Step 4 checks: refactor didn't change behavior."""
    truth_hist, ekf_hist, dr_hist = result.truth, result.ekf, result.dr
    truth_xy = np.array([[s.x, s.y] for s in truth_hist])
    ekf_xy = np.array([s.x[:2] for s in ekf_hist])
    dr_xy = np.array([s.x[:2] for s in dr_hist])
    dr_radius = fit_circle_radius(dr_xy)
    expected_dr_radius = V_TRUE / (OMEGA_TRUE + GYRO_BIAS_TRUE)
    print(f"truth radius fit:   {fit_circle_radius(truth_xy):.3f} m (expected {V_TRUE / OMEGA_TRUE:.3f})")
    print(f"DR radius fit:      {dr_radius:.3f} m (expected ≈ {expected_dr_radius:.3f})")
    print(f"final pos error:    EKF {np.linalg.norm(ekf_xy[-1] - truth_xy[-1]):.3f} m | "
          f"DR {np.linalg.norm(dr_xy[-1] - truth_xy[-1]):.3f} m")
    print(f"final bias est:     EKF {ekf_hist[-1].x[4]:.4f} rad/s (true {GYRO_BIAS_TRUE})")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viz", action="store_true", help="run headless and print regression checks")
    print_regression_checks(run_sim(use_viz=not parser.parse_args().no_viz))
