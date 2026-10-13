"""EKF core, copied from ekf_review/ekf_review_refined.py (+ wheel_speed_measurement). No ROS imports."""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


def subtract_residual(z: Vec, z_hat: Vec) -> Vec:
    """Default z ⊟ ẑ for ordinary (non-angular) measurements."""
    return z - z_hat


@dataclass(frozen=True)
class Control:
    """What the rover is *actually* doing (truth input). Estimator never sees this."""

    v: float  # m/s
    omega: float  # rad/s


@dataclass(frozen=True)
class TruthState:
    t: float
    x: float
    y: float
    theta: float  # keep unwrapped; wrap only errors
    v: float
    omega: float  # sensors need the true rate to synthesize gyro readings


@dataclass(frozen=True)
class ProcessInput:
    """Sensor reading that drives predict() via the motion model; its noise lives in Q, not R."""

    t: float
    gyro_z: float  # measured yaw rate (rad/s), includes bias + noise


@dataclass(frozen=True)
class Measurement:
    """Everything update() needs; one per sensor reading."""

    t: float
    z: Vec  # shape (m,)
    R: Vec  # shape (m, m)
    h: Callable[[Vec], Vec]  # x -> predicted z, shape (m,)
    H: Callable[[Vec], Vec]  # x -> Jacobian, shape (m, n)
    residual: Callable[[Vec, Vec], Vec] = subtract_residual  # z ⊟ ẑ
    source: str = ""  # "star_tracker", "wheel", ... for logging/plots


@dataclass(frozen=True, eq=False)
class EstimateSnapshot:
    """Filter output at one instant; what History stores and the visualizer reads."""

    t: float
    x: Vec  # shape (n,); pass x.copy() so later filter steps can't change it
    P: Vec  # shape (n, n); pass P.copy()


class MotionModel(Protocol):
    def f(self, x: Vec, u: ProcessInput, dt: float) -> Vec: ...
    def F(self, x: Vec, u: ProcessInput, dt: float) -> Vec: ...
    def Q(self, x: Vec, u: ProcessInput, dt: float) -> Vec: ...
    def boxplus(
        self, x: Vec, dx: Vec
    ) -> Vec: ...  # x ⊞ dx: apply a correction, keep x valid


class Sensor(Protocol):
    def sample(self, truth: TruthState) -> Measurement | None: ...


class ControlSource(Protocol):
    def command(self, t: float) -> Control: ...


class UnicycleBiasModel:
    """State [x, y, theta, v, b_g]; gyro drives heading."""

    def __init__(self, Q: Vec) -> None:
        # HINT: store as self._Q (not self.Q -- that name is taken by the method below)
        # HINT: constant Q for now so the refactor doesn't change behavior; Q(dt) comes later
        self._Q = Q

    def boxplus(
        self, x: Vec, dx: Vec
    ) -> Vec:  # x ⊞ dx: apply a correction, keep x valid
        x_new = x + dx # new array, so modifying it below doesn't touch x
        x_new[2] = normalize_angle(x_new[2])
        return x_new


    def f(self, x: Vec, u: ProcessInput, dt: float) -> Vec:
        # HINT: port get_f from ekf_review.py
        # HINT: unpack x first (px, py, theta, v, bg = x) -- avoid naming one of them `x`, it shadows the state
        # HINT: the yaw rate comes from u.gyro_z (measured), never from truth
        # HINT: return a new array; don't modify x in place
        px, py, theta, v, bg = x

        return np.array(
            [
                px + v * np.cos(theta) * dt,
                py + v * np.sin(theta) * dt,
                normalize_angle(theta + (u.gyro_z - bg) * dt),
                v,
                bg,
            ]
        )

    def F(self, x: Vec, u: ProcessInput, dt: float) -> Vec:
        # HINT: port get_F; evaluate at the x you're given (the EKF must pass the *pre-prediction* state)
        # HINT: sanity check later: compare against a finite-difference Jacobian of f
        #       (perturb each state by eps, (f(x+eps*e_i) - f(x)) / eps -> column i)

        px, py, theta, v, bg = x

        return np.array(
            [
                [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0],
                [0, 1, v * np.cos(theta) * dt, np.sin(theta) * dt, 0],
                [0, 0, 1, 0, -dt],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )

    def Q(self, x: Vec, u: ProcessInput, dt: float) -> Vec:
        # HINT: just return self._Q for now; x, u, dt are unused until we derive Q from sensor specs
        return self._Q


class EKF:
    """Generic EKF: knows nothing about rovers. Model supplies f/F/Q, measurements supply z/h/H/R."""

    def __init__(
        self,
        model: MotionModel,
        x0: Vec,
        P0: Vec,
    ) -> None:
        self.model = model
        self.x = np.asarray(x0, dtype=float).copy()
        self.P = np.asarray(a=P0, dtype=float).copy()

    def predict(self, u: ProcessInput, dt: float) -> None:

        # save before updating state
        F = self.model.F(self.x, u, dt)
        Q = self.model.Q(self.x, u, dt)

        # propagate state (and uncertainty) from dynamics and known control inputs
        self.x = self.model.f(self.x, u, dt)
        self.P = F @ self.P @ F.T + Q

    def update(self, meas: Measurement) -> None:

        # innovation: actual measurement minus predicted measurement h(x)
        y = meas.residual(meas.z, meas.h(self.x))

        # measurement Jacobian at the prior state
        H: Vec = meas.H(self.x)

        # innovation covariance: state uncertainty mapped through H, plus sensor noise
        S_innov: Vec = H @ self.P @ H.T + meas.R

        # Kalman gain K = P H^T S^-1, via solve() instead of an explicit inverse
        K: Vec = np.linalg.solve(S_innov, H @ self.P).T

        # state update
        self.x = self.model.boxplus(self.x, K @ y)

        # covariance update, Joseph form (stays symmetric PSD under rounding)
        I_KH: Vec = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ meas.R @ K.T

    def snapshot(self, t: float) -> EstimateSnapshot:
        # HINT: EstimateSnapshot(t, self.x.copy(), self.P.copy()) -- copies, or history entries change later
        return EstimateSnapshot(t, self.x.copy(), self.P.copy())


def normalize_angle[T: (float, Vec)](theta: T) -> T:
    return (theta + np.pi) % (2 * np.pi) - np.pi


def heading_measurement(t: float, theta_meas: float, sigma: float) -> Measurement:
    return Measurement(
        t=t,
        z=np.array([theta_meas]),  # what the sensor read
        R=np.array([[sigma**2]]),
        h=lambda x: np.array([x[2]]),  # predicted heading = theta, as shape (1,)
        H=lambda x: np.array([[0.0, 0.0, 1.0, 0.0, 0.0]]),  # d h / d x (constant here)
        residual=lambda z, z_hat: normalize_angle(z - z_hat),
        source="star_tracker",
    )

# --- added for the ROS template (see docs/ADDING_SENSORS.md) ---
def wheel_speed_measurement(t: float, v_meas: float, sigma: float) -> Measurement:
    return Measurement(
        t=t,
        z=np.array([v_meas]),
        R=np.array([[sigma**2]]),
        h=lambda x: np.array([x[3]]),
        H=lambda x: np.array([[0.0, 0.0, 0.0, 1.0, 0.0]]),
        source="wheel_speed",
    )


if __name__ == "__main__":

    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    P0 = np.eye(x0.shape[0]) # HELP Cannot subscript object of type `int` with no `__getitem__` method
    Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.00001])

    gyro_rate = 0.1 # rad/s
    dt = 0.1 # s

    measurement = heading_measurement(t=dt, theta_meas=0.2, sigma=0.1)   # sigma=0.1 -> R = 0.01, same as the original test
    uni_bias_model = UnicycleBiasModel(Q)
    ekf = EKF(uni_bias_model, x0, P0)

    ekf.predict(ProcessInput(t=dt, gyro_z=gyro_rate), dt)
    ekf.update(measurement)

    np.testing.assert_allclose(ekf.x, [0.1, 0.01862562, 0.19813744, 1.0, -0.01862562], atol=1e-7)
