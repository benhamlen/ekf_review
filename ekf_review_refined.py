from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


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
    angle_idx: tuple[int, ...] = ()  # which components of z to wrap
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
                theta + (u.gyro_z - bg) * dt,
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
        angle_state_idx: tuple[int, ...] = (2,),
    ) -> None:
        # HINT: store model, angle_state_idx
        # HINT: store x0.copy() and P0.copy() -- avoids the aliasing bug from before
        # HINT: cast to float (np.asarray(x0, dtype=float)) so integer inputs don't silently truncate

        self.model = model
        self.x0 = x0
        self.P0 = P0
        self.angle_state_idx = angle_state_idx

        self.x = x0.copy()
        self.P = P0.copy()

    def predict(self, u: ProcessInput, dt: float) -> None:
        # HINT: order matters -- compute F (and Q) from the CURRENT self.x *before* overwriting it
        # HINT: then self.x = model.f(...), self.P = F P F^T + Q
        # HINT: wrap every index in self.angle_state_idx with normalize_angle
        # HINT: optional: re-symmetrize P = 0.5 * (P + P.T) to kill rounding drift

        # i dont like how much this has
        px, py, theta, v, bg = self.x

        # propagate state from dynamics and known control inputs
        state_updated: NDArray[np.float64] = self.model.f(self.x, u, theta, v, bg, dt, gyro_z)

    def update(self, meas: Measurement) -> None:
        # HINT: innovation y = meas.z - meas.h(self.x); shapes must both be (m,)
        # HINT: wrap y[i] for i in meas.angle_idx (angles in measurement space, not state space)
        # HINT: H = meas.H(self.x) -- evaluate at the PRIOR state, same x you used for h
        # HINT: S = H P H^T + R;  K = solve(S, H P).T   (why .T works: P and S are symmetric)
        # HINT: I = np.eye(len(self.x)) -- size from the state, not a hard-coded 5
        # HINT: x += K y, then wrap self.angle_state_idx; P via Joseph form
        # HINT: optional for later: return/store y and S -- needed for NIS consistency checks
        ...

    def snapshot(self, t: float) -> EstimateSnapshot:
        # HINT: EstimateSnapshot(t, self.x.copy(), self.P.copy()) -- copies, or history entries change later
        ...


def normalize_angle(theta: float) -> float:
    # HINT: copy from ekf_review.py
    ...


def heading_measurement(t: float, theta_meas: float, sigma: float) -> Measurement:
    return Measurement(
        t=t,
        z=np.array([theta_meas]),  # what the sensor read
        R=np.array([[sigma**2]]),
        h=lambda x: np.array(
            [x[2]]
        ),  # IS ONLY ELEMENT 2 (non-includive upper bound) what it *should* read given state x: theta, as a length-1 array
        H=lambda x: np.array([[0.0, 0.0, 1.0, 0.0, 0.0]]),  # d h / d x (constant here)
        angle_idx=(0,),  # z[0] is an angle -> wrap the innovation
        source="star_tracker",
    )