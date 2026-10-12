from dataclasses import dataclass, field

import numpy as np
from matplotlib import pyplot as plt
from numpy.typing import NDArray


@dataclass
class Estimator:
    x: NDArray[np.float64]
    P: NDArray[np.float64]
    history_x: list = field(default_factory=list)

rng: np.random.Generator = np.random.default_rng(42)

I: NDArray[np.float64] = np.eye(5)

# motion model f(x, u): constant-velocity unicycle driven by measured yaw rate
def get_f(
    x: float, y: float, theta: float, v: float, bg: float, dt: float, gyro_z: float
) -> NDArray[np.float64]:
    return np.array([
        x + v*np.cos(theta)*dt,
        y + v*np.sin(theta)*dt,
        theta + (gyro_z - bg)*dt,
        v,
        bg
    ])

# Jacobian F = df/dx, evaluated at the state *before* prediction
def get_F(v: float, theta: float, dt: float) -> NDArray[np.float64]:
    return np.array([[1, 0, -v*np.sin(theta)*dt, np.cos(theta)*dt, 0],
                            [0, 1, v*np.cos(theta)*dt, np.sin(theta)*dt, 0],
                            [0, 0, 1, 0, -dt],
                            [0,0,0,1,0],
                            [0,0,0,0,1]])

def predict(
    state: NDArray[np.float64],
    P: NDArray[np.float64],
    gyro_z: float,
    dt: float,
    Q: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    x, y, theta, v, bg = state
    
    # propagate state from dynamics and known control inputs
    state_updated: NDArray[np.float64] = get_f(x, y, theta, v, bg, dt, gyro_z)
    
    # propagate state uncertainty
    F: NDArray[np.float64] = get_F(v, theta, dt)
    P_updated: NDArray[np.float64] = F@P@F.T + Q

    # normalize angle to [-pi,pi)
    state_updated[2] = normalize_angle(state_updated[2])

    return state_updated, P_updated


def update(
    x: NDArray[np.float64],
    P: NDArray[np.float64],
    z: NDArray[np.float64],
    H: NDArray[np.float64],
    R: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    
    # innovation: actual measurement minus predicted measurement h(x) = Hx
    # (wrap y[0] because it's an angle; this assumes a heading-only measurement)
    y: NDArray[np.float64] = z - H@x
    y[0] = normalize_angle(y[0])

    # innovation covariance: predicted spread of y (state uncertainty mapped through H, plus sensor noise)
    S: NDArray[np.float64] = H@P@H.T + R

    # Kalman gain K = P H^T S^-1, computed via solve() instead of an explicit inverse
    # K = P@H.T@np.linalg.inv(S)
    K: NDArray[np.float64] = np.linalg.solve(S, H @ P).T

    # calculate updated state estimate
    x_plus: NDArray[np.float64] = x + K@y
    x_plus[2] = normalize_angle(x_plus[2])

    # covariance update, Joseph form (stays symmetric PSD under rounding)
    P_plus: NDArray[np.float64] = (I - K@H)@P@(I-K@H).T+K@R@K.T

    return x_plus, P_plus


def normalize_angle(theta: float) -> float:
    return (theta + np.pi) % (2 * np.pi) - np.pi


if __name__ == "__main__":

    # constants
    BIAS_TRUE: float = 0.02        # yaw gyro truth bias (rad/s)
    SIGMA_GYRO: float = 0.01       # yaw gyro white-noise std per sample at 10 Hz (rad/s)
    SIGMA_HEADING: float = 0.005   # rad, star-tracker-derived heading
    OMEGA_TRUE: float = 0.1 # true yaw *rate* of vehicle (rad/s)
    dt: float = 0.1 # sim timestep (seconds)
    V_TRUE: float = 1.0 # truth vehicle forward velocity (m/s)
    DURATION_S: float = 120.0 # sim duration (seconds)
    EKF_UPDATE_INTERVAL_STEPS = 10 # star-tracker heading update every 10 steps = 1 Hz

    # system matrices
    H: NDArray[np.float64] = np.array([[0., 0., 1., 0., 0.]]) # measurement Jacobian: rows are dh_i/dx for each measured quantity (here h(x) = theta, so h(x) = Hx exactly)
    R: NDArray[np.float64] = np.array([[SIGMA_HEADING**2]]) # measurement noise covariance (star-tracker heading noise)
    Q: NDArray[np.float64] = np.diag([0.001, 0.001, 0.0001, 0.01, 0.00001]) # process noise covariance: uncertainty added each predict step from unmodeled dynamics
    # and noisy control inputs (gyro noise belongs here because the gyro drives predict, not update)

    # state and covariance
    x0 = np.array([0., 0., 0., 1., 0.]) # initial states
    P0 = np.eye(5)# initial state covariances
    ekf = Estimator(x0.copy(), P0.copy())
    dr = Estimator(x0.copy(), P0.copy())

    # precompute true trajectory and sensor readings
    time_vec: NDArray[np.float64] = np.arange(0, DURATION_S, dt) # 1200 steps of dt
    NUM_STEPS: int = len(time_vec)
    truth_states: NDArray[np.float64] = np.array([(V_TRUE/OMEGA_TRUE)*np.sin(OMEGA_TRUE*time_vec), (V_TRUE/OMEGA_TRUE)*(1-np.cos(OMEGA_TRUE*time_vec)), OMEGA_TRUE*time_vec]).T
    gyro_input: NDArray[np.float64] = OMEGA_TRUE + BIAS_TRUE + rng.normal(0, SIGMA_GYRO, NUM_STEPS)
    heading_meas: NDArray[np.float64] = truth_states[:,2] + rng.normal(0, SIGMA_HEADING, NUM_STEPS)

    assert(len(truth_states) == len(time_vec))

    print(f"t shape: {time_vec.shape}")
    print(f"gyro array shape: {gyro_input.shape}")
    print(f"truth states shape: {truth_states.shape}")

    # live visualization setup
    PLOT_EVERY_STEPS: int = 10 # redraw once per sim second; redrawing every step is slow
    plt.ion()
    fig, ax = plt.subplots()
    ax.plot(truth_states[:, 0], truth_states[:, 1], color="0.85", lw=1, label="truth (full path)")
    (truth_line,) = ax.plot([], [], "k-", lw=1.5, label="truth")
    (ekf_line,) = ax.plot([], [], "C0-", lw=1.5, label="EKF")
    (dr_line,) = ax.plot([], [], "C3--", lw=1.5, label="dead reckoning")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.legend(loc="upper right")

    for (step_num, (time, truth_state)) in enumerate(zip(time_vec, truth_states)):
        print(f"----\nstep {step_num} ({time:.4f}s) -- truth state: {truth_state}")

        if step_num == 0:
            print("skipping first step--kluge to enable convenient prediction at step_num-1 and update at step_num")
            continue

        # predict both EKF and DR with the same noisy, biased gyro reading (estimators never see OMEGA_TRUE)
        for est in (ekf, dr):
            est.x, est.P = predict(est.x, est.P, gyro_input[step_num-1], dt, Q)

        # update EKF every N steps with heading measurement
        if step_num % EKF_UPDATE_INTERVAL_STEPS == 0:
            ekf.x, ekf.P = update(ekf.x, ekf.P, heading_meas[step_num], H, R) 
            ax.scatter(ekf.x[0], ekf.x[1], marker="*", s=150, color="C0", edgecolors="k", linewidths=0.5, zorder=5)

        # save x_ekf, P_ekf, x_dr, P_dr

        # live visualization: append current positions to each trail
        for line, (px, py) in [(truth_line, truth_states[step_num, :2]), (ekf_line, ekf.x[:2]), (dr_line, dr.x[:2])]:
            line.set_data(np.append(line.get_xdata(), px), np.append(line.get_ydata(), py))
        if step_num % PLOT_EVERY_STEPS == 0:
            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"t = {time:.1f} s")
            plt.pause(0.001)
        




    # keep the final frame open
    plt.ioff()
    plt.show()
    
