"""Step 5 plots: error vs. time with ±3σ envelopes, bias convergence, NEES consistency.

Run:  pixi run python plot_eval.py            (window + saves eval.png)
      pixi run python plot_eval.py --no-show  (just saves eval.png)
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2

from evaluation import nees, sigmas, state_errors
from sim import SimResult, run_sim

IDX_X, IDX_Y, IDX_THETA, IDX_BG = 0, 1, 2, 4
N_STATE = 5
EKF_COLOR, DR_COLOR, TRUTH_COLOR = "C0", "C3", "0.2"
BAND_ALPHA = 0.18


def plot_evaluation(result: SimResult, out_path: str = "eval.png") -> plt.Figure:
    t = np.array([s.t for s in result.truth])
    err_ekf = state_errors(result.truth, result.ekf, result.bias_true)
    err_dr = state_errors(result.truth, result.dr, result.bias_true)
    sig_ekf = sigmas(result.ekf)
    nees_ekf = nees(err_ekf, result.ekf)
    bias_est = np.array([s.x[IDX_BG] for s in result.ekf])

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True, constrained_layout=True)
    ax_head, ax_bias, ax_pos, ax_nees = axes

    # 1. heading error with ±3σ envelope (EKF only -- DR error is ~100x larger, see panel 3 for scale)
    ax_head.fill_between(t, -3 * sig_ekf[:, IDX_THETA], 3 * sig_ekf[:, IDX_THETA],
                         color=EKF_COLOR, alpha=BAND_ALPHA, linewidth=0, label="EKF ±3σ")
    ax_head.plot(t, err_ekf[:, IDX_THETA], color=EKF_COLOR, lw=1.2, label="EKF error")
    ax_head.axhline(0, color="0.5", lw=0.8)
    # P0 = I makes the first seconds' band ±3 rad; scale the axis to the settled part so the error is visible
    settled = t > 5.0
    ax_head.set_ylim(np.array([-1.5, 1.5]) * 3 * sig_ekf[settled, IDX_THETA].max())
    ax_head.set_ylabel("heading error (rad)")
    ax_head.set_title("Heading error — should stay inside its ±3σ band")
    ax_head.legend(loc="upper right")

    # 2. gyro-bias estimate vs. truth, with ±3σ around the estimate
    ax_bias.fill_between(t, bias_est - 3 * sig_ekf[:, IDX_BG], bias_est + 3 * sig_ekf[:, IDX_BG],
                         color=EKF_COLOR, alpha=BAND_ALPHA, linewidth=0, label="EKF ±3σ")
    ax_bias.plot(t, bias_est, color=EKF_COLOR, lw=1.2, label="EKF bias estimate")
    ax_bias.axhline(result.bias_true, color=TRUTH_COLOR, lw=1.2, ls="--", label=f"true bias ({result.bias_true})")
    ax_bias.set_ylabel("gyro bias (rad/s)")
    ax_bias.set_title("Bias observability — inferred from heading updates alone")
    ax_bias.set_ylim(-0.02, 0.05)
    ax_bias.legend(loc="upper right")

    # 3. horizontal position error, log scale so EKF and DR fit on one axis
    pos_err_ekf = np.linalg.norm(err_ekf[:, [IDX_X, IDX_Y]], axis=1)
    pos_err_dr = np.linalg.norm(err_dr[:, [IDX_X, IDX_Y]], axis=1)
    pos_3sig_ekf = 3 * np.sqrt(sig_ekf[:, IDX_X] ** 2 + sig_ekf[:, IDX_Y] ** 2)
    ax_pos.plot(t, pos_err_dr, color=DR_COLOR, lw=1.2, ls="--", label="dead reckoning")
    ax_pos.plot(t, pos_err_ekf, color=EKF_COLOR, lw=1.2, label="EKF")
    ax_pos.plot(t, pos_3sig_ekf, color=EKF_COLOR, lw=0.8, ls=":", label="EKF 3σ (radial, approx.)")
    ax_pos.set_yscale("log")
    ax_pos.set_ylabel("position error (m)")
    ax_pos.set_title("Horizontal position error — no position measurements, so nothing bounds it")
    ax_pos.legend(loc="lower right")

    # 4. NEES with two-sided 95% χ² bounds for a single run
    lo, hi = chi2.ppf([0.025, 0.975], N_STATE)
    inside = np.mean((nees_ekf >= lo) & (nees_ekf <= hi))
    ax_nees.axhspan(lo, hi, color="0.85", label=f"95% χ²({N_STATE}) bounds")
    ax_nees.plot(t, nees_ekf, color=EKF_COLOR, lw=1.0, label="EKF NEES")
    ax_nees.axhline(N_STATE, color="0.4", lw=0.8, ls="--", label=f"expected mean = {N_STATE}")
    ax_nees.set_yscale("log")
    ax_nees.set_ylabel("NEES")
    ax_nees.set_xlabel("time (s)")
    ax_nees.set_title(f"Consistency — mean NEES {np.mean(nees_ekf):.2f}, {100 * inside:.0f}% of steps inside bounds")
    ax_nees.legend(loc="upper right")

    for ax in axes:
        ax.grid(True, color="0.9", lw=0.6)
        ax.spines[["top", "right"]].set_visible(False)

    fig.savefig(out_path, dpi=120)
    print(f"saved {out_path}")
    return fig


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    plot_evaluation(run_sim(use_viz=False))
    if not args.no_show:
        plt.show()
