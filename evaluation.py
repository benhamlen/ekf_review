"""Step 5: estimator evaluation against simulation ground truth.

Pure NumPy, no plotting -- plot_eval.py consumes these arrays.
State layout: [x, y, theta, v, b_g] (UnicycleBiasModel).
"""

import numpy as np

from ekf_review_refined import EstimateSnapshot, TruthState, Vec, normalize_angle


def truth_state_vectors(truth_hist: list[TruthState], bias_true: float) -> Vec:
    """(N, 5) true state in the *filter's* layout, so it can be subtracted from estimates.

    HINT: TruthState has x, y, theta, v -- but no bias. The true bias is a sensor
          property, so it's passed in. Build rows [x, y, theta, v, bias_true].
    """
    
    return np.array([[t.x, t.y, t.theta, t.v, bias_true] for t in truth_hist])


def state_errors(truth_hist: list[TruthState], est_hist: list[EstimateSnapshot], bias_true: float) -> Vec:
    """(N, 5) estimation error e_k = x̂_k − x_k.

    HINT: stack the estimates into an (N, 5) array, subtract truth_state_vectors(...).
    HINT: one column needs special treatment -- which one, and why? (truth theta is unwrapped)
    """


    errors = np.array([s.x for s in est_hist]) - truth_state_vectors(truth_hist, bias_true)

    # remember to do angle wraping
    errors[:,2] = normalize_angle(errors[:,2])

    return errors


def sigmas(est_hist: list[EstimateSnapshot]) -> Vec:
    """(N, 5) 1-σ per state = sqrt of the diagonal of each P_k.

    HINT: np.diagonal works on a stacked (N, 5, 5) array with axis1=1, axis2=2.
    """
    return np.array([np.sqrt(np.diagonal(state_est.P)) for state_est in est_hist])


def nees(errors: Vec, est_hist: list[EstimateSnapshot]) -> Vec:
    """(N,) normalized estimation error squared: ε_k = e_kᵀ P_k⁻¹ e_k.

    HINT: solve, don't invert:  e_kᵀ (P_k⁻¹ e_k) = e_k @ np.linalg.solve(P_k, e_k)
    HINT: a loop over k is fine; np.einsum is the vectorized bonus.
    WHAT IT MEANS: if the filter is consistent (errors are zero-mean and P is honest),
          ε_k ~ χ²(n) with n = 5, so its mean should be ≈ 5.
          Mean >> n: overconfident (P too small -- usually Q or R too small).
          Mean << n: conservative (P too big).
    """
    nees_values = np.empty(len(est_hist))
    for k, (e_k, snap) in enumerate(zip(errors, est_hist)):
        # P_k⁻¹ e_k without forming the inverse; then dot with e_k -> scalar
        nees_values[k] = e_k @ np.linalg.solve(snap.P, e_k)
    return nees_values
    


