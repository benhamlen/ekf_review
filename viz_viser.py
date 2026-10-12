"""Viser front end for the EKF playground.

Reads only TruthState and EstimateSnapshot -- never touches filter internals.
Open http://localhost:8080 in a browser (WSL2 forwards localhost to Windows).
"""

import time
from collections.abc import Iterable

import numpy as np
import viser

from ekf_review_refined import EstimateSnapshot, TruthState, Vec

# state layout assumed for plotting (matches UnicycleBiasModel: [x, y, theta, v, b_g])
IDX_X, IDX_Y, IDX_THETA = 0, 1, 2

TRUTH_COLOR = (20, 20, 20)
DEFAULT_COLORS = [(31, 119, 180), (214, 39, 40), (44, 160, 44), (148, 103, 189)]
UPDATE_MARKER_COLOR = (255, 190, 0)
Z_LIFT = 0.02  # draw slightly above the grid so lines aren't z-fighting with it


def _polyline_to_segments(xy: Vec) -> Vec:
    """(N, 2) polyline -> (N-1, 2, 3) line segments in the z=Z_LIFT plane."""
    xyz = np.column_stack([xy, np.full(len(xy), Z_LIFT)])
    return np.stack([xyz[:-1], xyz[1:]], axis=1)


def covariance_ellipse(center: Vec, P_xy: Vec, n_sigma: float = 3.0, n_pts: int = 64) -> Vec:
    """Points (n_pts, 2) on the n-sigma ellipse of a 2x2 position covariance."""
    eigvals, eigvecs = np.linalg.eigh(P_xy)
    angles = np.linspace(0.0, 2 * np.pi, n_pts)
    unit_circle = np.stack([np.cos(angles), np.sin(angles)])  # (2, n_pts)
    axes = eigvecs @ np.diag(n_sigma * np.sqrt(np.maximum(eigvals, 0.0)))
    return (center[:, None] + axes @ unit_circle).T


class ViserViz:
    """Live 2D (on the ground plane) view of truth vs. estimators.

    Call log_step() once per sim step; it redraws every `redraw_every` steps and
    sleeps so playback runs at the speed set by the GUI slider.
    """

    def __init__(self, port: int = 8080, redraw_every: int = 10, heading_arrow_len: float = 1.5) -> None:
        self.server = viser.ViserServer(port=port)
        self.server.scene.set_up_direction("+z")
        self.server.scene.add_grid("/grid", width=60.0, height=60.0, plane="xy", cell_size=1.0, section_size=5.0)

        self.redraw_every = redraw_every
        self.heading_arrow_len = heading_arrow_len

        self._time_md = self.server.gui.add_markdown("**t = 0.0 s**")
        self._speed = self.server.gui.add_slider("playback speed (x real time)", min=1, max=50, step=1, initial_value=10)
        self._show_cov = self.server.gui.add_checkbox("show 3σ position ellipses", initial_value=True)
        self._legend = self.server.gui.add_markdown("")

        self._truth_xy: list[tuple[float, float]] = []
        self._est_xy: dict[str, list[tuple[float, float]]] = {}
        self._colors: dict[str, tuple[int, int, int]] = {}
        self._update_xy: list[tuple[float, float]] = []
        self._step = 0
        self._last_draw_t: float | None = None

    def _color_for(self, name: str) -> tuple[int, int, int]:
        if name not in self._colors:
            self._colors[name] = DEFAULT_COLORS[len(self._colors) % len(DEFAULT_COLORS)]
            legend_lines = ["**legend**", "- ⬛ truth"] + [
                f"- <span style='color:rgb{c}'>■</span> {n}" for n, c in self._colors.items()
            ] + ["- ★ measurement update"]
            self._legend.content = "\n".join(legend_lines)
        return self._colors[name]

    def log_step(
        self,
        truth: TruthState,
        estimates: dict[str, EstimateSnapshot],
        updated: Iterable[str] = (),
    ) -> None:
        """Record one step. `updated` names the estimators that applied a measurement this step."""
        self._truth_xy.append((truth.x, truth.y))
        for name, snap in estimates.items():
            self._color_for(name)
            self._est_xy.setdefault(name, []).append((snap.x[IDX_X], snap.x[IDX_Y]))
        for name in updated:
            snap = estimates[name]
            self._update_xy.append((snap.x[IDX_X], snap.x[IDX_Y]))

        if self._step % self.redraw_every == 0:
            self._draw(truth, estimates)
        self._step += 1

    def _draw(self, truth: TruthState, estimates: dict[str, EstimateSnapshot]) -> None:
        scene = self.server.scene

        if len(self._truth_xy) >= 2:
            scene.add_line_segments(
                "/truth/trail", _polyline_to_segments(np.array(self._truth_xy)), TRUTH_COLOR, thickness=0.08
            )
        self._draw_pose("/truth/pose", np.array([truth.x, truth.y]), truth.theta, TRUTH_COLOR)

        for name, snap in estimates.items():
            color = self._colors[name]
            xy = np.array(self._est_xy[name])
            if len(xy) >= 2:
                scene.add_line_segments(f"/est/{name}/trail", _polyline_to_segments(xy), color, thickness=0.06)
            self._draw_pose(f"/est/{name}/pose", snap.x[[IDX_X, IDX_Y]], snap.x[IDX_THETA], color)

            ellipse = covariance_ellipse(snap.x[[IDX_X, IDX_Y]], snap.P[np.ix_([IDX_X, IDX_Y], [IDX_X, IDX_Y])])
            scene.add_line_segments(
                f"/est/{name}/cov3sigma",
                _polyline_to_segments(ellipse),
                color,
                thickness=0.04,
                visible=self._show_cov.value,
            )

        if self._update_xy:
            pts = np.column_stack([np.array(self._update_xy), np.full(len(self._update_xy), 2 * Z_LIFT)])
            scene.add_point_cloud("/updates", pts.astype(np.float32), UPDATE_MARKER_COLOR, point_size=0.35, point_shape="sparkle")

        self._time_md.content = f"**t = {truth.t:.1f} s**"

        # pace playback: sleep the sim time elapsed since the last draw, scaled by the speed slider
        if self._last_draw_t is not None:
            time.sleep(max(0.0, (truth.t - self._last_draw_t) / self._speed.value))
        self._last_draw_t = truth.t

    def _draw_pose(self, name: str, xy: Vec, theta: float, color: tuple[int, int, int]) -> None:
        start = np.array([xy[0], xy[1], 2 * Z_LIFT])
        tip = start + self.heading_arrow_len * np.array([np.cos(theta), np.sin(theta), 0.0])
        self.server.scene.add_arrows(
            name, np.array([[start, tip]]), color, shaft_radius=0.06, head_radius=0.18, head_length=0.35
        )

    def hold(self) -> None:
        """Keep the server alive after the sim finishes so the final frame stays viewable."""
        print("sim done -- viewer still running at http://localhost:8080 (Ctrl+C to exit)")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    # Self-test with FAKE estimates (no filter): checks the viewer, not the EKF.
    viz = ViserViz()
    dt, radius, omega = 0.1, 10.0, 0.1
    for k in range(1200):
        t = k * dt
        truth = TruthState(t=t, x=radius * np.sin(omega * t), y=radius * (1 - np.cos(omega * t)),
                           theta=omega * t, v=1.0, omega=omega)
        fake_x = np.array([truth.x + 0.3 * np.sin(0.5 * t), truth.y, truth.theta + 0.05, 1.0, 0.0])
        fake_P = np.diag([0.01 + 0.002 * t, 0.01 + 0.001 * t, 0.01, 0.01, 0.0001])
        viz.log_step(truth, {"fake": EstimateSnapshot(t, fake_x, fake_P)}, updated=["fake"] if k % 10 == 0 else ())
    viz.hold()
