"""
Multi-kart tracker in the track (Frenet) frame.

State per opponent is [s, d, s_dot, d_dot] relative to the racing line:
s = arc length along it, d = signed lateral offset (left positive). Karts
follow the track, so "constant velocity along the line" is a far better
motion model than constant velocity in x/y: cornering is explained by the
line's geometry instead of showing up as process noise. On a 12 m corner at
7 m/s with 0.3 m / 1 m measurement noise, velocity error was 1.2 / 1.9 m/s
RMS for the best-tuned x/y model and 0.45 / 0.65 m/s for this one. It is
also the frame the MPC already plans in (same xy-derived tangent).

Measurements arrive as map-frame (x, y) + 2x2 covariance (from
kart_ranging.body_to_world) and are projected into (s, d) with a linearized
covariance, so the filter itself stays linear.

Association is Hungarian on a gated Mahalanobis cost, confirmed tracks
first (matching cascade). Detection counts are tiny, so a step is a few
2x2/4x4 ops per track (~0.4 ms for 3 karts + clutter on a laptop).

Track lifecycle:
  tentative -> confirmed after `confirm_hits` hits within `confirm_window_s`
               (a one-frame false positive never reaches the planner)
  confirmed -> deleted after `max_coast_s` without a hit, or the longer
               `max_coast_out_of_fov_s` when the prediction is outside the
               camera's view. A kart alongside us during an overtake is
               invisible to a forward camera, and forgetting it right then
               is the worst possible failure.
"""

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

_H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
_I4 = np.eye(4)
_BIG = 1.0e9


class TrackFrame:
    """Closed racing line as a Frenet reference. Rows are
    (s_m, x_m, y_m, psi_rad, kappa_radpm, ...) like data/racing_line/*.csv."""

    def __init__(self, racing_line: list):
        a = np.asarray([r[:5] for r in racing_line], dtype=np.float64)
        if a.shape[0] < 3:
            raise ValueError("TrackFrame needs a racing line with >= 3 points")
        if np.hypot(*(a[-1, 1:3] - a[0, 1:3])) < 1e-6:  # some lines repeat the first point
            a = a[:-1]
        self.s = a[:, 0] - a[0, 0]
        self.x, self.y, self.kappa = a[:, 1], a[:, 2], a[:, 4]
        # The CSV psi column is not the direction of travel (it is ~pi/2 off,
        # 0 = north). Recompute the tangent from xy like MPCPlanner does, but
        # wrapped across the seam since the loop is closed.
        self.psi = np.arctan2(np.roll(self.y, -1) - np.roll(self.y, 1), np.roll(self.x, -1) - np.roll(self.x, 1))
        self.s_total = float(self.s[-1] + np.hypot(*(a[-1, 1:3] - a[0, 1:3])))
        self.n = a.shape[0]

    def wrap_ds(self, ds):
        """Signed arc-length difference folded into (-L/2, L/2]."""
        L = self.s_total
        return (np.asarray(ds) + 0.5 * L) % L - 0.5 * L

    def _nearest(self, px: np.ndarray, py: np.ndarray, hint_s: Optional[float], window_m: float) -> np.ndarray:
        if hint_s is None:
            idx = np.arange(self.n)
        else:
            near = np.abs(self.wrap_ds(self.s - hint_s)) <= window_m
            idx = np.flatnonzero(near) if near.any() else np.arange(self.n)
        d2 = (px[:, None] - self.x[idx]) ** 2 + (py[:, None] - self.y[idx]) ** 2
        return idx[np.argmin(d2, axis=1)]

    def to_frenet(
        self, xy: np.ndarray, cov: np.ndarray, hint_s: Optional[float] = None, window_m: float = 80.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """(M, 2) map points + (M, 2, 2) covariances -> (M, 2) [s, d] + covariances.

        hint_s restricts the search to +-window_m of arc length (e.g. the ego's
        own s), which keeps a point from snapping to a parallel stretch of
        track on the far side of a hairpin.
        """
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        j = self._nearest(xy[:, 0], xy[:, 1], hint_s, window_m)
        c, s_ = np.cos(self.psi[j]), np.sin(self.psi[j])
        ex, ey = xy[:, 0] - self.x[j], xy[:, 1] - self.y[j]
        along = ex * c + ey * s_
        d = -ex * s_ + ey * c
        s = (self.s[j] + along) % self.s_total
        # ds = along / (1 - kappa d): arc length compresses on the inside of a turn
        scale = 1.0 / np.clip(1.0 - self.kappa[j] * d, 0.2, 5.0)
        J = np.empty((xy.shape[0], 2, 2))
        J[:, 0, 0], J[:, 0, 1] = c * scale, s_ * scale
        J[:, 1, 0], J[:, 1, 1] = -s_, c
        cov_f = J @ np.asarray(cov, dtype=np.float64).reshape(-1, 2, 2) @ np.transpose(J, (0, 2, 1))
        return np.stack([s, d], axis=-1), cov_f

    def _interp(self, s: float) -> Tuple[float, float, float, float]:
        s = s % self.s_total
        i = int(np.searchsorted(self.s, s, side="right") - 1)
        i1 = (i + 1) % self.n
        seg = (self.s[i1] - self.s[i]) if i1 else (self.s_total - self.s[i])
        f = (s - self.s[i]) / seg if seg > 0 else 0.0
        x = self.x[i] + f * (self.x[i1] - self.x[i])
        y = self.y[i] + f * (self.y[i1] - self.y[i])
        dpsi = (self.psi[i1] - self.psi[i] + math.pi) % (2 * math.pi) - math.pi
        return x, y, self.psi[i] + f * dpsi, self.kappa[i] + f * (self.kappa[i1] - self.kappa[i])

    def to_world(self, s: float, d: float, s_dot: float = 0.0, d_dot: float = 0.0) -> Tuple[float, ...]:
        """(s, d, s_dot, d_dot) -> (x, y, vx, vy) in the map frame."""
        x, y, psi, kappa = self._interp(s)
        c, sn = math.cos(psi), math.sin(psi)
        v_along = s_dot * (1.0 - kappa * d)
        return (
            x - d * sn, y + d * c,
            v_along * c - d_dot * sn, v_along * sn + d_dot * c,
        )


@dataclass
class _Track:
    id: int
    x: np.ndarray  # [s, d, s_dot, d_dot]
    P: np.ndarray  # 4x4
    t: float  # time the state refers to
    t_born: float
    t_last_hit: float
    hits: int
    confirmed: bool = False


@dataclass(frozen=True)
class TrackState:
    id: int
    s: float
    d: float
    s_dot: float
    d_dot: float
    x: float  # map frame
    y: float
    vx: float
    vy: float
    speed_mps: float
    heading_rad: float  # NaN while too slow for velocity to define it
    cov: np.ndarray  # 4x4 over [s, d, s_dot, d_dot]
    t: float
    age_s: float
    coast_s: float  # time since last detection


class KartTracker:
    def __init__(self, frame: TrackFrame, params: Optional[dict] = None):
        g = (params or {}).get
        self.frame = frame
        # White-noise acceleration PSDs per axis (along / across the line).
        # Picked on a line6 sim with a 3 m/s^2 brake and a 2 m line change:
        # best or tied-best velocity and 1 s-ahead error at 0.3 m and 1 m
        # measurement noise. Larger values only add jitter.
        self.q_s = float(g("accel_psd_s", 2.0))
        self.q_d = float(g("accel_psd_d", 0.5))
        # chi2(2) 99.97 %. Tighter gates reject ordinary noise tails often
        # enough (0.3 % of frames at 11.8) to split one kart into two tracks.
        self.gate = float(g("gate_chi2", 16.0))
        # Two kart centres cannot physically be closer than this, so anything
        # nearer an existing track is the same kart (duplicate box / outlier).
        self.min_sep = float(g("min_separation_m", 1.0))
        self.confirm_hits = int(g("confirm_hits", 3))
        self.confirm_window_s = float(g("confirm_window_s", 0.3))
        self.max_coast_s = float(g("max_coast_s", 0.4))
        self.max_coast_out_of_fov_s = float(g("max_coast_out_of_fov_s", 2.0))
        self.init_vel_sigma = float(g("init_vel_sigma_mps", 4.0))
        self.init_d_dot_sigma = float(g("init_d_dot_sigma_mps", 1.0))
        self.max_pos_var = float(g("max_pos_var_m2", 25.0))
        self.min_heading_speed = float(g("min_heading_speed_mps", 0.5))
        self.search_window_m = float(g("search_window_m", 80.0))
        self._tracks: List[_Track] = []
        self._next_id = 1

    # Filter primitives

    def _predict(self, tr: _Track, t: float) -> None:
        dt = t - tr.t
        if dt <= 0.0:
            return
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
        d3, d2 = dt ** 3 / 3.0, dt ** 2 / 2.0
        qs, qd = self.q_s, self.q_d
        Q = np.array(
            [[qs * d3, 0, qs * d2, 0], [0, qd * d3, 0, qd * d2],
             [qs * d2, 0, qs * dt, 0], [0, qd * d2, 0, qd * dt]], dtype=np.float64,
        )
        tr.x = F @ tr.x
        tr.x[0] %= self.frame.s_total
        tr.P = F @ tr.P @ F.T + Q
        tr.t = t

    def _innovation(self, tr: _Track, z: np.ndarray) -> np.ndarray:
        return np.array([float(self.frame.wrap_ds(z[0] - tr.x[0])), z[1] - tr.x[1]])

    def _update(self, tr: _Track, z: np.ndarray, R: np.ndarray) -> None:
        S = tr.P[:2, :2] + R
        K = tr.P[:, :2] @ np.linalg.inv(S)
        tr.x = tr.x + K @ self._innovation(tr, z)
        tr.x[0] %= self.frame.s_total
        A = _I4 - K @ _H
        tr.P = A @ tr.P @ A.T + K @ R @ K.T  # Joseph form: stays symmetric PSD

    def _associate(self, tracks: List[_Track], dets: np.ndarray, z: np.ndarray,
                   Rs: np.ndarray, t: float, assigned_det: np.ndarray) -> None:
        """Hungarian on gated Mahalanobis d^2 + log|S|; updates matched tracks."""
        cost = np.full((len(tracks), dets.size), _BIG)
        for i, tr in enumerate(tracks):
            Pp = tr.P[:2, :2]
            for c, j in enumerate(dets):
                S = Pp + Rs[j]
                y = self._innovation(tr, z[j])
                d2 = float(y @ np.linalg.solve(S, y))
                if d2 <= self.gate:
                    cost[i, c] = d2 + math.log(max(np.linalg.det(S), 1e-12))
        for i, c in zip(*linear_sum_assignment(cost)):
            if cost[i, c] >= _BIG:
                continue
            j = dets[c]
            tr = tracks[i]
            self._update(tr, z[j], Rs[j])
            tr.hits += 1
            tr.t_last_hit = t
            assigned_det[j] = True

    # Main step

    def step(
        self,
        t: float,
        meas_xy: np.ndarray,
        meas_cov: np.ndarray,
        ego_s: Optional[float] = None,
        ego_s_dot: Optional[float] = None,
        in_fov: Optional[Callable[[float, float], bool]] = None,
    ) -> List[TrackState]:
        """Advance to time t (s) and fold in this frame's detections.

        meas_xy (M, 2) / meas_cov (M, 2, 2) are map-frame kart centres.
        ego_s bounds the line search to near the ego kart; ego_s_dot seeds new
        tracks (every kart on track is going the same way at roughly our
        speed, a much better prior than zero). in_fov(x, y) says whether a
        map-frame point is inside the camera view.
        Returns confirmed tracks.
        """
        m = int(np.asarray(meas_xy).size // 2)
        if m:
            z, Rs = self.frame.to_frenet(meas_xy, meas_cov, hint_s=ego_s, window_m=self.search_window_m)
        else:
            z, Rs = np.zeros((0, 2)), np.zeros((0, 2, 2))

        for tr in self._tracks:
            self._predict(tr, t)

        # Matching cascade: confirmed tracks pick first, tentative ones only
        # get the leftovers. Otherwise a tentative track born from a noise
        # outlier (large covariance) out-bids the real track for the next
        # few detections, confirms, and steals the kart's identity.
        assigned_det = np.zeros(m, dtype=bool)
        for stage in (True, False):
            cands = [tr for tr in self._tracks if tr.confirmed is stage]
            free = np.flatnonzero(~assigned_det)
            if cands and free.size:
                self._associate(cands, free, z, Rs, t, assigned_det)

        # Lifecycle
        survivors = []
        for tr in self._tracks:
            if not tr.confirmed and tr.hits >= self.confirm_hits:
                tr.confirmed = True
            if not tr.confirmed:
                if t - tr.t_born > self.confirm_window_s:
                    continue
            else:
                limit = self.max_coast_s
                if in_fov is not None and t > tr.t_last_hit:
                    x, y, _, _ = self.frame.to_world(tr.x[0], tr.x[1])
                    if not in_fov(x, y):
                        limit = self.max_coast_out_of_fov_s
                if t - tr.t_last_hit > limit:
                    continue
            if tr.P[0, 0] + tr.P[1, 1] > self.max_pos_var:
                continue
            # Overlaps an older track: same kart, keep the older identity
            if any(math.hypot(*self._innovation(o, tr.x[:2])) < self.min_sep for o in survivors):
                continue
            survivors.append(tr)  # _tracks is in birth order, so older wins
        self._tracks = survivors

        # Births from unassigned detections, unless the detection is the same
        # kart as an existing track (inside its gate or physically too close).
        v0 = 0.0 if ego_s_dot is None else float(ego_s_dot)
        for j in np.flatnonzero(~assigned_det):
            dup = False
            for tr in self._tracks:
                y = self._innovation(tr, z[j])
                if (math.hypot(*y) < self.min_sep
                        or float(y @ np.linalg.solve(tr.P[:2, :2] + Rs[j], y)) <= self.gate):
                    dup = True
                    break
            if dup:
                continue
            P = np.zeros((4, 4))
            P[:2, :2] = Rs[j]
            P[2, 2] = self.init_vel_sigma ** 2
            P[3, 3] = self.init_d_dot_sigma ** 2
            self._tracks.append(
                _Track(
                    id=self._next_id, x=np.array([z[j, 0], z[j, 1], v0, 0.0]), P=P,
                    t=t, t_born=t, t_last_hit=t, hits=1,
                    confirmed=self.confirm_hits <= 1,
                )
            )
            self._next_id += 1

        return self.confirmed(t)

    def confirmed(self, t: float) -> List[TrackState]:
        out = []
        for tr in self._tracks:
            if not tr.confirmed:
                continue
            s, d, s_dot, d_dot = (float(v) for v in tr.x)
            x, y, vx, vy = self.frame.to_world(s, d, s_dot, d_dot)
            speed = math.hypot(vx, vy)
            out.append(
                TrackState(
                    id=tr.id, s=s, d=d, s_dot=s_dot, d_dot=d_dot,
                    x=x, y=y, vx=vx, vy=vy, speed_mps=speed,
                    heading_rad=math.atan2(vy, vx) if speed >= self.min_heading_speed else math.nan,
                    cov=tr.P.copy(), t=tr.t, age_s=t - tr.t_born, coast_s=t - tr.t_last_hit,
                )
            )
        return out

    def reset(self) -> None:
        self._tracks = []
