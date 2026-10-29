"""
Opponent karts for the planner: state, track limits, prediction, tactics.

Opponents arrive as Frenet states on the static racing line (the tracker in
opencv_pathfinder already works in that frame). This module turns them into
what MPCPlanner needs each tick:

  predict()      world (x, y, heading) of every opponent at every MPC horizon
                 step: constant speed along the line, lateral velocity decayed
                 (karts don't drift sideways forever), clamped to the track.
  TrackBounds    real left/right room at every racing-line vertex, from the
                 track KML. The track is ~5 m wide and the line comes within
                 0.6 m of the edge at apexes, so "is there room to pass on
                 this side" has to use the real edges, not a symmetric corridor.
  PassPlanner    pass / follow decision with hysteresis -> lateral offset
                 d_ref the MPC tracks instead of the line (d = 0).

Collision avoidance itself is a cost inside MPCPlanner._solve; this module
only decides where the kart wants to be.
"""

import math
import re
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import TrackFrame

_EARTH_R = 6378137.0


@dataclass(frozen=True)
class Opponent:
    id: int
    s: float  # arc length on the static racing line (m)
    d: float  # lateral offset, left positive (m)
    s_dot: float  # m/s along the line
    d_dot: float  # m/s across it
    stamp_s: float  # time the state refers to (s, planner clock)
    # 1-sigma position uncertainty along / across the line (tracker
    # covariance). The planner widens its keep-out box by these.
    sigma_s: float = 0.0
    sigma_d: float = 0.0


# Wire format for /opponents (std_msgs/Float64MultiArray): [n, then n rows of
# OPP_FIELDS]. Float64 because stamps are ROS-clock seconds (~1.7e9), which a
# float32 can only resolve to ~2 minutes.

OPP_FIELDS = 8  # id, s, d, s_dot, d_dot, stamp_s, sigma_s, sigma_d


def encode_opponents(opps: Sequence[Opponent]) -> List[float]:
    out = [float(len(opps))]
    for o in opps:
        out += [float(o.id), o.s, o.d, o.s_dot, o.d_dot, o.stamp_s, o.sigma_s, o.sigma_d]
    return out


def decode_opponents(data: Sequence[float]) -> Tuple[Opponent, ...]:
    if len(data) == 0:
        return ()
    n = int(data[0])
    if n < 0 or len(data) != 1 + n * OPP_FIELDS:
        raise ValueError(f"/opponents payload: {len(data)} values for n={n}")
    rows = (data[1 + i * OPP_FIELDS: 1 + (i + 1) * OPP_FIELDS] for i in range(n))
    return tuple(Opponent(int(r[0]), *(float(v) for v in r[1:])) for r in rows)


# Track limits


def _kml_polylines(path: str, lat0: float, lon0: float) -> dict:
    """Placemark name -> (N, 2) local ENU metres (same origin as gps_node)."""
    text = open(path).read()
    k = math.cos(math.radians(lat0))
    out = {}
    for pm in re.findall(r"<Placemark.*?</Placemark>", text, re.S):
        name = re.search(r"<name>([^<]*)</name>", pm)
        coords = re.search(r"<coordinates>(.*?)</coordinates>", pm, re.S)
        if not (name and coords):
            continue
        ll = np.array([[float(v) for v in p.split(",")[:2]] for p in coords.group(1).split()])
        out[name.group(1)] = np.column_stack([
            np.radians(ll[:, 0] - lon0) * _EARTH_R * k,
            np.radians(ll[:, 1] - lat0) * _EARTH_R,
        ])
    return out


def _nearest_on_polyline(p: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """(n, 2) points -> nearest (n, 2) points on a closed polyline."""
    a = poly
    b = np.roll(poly, -1, axis=0)
    ab = b - a
    ab2 = np.maximum((ab * ab).sum(1), 1e-12)
    t = np.clip(((p[:, None, :] - a[None]) * ab[None]).sum(-1) / ab2, 0.0, 1.0)  # (n, m)
    q = a[None] + t[..., None] * ab[None]
    j = np.argmin(((p[:, None, :] - q) ** 2).sum(-1), axis=1)
    return q[np.arange(p.shape[0]), j]


class TrackBounds:
    """Room to the track edge on each side of each racing-line vertex.

    left[i] / right[i] are distances (m, >= 0) from vertex i to the left
    (+d) and right (-d) edges, so the drivable band there is
    d in [-right[i], left[i]].
    """

    def __init__(self, left: np.ndarray, right: np.ndarray, frame: TrackFrame):
        self.left = np.asarray(left, dtype=np.float64)
        self.right = np.asarray(right, dtype=np.float64)
        self.frame = frame

    @classmethod
    def symmetric(cls, frame: TrackFrame, half_width: float) -> "TrackBounds":
        full = np.full(frame.n, half_width)
        return cls(full, full.copy(), frame)

    @classmethod
    def from_kml(cls, path: str, frame: TrackFrame, lat0: float, lon0: float) -> "TrackBounds":
        polys = list(_kml_polylines(path, lat0, lon0).values())
        if len(polys) < 2:
            raise ValueError(f"{path}: need two boundary polylines, found {len(polys)}")
        p = np.column_stack([frame.x, frame.y])
        nrm = np.column_stack([-np.sin(frame.psi), np.cos(frame.psi)])
        left = np.full(frame.n, np.inf)
        right = np.full(frame.n, np.inf)
        for poly in polys:
            off = _nearest_on_polyline(p, poly) - p
            signed = (off * nrm).sum(1)
            dist = np.hypot(off[:, 0], off[:, 1])
            left = np.where(signed > 0, np.minimum(left, dist), left)
            right = np.where(signed <= 0, np.minimum(right, dist), right)
        if not (np.isfinite(left).all() and np.isfinite(right).all()):
            raise ValueError(f"{path}: racing line is not between the two boundaries everywhere")
        return cls(left, right, frame)

    def room(self, s) -> Tuple[np.ndarray, np.ndarray]:
        """(left, right) room at arc length(s), linear between vertices."""
        f = self.frame
        s = np.asarray(s, dtype=np.float64) % f.s_total
        i = np.searchsorted(f.s, s, side="right") - 1
        i1 = (i + 1) % f.n
        seg = np.where(i1 > 0, f.s[i1] - f.s[i], f.s_total - f.s[i])
        w = np.where(seg > 0, (s - f.s[i]) / np.where(seg > 0, seg, 1.0), 0.0)
        return (self.left[i] + w * (self.left[i1] - self.left[i]),
                self.right[i] + w * (self.right[i1] - self.right[i]))


# Prediction


def predict(
    frame: TrackFrame,
    opps: Sequence[Opponent],
    now_s: float,
    t_steps: np.ndarray,
    d_dot_tau_s: float = 0.5,
    bounds: Optional[TrackBounds] = None,
    kart_half_width: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World (x, y, heading) arrays of shape (M, N) at times now_s + t_steps.

    Propagates each state from its own stamp, so tracker latency is
    compensated. Lateral velocity decays with time constant d_dot_tau_s.
    """
    m, n = len(opps), len(t_steps)
    if m == 0:
        z = np.zeros((0, n))
        return z, z, z
    s0 = np.array([o.s for o in opps])[:, None]
    d0 = np.array([o.d for o in opps])[:, None]
    vs = np.array([o.s_dot for o in opps])[:, None]
    vd = np.array([o.d_dot for o in opps])[:, None]
    lag = now_s - np.array([o.stamp_s for o in opps])[:, None]
    t = np.maximum(lag + np.asarray(t_steps)[None, :], 0.0)
    s = s0 + vs * t
    d = d0 + vd * d_dot_tau_s * (1.0 - np.exp(-t / max(d_dot_tau_s, 1e-3)))
    if bounds is not None:
        left, right = bounds.room(s)
        d = np.clip(d, -right + kart_half_width, left - kart_half_width)
    x, y, psi, _ = frame.pose_at(s)
    return x - d * np.sin(psi), y + d * np.cos(psi), psi


# Tactics


FREE, FOLLOW, PASS = "free", "follow", "pass"


class PassPlanner:
    """Decides where across the track the ego kart should be.

    FREE   no kart in the way: d_ref = 0 (racing line).
    FOLLOW kart ahead in our lane and no side has room: d_ref = 0; the MPC's
           opponent cost + speed cap keep the gap.
    PASS   committed to one side of a target kart until we are a kart length
           past it, then back to FREE (the opponent cost stops us cutting
           back across its nose).
    """

    def __init__(self, params: Optional[dict] = None):
        g = (params or {}).get
        self.kart_length = float(g("kart_length_m", 1.8))
        self.kart_width = float(g("kart_width_m", 1.0))
        self.lat_gap = float(g("pass_lateral_gap_m", 0.5))  # side-by-side clearance
        self.edge_margin = float(g("edge_margin_m", 0.2))
        self.lookahead_s = float(g("pass_lookahead_s", 3.0))
        self.lookahead_min = float(g("pass_lookahead_min_m", 12.0))
        self.min_closing = float(g("pass_min_closing_mps", 0.5))
        self.clear_m = float(g("pass_clear_m", 1.5))
        self.room_window_m = float(g("pass_room_window_m", 20.0))
        # A kart this far ahead of the target leaves a slot to drop into after
        # the pass, so it does not block that side.
        self.slot_gap_m = float(g("pass_slot_gap_m", 6.0))
        # How long a pass target that dropped out of view is held as a ghost.
        # A single forward camera cannot see a kart matching our speed beside
        # us, so this must outlast being boxed in alongside (3 s did not).
        self.lost_hold_s = float(g("pass_lost_hold_s", 8.0))
        self._last_tgt: Optional[Opponent] = None
        self.ghost: Optional[Opponent] = None
        self.state = FREE
        self.side = 0
        self.target_id: Optional[int] = None
        self.d_pass = 0.0

    def _lane_room(self, bounds: TrackBounds, s0: float) -> Tuple[float, float]:
        """Tightest left/right room over the stretch a pass would use."""
        left, right = bounds.room(s0 + np.linspace(0.0, self.room_window_m, 11))
        return float(left.min()), float(right.min())

    def _side_ok(self, side: int, tgt: Opponent, others: List[Opponent], ego_s: float,
                 bounds: TrackBounds, frame: TrackFrame) -> Optional[float]:
        d_pass = tgt.d + side * (self.kart_width + self.lat_gap)
        left, right = self._lane_room(bounds, ego_s)
        half = 0.5 * self.kart_width + self.edge_margin
        if not (-right + half <= d_pass <= left - half):
            return None
        for o in others:  # someone already in that lane near the target
            ds = float(frame.wrap_ds(o.s - tgt.s))
            # Widen by the other kart's uncertainty: a track coasting out of
            # view (beside us) can be metres off along the line, and "it is
            # probably behind the target" is not good enough to move over.
            ms, md = 2.0 * o.sigma_s, o.sigma_d
            if (-self.kart_length * 2 - ms < ds < self.slot_gap_m + ms
                    and abs(o.d - d_pass) < self.kart_width + self.lat_gap + md):
                return None
        return d_pass

    def update(self, ego_s: float, ego_d: float, ego_v: float, opps: Sequence[Opponent],
               frame: TrackFrame, bounds: TrackBounds, now_s: Optional[float] = None) -> float:
        """Opponent states must already be propagated to now. Returns d_ref."""
        by_id = {o.id: o for o in opps}
        if now_s is None:
            now_s = max((o.stamp_s for o in opps), default=0.0)
        self.ghost = None
        look = max(self.lookahead_min, ego_v * self.lookahead_s)

        if self.state == PASS:
            tgt = by_id.get(self.target_id)
            if tgt is not None:
                self._last_tgt = tgt
            elif self._last_tgt is not None and self._last_tgt.stamp_s + self.lost_hold_s >= now_s:
                # Lost is not passed: a kart beside us is outside the camera.
                # Carry on against its extrapolated ghost (also exposed so the
                # planner keeps it in the collision cost).
                lt = self._last_tgt
                tgt = Opponent(lt.id, lt.s + lt.s_dot * (now_s - lt.stamp_s), lt.d, lt.s_dot, 0.0, now_s,
                               lt.sigma_s + 0.5 * (now_s - lt.stamp_s), lt.sigma_d + 0.2 * (now_s - lt.stamp_s))
                self.ghost = tgt
            ds_t = float(frame.wrap_ds(tgt.s - ego_s)) if tgt is not None else -math.inf
            if ds_t < -(self.kart_length + self.clear_m):
                self.state, self.side, self.target_id = FREE, 0, None  # done (or target lost)
            else:
                others = [o for o in opps if o.id != tgt.id]
                d_pass = self._side_ok(self.side, tgt, others, ego_s, bounds, frame)
                if d_pass is not None:
                    self.d_pass = d_pass
                    return d_pass
                if ds_t < self.kart_length + self.clear_m:
                    # Side closed while we are alongside: merging back now
                    # means steering into the target. Hold the lane; the
                    # opponent cost brakes us until we are behind it.
                    return self.d_pass
                self.state, self.side, self.target_id = FOLLOW, 0, None  # still behind: abort safely

        # Nearest kart ahead that overlaps our lane and that we are catching
        blocker = None
        for o in opps:
            ds = float(frame.wrap_ds(o.s - ego_s))
            if 0.0 < ds < look and abs(o.d - ego_d) < self.kart_width + self.lat_gap:
                if ego_v - o.s_dot > self.min_closing or ds < 2.0 * self.kart_length:
                    if blocker is None or ds < float(frame.wrap_ds(blocker.s - ego_s)):
                        blocker = o
        if blocker is None:
            self.state = FREE
            return 0.0

        others = [o for o in opps if o.id != blocker.id]
        options = []
        for side in (+1, -1):
            d_pass = self._side_ok(side, blocker, others, ego_s, bounds, frame)
            if d_pass is not None:
                options.append((abs(d_pass), side, d_pass))  # least deviation from the line wins
        if not options:
            self.state = FOLLOW
            return 0.0
        _, self.side, d_pass = min(options)
        self.state, self.target_id, self.d_pass, self._last_tgt = PASS, blocker.id, d_pass, blocker
        return d_pass
