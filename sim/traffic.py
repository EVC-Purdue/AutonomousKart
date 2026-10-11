"""Scripted opponent karts for closed-loop sims.

Opponents drive along the racing line in Frenet coordinates with scripted
speed and lateral profiles. Each tick the ego planner sees them either
perfectly (`perception="truth"`) or through the real perception back end
(`perception="tracker"`): forward-camera FOV and range limits, range-scaled
position noise, and the actual KartTracker from opencv_pathfinder. Collisions
use oriented kart rectangles.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Union

import numpy as np

from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import KartTracker, TrackFrame
from autonomous_kart.nodes.pathfinder.opponents import Opponent

Profile = Union[float, Callable[[float], float]]


def _eval(p: Profile, t: float) -> float:
    return float(p(t)) if callable(p) else float(p)


@dataclass
class ScriptedKart:
    id: int
    s0: float  # start arc length (m)
    speed: Profile  # target speed (m/s), constant or f(t)
    d: Profile = 0.0  # lateral offset (m), constant or f(t)
    accel_max: float = 4.0
    brake_max: float = 6.0
    s: float = field(init=False)
    v: float = field(init=False)
    travelled: float = field(init=False)  # cumulative, for counting passes

    def __post_init__(self):
        self.s = self.s0
        self.v = _eval(self.speed, 0.0)
        self.travelled = 0.0


def _rect_gap(ax, ay, ah, bx, by, bh, half_l, half_w) -> float:
    """Separating-axis gap between two equal oriented rectangles.
    > 0: separated by at least that much; < 0: overlapping."""
    d = np.array([bx - ax, by - ay])
    axes = [(math.cos(ah), math.sin(ah)), (-math.sin(ah), math.cos(ah)),
            (math.cos(bh), math.sin(bh)), (-math.sin(bh), math.cos(bh))]
    ea = [(half_l, axes[0]), (half_w, axes[1])]
    eb = [(half_l, axes[2]), (half_w, axes[3])]
    gap = -math.inf
    for u in axes:
        ra = sum(e * abs(u[0] * v[0] + u[1] * v[1]) for e, v in ea)
        rb = sum(e * abs(u[0] * v[0] + u[1] * v[1]) for e, v in eb)
        gap = max(gap, abs(d[0] * u[0] + d[1] * u[1]) - ra - rb)
    return gap


class Traffic:
    def __init__(self, line: list, karts: Sequence[ScriptedKart], perception: str = "truth",
                 kart_length: float = 1.8, kart_width: float = 1.0,
                 fov_deg: float = 90.0, max_range_m: float = 40.0,
                 range_noise_frac: float = 0.05, range_noise_floor_m: float = 0.2,
                 latency_s: float = 0.1, camera_hz: float = 30.0, rng_seed: int = 0):
        self.frame = TrackFrame(line)
        self.karts = list(karts)
        self.perception = perception
        self.half_l, self.half_w = 0.5 * kart_length, 0.5 * kart_width
        self.fov = math.radians(fov_deg)
        self.max_range = max_range_m
        self.rn_frac, self.rn_floor = range_noise_frac, range_noise_floor_m
        self.latency = latency_s
        self.cam_dt = 1.0 / camera_hz
        self._rng = np.random.default_rng(rng_seed)
        self._tracker = KartTracker(self.frame)
        self._next_cam_t = 0.0
        self._delayed: List[tuple] = []  # (deliver_t, Opponents)
        self._latest: tuple = ()
        # Metrics
        self.min_gap = math.inf
        self.collision_steps = 0
        self.collided_ids: set = set()

    # Truth

    def step(self, t: float, dt: float) -> None:
        for k in self.karts:
            vt = _eval(k.speed, t)
            dv = np.clip(vt - k.v, -k.brake_max * dt, k.accel_max * dt)
            k.v = max(0.0, k.v + dv)
            k.s = (k.s + k.v * dt) % self.frame.s_total
            k.travelled += k.v * dt

    def _world(self, k: ScriptedKart, t: float):
        d = _eval(k.d, t)
        x, y, psi, _ = self.frame.pose_at(k.s)
        return float(x - d * math.sin(psi)), float(y + d * math.cos(psi)), float(psi), d

    def truth(self, t: float) -> tuple:
        return tuple(Opponent(k.id, k.s, _eval(k.d, t), k.v, 0.0, t) for k in self.karts)

    # What the planner sees

    def observe(self, t: float, ego_x: float, ego_y: float, ego_yaw: float, ego_s: float,
                ego_v: float) -> tuple:
        if self.perception == "truth":
            return self.truth(t)
        if t + 1e-9 >= self._next_cam_t:
            self._next_cam_t = t + self.cam_dt
            meas, covs = [], []
            for k in self.karts:
                x, y, _, _ = self._world(k, t)
                dx, dy = x - ego_x, y - ego_y
                rng = math.hypot(dx, dy)
                bearing = math.atan2(dy, dx) - ego_yaw
                bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
                if rng > self.max_range or abs(bearing) > 0.5 * self.fov:
                    continue
                sr = self.rn_floor + self.rn_frac * rng  # range sigma
                sb = 0.01  # bearing sigma (rad)
                r_m = rng + self._rng.normal(0.0, sr)
                b_m = bearing + self._rng.normal(0.0, sb) + ego_yaw
                meas.append([ego_x + r_m * math.cos(b_m), ego_y + r_m * math.sin(b_m)])
                c, s_ = math.cos(b_m), math.sin(b_m)
                J = np.array([[c, -r_m * s_], [s_, r_m * c]])
                covs.append(J @ np.diag([sr * sr, sb * sb]) @ J.T)
            z = np.array(meas).reshape(-1, 2)
            R = np.array(covs).reshape(-1, 2, 2)

            def in_fov(x, y):
                b = math.atan2(y - ego_y, x - ego_x) - ego_yaw
                b = (b + math.pi) % (2 * math.pi) - math.pi
                return abs(b) <= 0.5 * self.fov and math.hypot(x - ego_x, y - ego_y) <= self.max_range

            tracks = self._tracker.step(t, z, R, ego_s=ego_s, ego_s_dot=ego_v, in_fov=in_fov)
            opps = tuple(Opponent(tr.id, tr.s, tr.d, tr.s_dot, tr.d_dot, tr.t,
                                  math.sqrt(tr.cov[0, 0]), math.sqrt(tr.cov[1, 1])) for tr in tracks)
            self._delayed.append((t + self.latency, opps))
        while self._delayed and self._delayed[0][0] <= t + 1e-9:
            self._latest = self._delayed.pop(0)[1]
        return self._latest

    def check(self, t: float, ego_x: float, ego_y: float, ego_yaw: float) -> None:
        hit = False
        for k in self.karts:
            x, y, psi, _ = self._world(k, t)
            g = _rect_gap(ego_x, ego_y, ego_yaw, x, y, psi, self.half_l, self.half_w)
            self.min_gap = min(self.min_gap, g)
            if g < 0.0:
                hit = True
                self.collided_ids.add(k.id)
        self.collision_steps += int(hit)

    def stats(self, ego_progress_m: float) -> dict:
        """ego_progress_m: ego's cumulative arc length from s = 0."""
        passed = [k.id for k in self.karts
                  if ego_progress_m > k.s0 + k.travelled + 2.0 * self.half_l]
        return {
            "passed_ids": passed,
            "min_gap_m": float(self.min_gap),
            "collision_steps": int(self.collision_steps),
            "collided_ids": sorted(self.collided_ids),
        }
