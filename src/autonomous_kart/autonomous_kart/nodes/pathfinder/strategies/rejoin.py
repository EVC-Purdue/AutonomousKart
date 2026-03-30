"""
Rejoin strategy: generates a cubic Bezier path from current pose
back onto the racing line when cross-track error (cte) is too large.
"""

import math
from typing import List, Optional, Tuple

from autonomous_kart.nodes.pathfinder.dynamic_line import (
    LineStrategy,
    KartState,
    WaypointRow,
)


def _heading_at_index(line: list, idx: int) -> float:
    n = len(line)
    if n < 2:
        return 0.0
    i0 = max(0, idx - 1)
    i1 = min(n - 1, idx + 1)
    dx = float(line[i1][1]) - float(line[i0][1])
    dy = float(line[i1][2]) - float(line[i0][2])
    return math.atan2(dy, dx)


def _cubic_bezier(p0, p1, p2, p3, t: float):
    u = 1.0 - t
    x = u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0]
    y = u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1]
    return (x, y)


def _bezier_curvature(p0, p1, p2, p3, t: float) -> float:
    u = 1.0 - t
    dx = 3 * (u**2 * (p1[0] - p0[0]) + 2 * u * t * (p2[0] - p1[0]) + t**2 * (p3[0] - p2[0]))
    dy = 3 * (u**2 * (p1[1] - p0[1]) + 2 * u * t * (p2[1] - p1[1]) + t**2 * (p3[1] - p2[1]))
    ddx = 6 * (u * (p2[0] - 2 * p1[0] + p0[0]) + t * (p3[0] - 2 * p2[0] + p1[0]))
    ddy = 6 * (u * (p2[1] - 2 * p1[1] + p0[1]) + t * (p3[1] - 2 * p2[1] + p1[1]))
    denom = (dx**2 + dy**2) ** 1.5
    if denom < 1e-9:
        return 0.0
    return (dx * ddy - dy * ddx) / denom


def _find_merge_idx(racing_line: list, closest_idx: int, lookahead_m: float) -> int:
    n = len(racing_line)
    s_closest = float(racing_line[closest_idx][0])
    s_target = s_closest + lookahead_m
    s_end = float(racing_line[-1][0])
    closed = s_end > 0.0 and (
        math.hypot(
            float(racing_line[0][1]) - float(racing_line[-1][1]),
            float(racing_line[0][2]) - float(racing_line[-1][2]),
        )
        < 2.0
    )
    if closed and s_target > s_end:
        s_target -= s_end
        j = 0
        while j < n and float(racing_line[j][0]) < s_target:
            j += 1
        return min(j, n - 1)
    else:
        j = closest_idx
        while j < n and float(racing_line[j][0]) < s_target:
            j += 1
        return min(j, n - 1)


class RejoinStrategy(LineStrategy):
    """Cubic Bezier rejoin when CTE exceeds threshold."""

    priority = 50  # Medium priority

    def __init__(
        self,
        cte_activate: float = 3.0,
        cte_deactivate: float = 1.0,
        merge_lookahead_m: float = 15.0,
        min_turning_radius: float = 3.0,
        num_samples: int = 50,
        rejoin_speed_pct: float = 0.4,
    ):
        self.cte_activate = cte_activate
        self.cte_deactivate = cte_deactivate
        self.merge_lookahead_m = merge_lookahead_m
        self.min_turning_radius = min_turning_radius
        self.num_samples = num_samples
        self.rejoin_speed_pct = rejoin_speed_pct

    def should_activate(self, state: KartState, racing_line: list) -> bool:
        return state.cross_track_error > self.cte_activate

    def should_deactivate(self, state: KartState, racing_line: list) -> bool:
        return state.cross_track_error < self.cte_deactivate

    def generate(
        self, state: KartState, racing_line: list
    ) -> Optional[Tuple[List[WaypointRow], int]]:
        n = len(racing_line)
        if n < 2:
            return None

        lookahead = self.merge_lookahead_m
        merge_idx = _find_merge_idx(racing_line, state.closest_idx, lookahead)
        merge_xy = (float(racing_line[merge_idx][1]), float(racing_line[merge_idx][2]))
        merge_yaw = _heading_at_index(racing_line, merge_idx)

        dist = math.hypot(merge_xy[0] - state.xy[0], merge_xy[1] - state.xy[1])
        if dist < 0.5:
            return None

        cp_dist = max(dist / 3.0, self.min_turning_radius * 0.5)

        p0 = state.xy
        p1 = (p0[0] + cp_dist * math.cos(state.yaw), p0[1] + cp_dist * math.sin(state.yaw))
        p3 = merge_xy
        p2 = (p3[0] - cp_dist * math.cos(merge_yaw), p3[1] - cp_dist * math.sin(merge_yaw))

        # Curvature check — push merge further if too tight
        max_kappa = 1.0 / self.min_turning_radius
        needs_retry = any(
            abs(_bezier_curvature(p0, p1, p2, p3, i / self.num_samples)) > max_kappa
            for i in range(self.num_samples + 1)
        )
        if needs_retry:
            lookahead *= 1.5
            merge_idx = _find_merge_idx(racing_line, state.closest_idx, lookahead)
            merge_xy = (float(racing_line[merge_idx][1]), float(racing_line[merge_idx][2]))
            merge_yaw = _heading_at_index(racing_line, merge_idx)
            dist = math.hypot(merge_xy[0] - state.xy[0], merge_xy[1] - state.xy[1])
            cp_dist = max(dist / 3.0, self.min_turning_radius * 0.5)
            p1 = (p0[0] + cp_dist * math.cos(state.yaw), p0[1] + cp_dist * math.sin(state.yaw))
            p3 = merge_xy
            p2 = (p3[0] - cp_dist * math.cos(merge_yaw), p3[1] - cp_dist * math.sin(merge_yaw))

        # Sample Bezier into waypoints
        v_max_col = float(racing_line[merge_idx][5]) if len(racing_line[merge_idx]) > 5 else 10.0
        points: List[WaypointRow] = []
        s_accum = 0.0
        prev = _cubic_bezier(p0, p1, p2, p3, 0.0)

        for i in range(self.num_samples + 1):
            t = i / self.num_samples
            pt = _cubic_bezier(p0, p1, p2, p3, t)
            if i > 0:
                s_accum += math.hypot(pt[0] - prev[0], pt[1] - prev[1])
            prev = pt
            speed_ramp = self.rejoin_speed_pct + (1.0 - self.rejoin_speed_pct) * (t ** 2)
            vx = speed_ramp * v_max_col
            points.append((s_accum, pt[0], pt[1], 0.0, 0.0, vx))

        return points, merge_idx