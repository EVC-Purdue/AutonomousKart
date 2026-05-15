import math
from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.dynamic_line import DynamicLineManager, KartState
from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder
from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs
from autonomous_kart.nodes.pathfinder.strategies.rejoin import RejoinStrategy


class PurePursuitPlanner(Planner):
    """Nav2-style Regulated Pure Pursuit over the racing line, with a Bezier
    rejoin overlay when cross-track error is high. Uses odom + racing line;
    track_angles is available via PlannerInputs but not consumed here."""

    name = "pure_pursuit"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)

        self.use_velocity_scaled_lookahead = bool(params.get("use_velocity_scaled_lookahead", True))
        self.lookahead_time_s = float(params.get("lookahead_time_s", 1.0))
        self.min_lookahead_m = float(params.get("min_lookahead_m", 0.6))
        self.max_lookahead_m = float(params.get("max_lookahead_m", 3.0))

        self.use_curvature_regulation = bool(params.get("use_curvature_regulation", True))
        self.min_radius_m = float(params.get("min_radius_m", 2.0))
        self.min_reg_speed_mps = float(params.get("min_reg_speed_mps", 4.0))

        self.approach_dist_m = float(params.get("approach_dist_m", 1.0))
        self.min_approach_speed_mps = float(params.get("min_approach_speed_mps", 1.0))

        self.search_window = int(params.get("search_window", 80))
        self.max_resync_dist = float(params.get("max_resync_dist", 80.0))
        self.max_closed_dist = float(params.get("max_closed_dist", 2.0))

        self.initial_sync_done = bool(params.get("initial_sync_done", False))
        self.closest_idx = 0

        self.line_manager = DynamicLineManager(racing_line, logger=logger)
        self.line_manager.register(RejoinStrategy(
            cte_activate=float(params.get("rejoin_cte_activate", 3.0)),
            cte_deactivate=float(params.get("rejoin_cte_deactivate", 1.0)),
            merge_lookahead_m=float(params.get("rejoin_merge_lookahead_m", 15.0)),
            min_turning_radius=self.min_radius_m,
        ))

    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        racing_line = self.racing_line
        if not racing_line:
            return None

        current_xy = inputs.pose_xy

        if self.use_velocity_scaled_lookahead:
            lookahead_m = inputs.speed_mps * self.lookahead_time_s
            if lookahead_m < self.min_lookahead_m:
                lookahead_m = self.min_lookahead_m
            elif lookahead_m > self.max_lookahead_m:
                lookahead_m = self.max_lookahead_m
        else:
            lookahead_m = self.min_lookahead_m

        if not self.initial_sync_done:
            self.closest_idx = self._full_nearest_idx(racing_line, current_xy)
            self.initial_sync_done = True
        else:
            if self.closest_idx < 0 or self.closest_idx >= len(racing_line):
                self.closest_idx = 0
            row = racing_line[self.closest_idx]
            dx = current_xy[0] - float(row[1])
            dy = current_xy[1] - float(row[2])
            if math.hypot(dx, dy) > self.max_resync_dist:
                self.closest_idx = self._full_nearest_idx(racing_line, current_xy)
            else:
                self.closest_idx = self._nearest_idx_forward(
                    racing_line, current_xy, self.closest_idx,
                    window=self.search_window, allow_wrap=True,
                )

        cte = self.line_manager.compute_cte(current_xy, racing_line, self.closest_idx)
        kart_state = KartState(
            xy=current_xy,
            yaw=inputs.yaw_rad,
            speed_mps=inputs.speed_mps,
            closest_idx=self.closest_idx,
            cross_track_error=cte,
        )
        self.line_manager.update(kart_state)
        self.closest_idx = kart_state.closest_idx

        if self.line_manager.is_active:
            dyn_line, dyn_idx = self.line_manager.get_line_and_idx()
            if not dyn_line or dyn_idx < 0:
                target_xy, speed_ref_mps = self._pick_lookahead_point(
                    racing_line, self.closest_idx, lookahead_m, current_xy
                )
            else:
                dyn_idx = self._nearest_idx_forward(
                    dyn_line, current_xy, dyn_idx,
                    window=self.search_window, allow_wrap=False,
                )
                target_xy, speed_ref_mps = self._pick_lookahead_point(
                    dyn_line, dyn_idx, lookahead_m, current_xy
                )
                self.line_manager.set_dynamic_closest_idx(dyn_idx)
        else:
            target_xy, speed_ref_mps = self._pick_lookahead_point(
                racing_line, self.closest_idx, lookahead_m, current_xy
            )

        motor_mps, steering_deg = pathfinder(
            current_xy=current_xy,
            target_xy=target_xy,
            yaw_rad=inputs.yaw_rad,
            speed_mps=inputs.speed_mps,
            wheelbase_m=self.kart.wheelbase_m,
            v_max_mps=self.kart.v_max_mps,
            steer_max_deg=self.kart.steer_max_deg,
            desired_speed_mps=speed_ref_mps,
            use_velocity_scaled_lookahead=self.use_velocity_scaled_lookahead,
            lookahead_time_s=self.lookahead_time_s,
            min_lookahead_m=self.min_lookahead_m,
            max_lookahead_m=self.max_lookahead_m,
            use_curvature_regulation=self.use_curvature_regulation,
            min_radius_m=self.min_radius_m,
            min_reg_speed_mps=self.min_reg_speed_mps,
            approach_dist_m=self.approach_dist_m,
            min_approach_speed_mps=self.min_approach_speed_mps,
        )
        return motor_mps, steering_deg

    def dynamic_line_state(self) -> Optional[dict]:
        if self.line_manager.is_active:
            line = self.line_manager.dynamic_line
            return {
                "active": True,
                "strategy": type(self.line_manager.active_strategy).__name__,
                "merge_idx": int(self.line_manager.merge_idx),
                "points": [[float(p[1]), float(p[2])] for p in line],
            }
        return {"active": False, "strategy": None, "merge_idx": -1, "points": []}

    @staticmethod
    def _clamp01(x: float) -> float:
        if x < 0.0:
            return 0.0
        if x > 1.0:
            return 1.0
        return x

    @staticmethod
    def _nearest_idx_forward(
        line: list,
        xy: Tuple[float, float],
        start_idx: int,
        window: int,
        allow_wrap: bool,
    ) -> int:
        n = len(line)
        if n == 0:
            return 0
        if start_idx < 0 or start_idx >= n:
            start_idx = 0
        if allow_wrap:
            max_offset = min(window, n)
        else:
            max_offset = min(window, n - start_idx)
        best_i = start_idx
        best_d2 = float("inf")
        cx, cy = xy
        for offset in range(max_offset):
            if allow_wrap:
                i = (start_idx + offset) % n
            else:
                i = start_idx + offset
            dx = line[i][1] - cx
            dy = line[i][2] - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    @staticmethod
    def _full_nearest_idx(line: list, xy: Tuple[float, float]) -> int:
        n = len(line)
        if n == 0:
            return 0
        cx, cy = xy
        best_i = 0
        best_d2 = float("inf")
        for i in range(n):
            dx = line[i][1] - cx
            dy = line[i][2] - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _pick_lookahead_point(
        self,
        line: list,
        closest_idx: int,
        lookahead_m: float,
        current_xy: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], float]:
        n = len(line)
        if n == 0:
            return current_xy, 0.0

        if lookahead_m <= 0.0:
            lookahead_m = 0.01

        if closest_idx < 0 or closest_idx >= n:
            closest_idx = 0

        s0 = float(line[closest_idx][0])
        s_target = s0 + float(lookahead_m)

        s_end = float(line[-1][0])
        closed = s_end > 0.0 and (
            math.hypot(line[0][1] - line[-1][1], line[0][2] - line[-1][2]) <= self.max_closed_dist
        )

        if closed and s_target > s_end:
            s_target -= s_end
            j = 0
            while j < n and float(line[j][0]) < s_target:
                j += 1
            if j >= n:
                j = n - 1
        else:
            j = closest_idx
            while j < n and float(line[j][0]) < s_target:
                j += 1
            if j >= n:
                j = n - 1

        tx = float(line[j][1])
        ty = float(line[j][2])
        vx_mps = float(line[j][5]) if len(line[j]) > 5 else 0.0

        return (tx, ty), vx_mps
