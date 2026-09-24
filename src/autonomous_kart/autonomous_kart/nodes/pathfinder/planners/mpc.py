"""
NORMAL  : track racing line inside the corridor
FAILSAFE: stale state / no feasible plan

Solves every pathfinder tick (60 Hz) by rolling out K random control
sequences with the kinematic bicycle model and picking the lowest-cost
sequence whose trajectory stays inside the corridor. When the kart
strays outside the corridor, the shared DynamicLineManager swaps in a
cubic-Bezier overlay (RejoinStrategy) as the reference line so the
solver can plan back onto the racing line without a separate mode.
"""

import math
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np
from std_msgs.msg import Float32MultiArray

from autonomous_kart.nodes.pathfinder.dynamic_line import (
    DynamicLineManager, KartState,
)
from autonomous_kart.nodes.pathfinder.planners.base import (
    KartConstants, Planner, PlannerInputs,
)
from autonomous_kart.nodes.pathfinder.planners.mpc_residual import (
    NUM_STEER_HIST, NUM_THROTTLE_HIST, ResidualLearner, _features,
)
from autonomous_kart.nodes.pathfinder.strategies.rejoin import RejoinStrategy

MODE_NORMAL, MODE_FAILSAFE = 0, 2


def _wrap(angle):
    """Wrap to (-pi, pi] works on scalars or numpy arrays."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MPCPlanner(Planner):
    name = "mpc"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list,
                 logger=None, node=None, residual: Optional[ResidualLearner] = None):
        super().__init__(params, kart, racing_line, logger, node=node)

        if not racing_line:
            raise ValueError("MPCPlanner requires a non-empty racing line")

        g = params.get  # canonical yaml -> code; defaults below are fallbacks only

        # Horizon / sampling
        self.N = int(g("horizon_steps", 20))
        self.dt = float(g("dt_s", 0.05))
        self.K = int(g("num_samples", 48))
        self.steer_sigma = math.radians(float(g("steer_sigma_deg", 6.0)))
        self.accel_sigma = float(g("accel_sigma_mps2", 1.0))
        self.proj_back = int(g("proj_window_back", 5))
        self.proj_fwd = int(g("proj_window_fwd", 60))
        self.v_window_s = float(g("frenet_v_window_s", 0.3))
        # Threshold (m^2) above which the windowed Frenet match is considered
        # unreliable and _frenet falls back to a full-line nearest search.
        self._resync_dist2 = float(g("frenet_resync_m", 5.0)) ** 2

        # Physical limits pulled from kart constants (kart-wide /**: block
        # no duplicates lurking in the mpc block).
        self.steer_max = math.radians(kart.steer_max_deg)
        self.steer_rate_max = math.radians(kart.steer_rate_max_degps)
        self.v_max = kart.v_max_mps
        self.wheelbase = kart.wheelbase_m
        self.a_min = kart.a_min_mps2
        self.a_max = kart.a_max_mps2
        self.a_lat_max = kart.a_lat_max_mps2
        self.target_speed = float(g("target_speed_mps", 6.0))
        # Below this speed, dpsi = v/L*tan(delta) ~= 0 so the cost can't
        # distinguish steering choices — hold delta at 0 until v crosses the gate.
        self.steer_observability_v = float(g("steer_observability_v_mps", 0.5))
        # Actuator gain: real-kart wheel angle ≈ actuator_gain × cmd_drive_steer
        # (fit from PP bags 20260519_15:06/21:46/22:05: slope ≈ 0.10 deg/deg).
        # Applied inside tan(δ) in the bicycle rollout so MPC's predicted yaw
        # rate matches the kart instead of overestimating by ~10×.
        self.actuator_gain = float(g("actuator_gain", 0.10))
        # Optional knot table replacing that constant with the measured curve:
        # the linkage has a dead band either side of centre and a slope that
        # reaches 0.55 past 20 deg, so no constant fits both ends. Fit by
        # scripts/mpc_debug/steer_map.py; empty falls back to actuator_gain.
        #
        # With a table the solver decides in WHEEL angle and the map answers
        # what to ask for, rather than deciding in command and the map saying
        # what that would do. Both give the same rollout, but only the first
        # gives the sampler a cost it can descend: the dead band makes the
        # cost flat across the +-1.7 deg the rate limit leaves reachable in a
        # tick, so a command-space search crawls out of centre at a few tenths
        # of a degree per tick and reaches the corner half a second late.
        self._map_cmd = np.radians(g("steer_map_cmd_deg", []) or [])
        self._map_wheel = np.radians(g("steer_map_wheel_deg", []) or [])
        if not bool(g("use_steer_map", True)):
            self._map_cmd = self._map_wheel = np.empty(0)
        # The rack cannot reach the 60 deg the kart constants allow: the bags
        # top out near 13 deg of actual wheel angle. Cap the command at the
        # equivalent of `wheel_max_deg` so neither the rollout nor a_lat can
        # believe in yaw the front axle cannot produce.
        self.wheel_max_deg = float(g("wheel_max_deg", 20.0))
        self.steer_max = min(self.steer_max,
                             math.radians(self.wheel_max_deg)
                             / max(self.actuator_gain, 1e-6))
        # First-order steer lag: actual wheel angle exponentially approaches
        # commanded with time constant steer_tau_s. Step-response analysis
        # (docs/superpowers/data_sim_heavy) puts the heavy-kart yaw rise time
        # at ~300 ms (τ≈0.15 s) vs ~50 ms (τ≈0.03 s) on the light kart. With
        # τ=0 the rollout is instantaneous (legacy behavior).
        self.steer_tau_s = float(g("steer_tau_s", 0.0))
        # Elite-mean fraction: instead of argmin over K samples, average the
        # top `mppi_elite_frac × K` samples' first actions. Cuts per-tick noise
        # without the phase-lag cost of a 1st-order filter. 1.0/K = pure argmin (no smoothing).
        self.mppi_elite_frac = float(g("mppi_elite_frac", 0.1))
        # Correlation time of the steering perturbation. At 0 the noise is
        # independent every horizon step, which is what ships: forty independent
        # kicks average back to the warm start, so the sampled sequences form a
        # band the same width at step 39 as at step 4 and an optimum outside it
        # is not in the draw at any horizon length. Above 0 each step keeps most
        # of the last one, so a sample can drift somewhere and hold, and a ramp
        # becomes something the proposal can express.
        self.steer_noise_tau = float(g("steer_noise_tau_s", 0.0))

        # Corridor (centerline = racing line)
        tw = float(g("track_half_width_m", 2.5))
        sm = float(g("safety_margin_m", 0.5))
        kw = float(g("kart_half_width_m", 0.5025))
        self.corridor_half = tw - sm - kw

        # Costs
        self.w_d = float(g("w_d", 100.0))
        self.w_heading = float(g("w_heading", 30.0))
        self.w_speed = float(g("w_speed", 5.0))
        self.w_delta = float(g("w_delta", 5.0))
        self.w_drate = float(g("w_delta_rate", 50.0))
        self.w_accel = float(g("w_accel", 2.0))
        self.w_boundary = float(g("w_boundary", 1000.0))
        self.w_progress = float(g("w_progress", 1.0))
        self.w_term_d = float(g("w_terminal_d", 200.0))
        self.w_term_h = float(g("w_terminal_heading", 100.0))
        self.w_a_lat = float(g("w_a_lat", 50.0))

        # Inner-corridor "stay centered" barrier: zero inside
        # edge_inner_frac * corridor_half of the line, quadratic outward.
        # Separate from w_d (flat quadratic everywhere) so the planner can be
        # gently centered without being whipped to the centerline in corners.
        self.w_edge = float(g("w_edge", 200.0))
        self.edge_inner_frac = float(g("edge_inner_frac", 0.5))
        self.edge_inner = self.corridor_half * self.edge_inner_frac

        # Failsafe / feasibility sentinels
        self.max_failures = int(g("max_consecutive_solver_failures", 3))
        self.infeasible_cost = float(g("infeasible_cost", 1.0e9))
        self.feasibility_threshold = float(g("feasibility_threshold", 5.0e8))

        # Rejoin: cubic-Bezier overlay generated by RejoinStrategy through the
        # shared DynamicLineManager. When CTE exceeds the activation gate the
        # manager swaps in a bezier from the kart's pose back onto the racing
        # line; the planner Frenet-projects against that bezier (treating it
        # as the reference line) until it deactivates near the merge point.
        self._rejoin_kwargs = dict(
            cte_activate=float(g("rejoin_cte_activate", 2.5)),
            cte_deactivate=float(g("rejoin_cte_deactivate", 1.0)),
            merge_lookahead_m=float(g("rejoin_merge_lookahead_m", 15.0)),
            min_turning_radius=float(g("rejoin_min_turning_radius", 3.0)),
        )
        self.line_manager = DynamicLineManager(racing_line, logger=logger)
        self.line_manager.register(RejoinStrategy(**self._rejoin_kwargs))

        # Racing line as flat numpy arrays with derived tangent for Frenet.
        # The static racing line is built once; `self.l_*` references the
        # currently active reference (static line or bezier overlay) and is
        # swapped each tick by _set_active_line.
        self._static_arrays = self._build_line_arrays(racing_line)
        self._bezier_arrays = None
        self._set_active_line(self._static_arrays)

        # State carried between ticks
        self.closest_idx = 0  # idx into the ACTIVE reference (line or bezier)
        self.static_closest_idx = 0  # idx into the static racing line (for CTE gate)
        # First plan() does an O(n) nearest search so a kart spawned mid-track
        # isn't pinned near idx 0 by _frenet's wrap-aware resync gate.
        self.initial_sync_done = False
        self.delta_prev = 0.0
        self.a_prev = 0.0
        self.u_mean = np.zeros((2, self.N))  # warm-start [delta(N), accel(N)]
        self.consec_failures = 0
        self.mode = MODE_NORMAL

        # Residual learner (delay sized by 1/system_frequency)
        solve_dt = 1.0 / float(node.get_parameter("system_frequency").value) \
            if node is not None else self.dt
        residual_params = {
            k[len("residual."):]: v for k, v in params.items()
            if k.startswith("residual.")
        }
        # Shared ResidualLearner survives planner / line swaps so training state persists
        s_total = float(self._static_arrays["s"][-1])
        self.residual = residual if residual is not None else ResidualLearner(
            residual_params, solve_dt, s_total=s_total,
        )
        self._nom_steps = max(1, int(round(self.residual.target_horizon_s / self.dt)))
        self._last_motor_mps = 0.0
        self._train_dt = solve_dt
        self._steer_hist = deque([0.0] * NUM_STEER_HIST, maxlen=NUM_STEER_HIST)
        self._throttle_hist = deque([0.0] * NUM_THROTTLE_HIST, maxlen=NUM_THROTTLE_HIST)
        # Pose history covers v_window_s of past samples at the solve cadence.
        pose_hist_len = max(2, int(round(self.v_window_s / solve_dt)) + 2)
        self._pose_hist = deque(maxlen=pose_hist_len)  # (now_ns, s, d)

        # Telemetry publisher
        self.status_pub = None
        if node is not None:
            self.status_pub = node.create_publisher(Float32MultiArray, "mpc/status", 5)

        # Live actuator_gain estimator (telemetry only — NOT fed back into
        # self.actuator_gain). Each plan() tick, if v and |δ| are above the
        # filter thresholds, we sample α = atan(yaw_rate · L / v) / δ_cmd_rad
        # and push to a rolling deque. Median of the deque is the headline
        # estimate published on mpc/actuator_gain_status as
        # [current_setting, estimated_median, n_samples].
        self._alpha_buf = deque(maxlen=300)
        self._last_yaw_for_alpha = None
        self._last_t_for_alpha = None
        self._alpha_pub = None
        if node is not None:
            self._alpha_pub = node.create_publisher(
                Float32MultiArray, "mpc/actuator_gain_status", 5
            )

        # Reusable rng
        self._rng = np.random.default_rng(0)

        # Latest raw GPS pose snapshot — appended to status payload so the
        # mpc/status log alone is enough to diff EKF vs raw GPS offline.
        self._gps_x = math.nan
        self._gps_y = math.nan
        self._gps_yaw = math.nan
        self._gps_v = math.nan
        # Wheel speed (m/s) + last-GPS-event EKF prior/posterior snapshot.
        # Drives the offline EKF-innovation analysis: prior is the IMU+wheel
        # dead-reckoned state right before GPS fused in; post is after.
        self._wheel_v = math.nan
        self._gps_event_seq = 0.0
        self._gps_have_yaw = 0.0
        self._gps_have_speed = 0.0
        self._ekf_prior_x = math.nan
        self._ekf_prior_y = math.nan
        self._ekf_prior_yaw = math.nan
        self._ekf_prior_v = math.nan
        self._ekf_post_x = math.nan
        self._ekf_post_y = math.nan
        self._ekf_post_yaw = math.nan
        self._ekf_post_v = math.nan

    # plan
    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        t0 = time.perf_counter()
        x, y = inputs.pose_xy
        yaw = inputs.yaw_rad
        v = max(0.0, inputs.speed_mps)
        self._gps_x, self._gps_y = inputs.gps_xy
        self._gps_yaw = inputs.gps_yaw_rad
        self._gps_v = inputs.gps_speed_mps
        self._wheel_v = inputs.wheel_speed_mps
        self._gps_event_seq = inputs.gps_event_seq
        self._gps_have_yaw = inputs.gps_have_yaw
        self._gps_have_speed = inputs.gps_have_speed
        self._ekf_prior_x, self._ekf_prior_y = inputs.ekf_prior_xy
        self._ekf_prior_yaw = inputs.ekf_prior_yaw_rad
        self._ekf_prior_v = inputs.ekf_prior_v
        self._ekf_post_x, self._ekf_post_y = inputs.ekf_post_xy
        self._ekf_post_yaw = inputs.ekf_post_yaw_rad
        self._ekf_post_v = inputs.ekf_post_v

        # Frenet against the STATIC racing line so the rejoin gate uses
        #    CTE-to-line, not CTE-to-overlay.
        self._set_active_line(self._static_arrays)
        if not self.initial_sync_done:
            d2 = (self.l_x - x) ** 2 + (self.l_y - y) ** 2
            self.static_closest_idx = int(np.argmin(d2))
            self.closest_idx = self.static_closest_idx
            self.initial_sync_done = True
        _, d_static, j_static, _, _ = self._frenet(x, y, self.static_closest_idx)
        self.static_closest_idx = j_static

        # Drive overlay activation / deactivation off CTE-to-line.
        was_active = self.line_manager.is_active
        kart_state = KartState(
            xy=(x, y), yaw=yaw, speed_mps=v,
            closest_idx=self.static_closest_idx,
            cross_track_error=abs(d_static),
        )
        self.line_manager.update(kart_state)
        # Manager may snap closest_idx to merge_idx on deactivation.
        self.static_closest_idx = kart_state.closest_idx
        now_active = self.line_manager.is_active

        # On state transitions: rebuild bezier arrays / reset warm-start.
        if now_active and not was_active:
            self._bezier_arrays = self._build_line_arrays(
                self.line_manager.dynamic_line
            )
            self.closest_idx = 0
            self._reset_warm_start()
        elif was_active and not now_active:
            # Resume Frenet against the static line at the kart's actual nearest idx (already computed this tick)
            self.closest_idx = int(self.static_closest_idx)
            self._bezier_arrays = None
            self._reset_warm_start()

        # Point Frenet reference at the active line (bezier or static).
        if now_active and self._bezier_arrays is not None:
            self._set_active_line(self._bezier_arrays)
        else:
            self._set_active_line(self._static_arrays)

        s, d, j_now, psi_track, _ = self._frenet(x, y, self.closest_idx)
        self.closest_idx = j_now

        # Track pose history for v_s / v_d estimates
        self._pose_hist.append((inputs.now_ns, s, d))
        v_s, v_d = self._frenet_velocity()

        # Speed target: the bezier carries its own vx ramp, so the per-step
        # v_ref = min(j_vx, v_target) inside _solve picks that up automatically.
        v_target = self.target_speed
        v_cap = self.v_max

        # Build candidate trajectories, evaluate, pick best
        zeros_breakdown = (0.0,) * 12
        try:
            best_u, best_cost, traj, feasible, breakdown = self._solve(
                x, y, yaw, v, j_now, v_target, v_cap,
            )
        except Exception:
            # NaN propagation - skip
            best_u, best_cost, traj, feasible = None, float("inf"), None, False
            breakdown = zeros_breakdown

        if not feasible or best_u is None:
            self.consec_failures += 1
            if self.consec_failures >= self.max_failures:
                self.mode = MODE_FAILSAFE
            self._publish_status(self.mode, False, t0, s, d, psi_track, v, v_target,
                                 0.0, 0.0, best_cost, 0.0,
                                 breakdown=breakdown,
                                 x=x, y=y, yaw_rad=yaw,
                                 throttle_mps_out=0.0, v_s=v_s, v_d=v_d,
                                 closest_idx_active=self.closest_idx,
                                 closest_idx_static=self.static_closest_idx,
                                 rejoin_active=self.line_manager.is_active,
                                 merge_idx=int(self.line_manager.merge_idx),
                                 kappa_local=0.0,
                                 consec_failures=self.consec_failures,
                                 corridor_half=self.corridor_half)
            return 0.0, 0.0

        self.consec_failures = 0
        # `best_u[0]` is the plan in whichever space the solver decided in, and
        # `cmd_out` is always the command to publish; without a map they are the
        # same number.
        delta_cmd = self._cmd_out if self._map_cmd.size else float(best_u[0, 0])
        accel_cmd = float(best_u[1, 0])

        if v < self.steer_observability_v:
            delta_cmd = 0.0
            self.u_mean[0, :] = 0.0

        v_ref_now = min(float(self.l_vx[j_now]), self.target_speed)
        v_ref_now = max(0.0, min(self.v_max, v_ref_now))
        throttle_mps = v_ref_now
        steering_deg = math.degrees(delta_cmd)

        # Update warm-start (shift by 1)
        self.u_mean[:, :-1] = best_u[:, 1:]
        self.u_mean[:, -1] = best_u[:, -1]
        self.delta_prev = delta_cmd
        self.a_prev = accel_cmd
        self._steer_hist.appendleft(delta_cmd)
        self._throttle_hist.appendleft(accel_cmd)

        # Residual learner (shadow)
        # nominal (held-command - straight line no acceleration) prediction over target_horizon_s of motion.
        nom_x, nom_y = self._hold_rollout(x, y, yaw, v, delta_cmd, accel_cmd, self._nom_steps)
        nom_s, nom_d, _, _, _ = self._frenet(nom_x, nom_y, j_now)
        kappa_local = self._curvature_at(j_now)
        phi = _features(
            d, v_s, v_d, kappa_local,
            tuple(self._steer_hist), tuple(self._throttle_hist),
            nom_s - s, nom_d - d,
        )
        self.residual.push(phi, s, d, nom_s - s, nom_d - d, v)
        self.residual.step(s, d)
        res_ps, res_pd, _residual_source = self.residual.predict(phi)
        # effective_mode gating: zero the residual any time we're not yet in apply
        # so future consumers automatically see (0, 0) until the apply gate opens.
        if self.residual.effective_mode() != "apply":
            res_ps, res_pd = 0.0, 0.0

        # Live actuator_gain estimator (telemetry only — NOT consumed by _solve).
        self._update_alpha_estimate(yaw, v, delta_cmd, inputs.now_ns)

        # Telemetry
        margin_min = self.corridor_half - float(np.max(np.abs(traj["d"])))
        self._publish_status(self.mode, True, t0, s, d, psi_track, v, v_target,
                             delta_cmd, accel_cmd, best_cost, margin_min,
                             res_ps, res_pd,
                             breakdown=breakdown,
                             x=x, y=y, yaw_rad=yaw,
                             throttle_mps_out=throttle_mps, v_s=v_s, v_d=v_d,
                             closest_idx_active=j_now,
                             closest_idx_static=self.static_closest_idx,
                             rejoin_active=self.line_manager.is_active,
                             merge_idx=int(self.line_manager.merge_idx),
                             kappa_local=kappa_local,
                             consec_failures=self.consec_failures,
                             corridor_half=self.corridor_half)

        return throttle_mps, steering_deg

    # helpers
    def _frenet(self, x: float, y: float, hint: int):
        """Returns (s, d, idx, psi_track, kappa). d is signed (left positive).

        Windowed search around `hint`, with a full-line resync when the windowed match is unreliable"""
        lo = max(0, hint - self.proj_back)
        hi = min(self.line_n, hint + self.proj_fwd)
        dx = self.l_x[lo:hi] - x
        dy = self.l_y[lo:hi] - y
        d2 = dx * dx + dy * dy
        j_rel = int(np.argmin(d2))
        j = lo + j_rel
        best_d2 = float(d2[j_rel])

        at_lo_edge = (j == lo and lo > 0)
        at_hi_edge = (j == hi - 1 and hi < self.line_n)
        # On a closed loop, the natural "window edge" near the seam is the last index of the array
        near_seam = self.line_closed and (
                hint <= self.proj_back or hint >= self.line_n - self.proj_back
        )
        if at_lo_edge or at_hi_edge or near_seam or best_d2 > self._resync_dist2:
            all_dx = self.l_x - x
            all_dy = self.l_y - y
            all_d2 = all_dx * all_dx + all_dy * all_dy
            j_full = int(np.argmin(all_d2))
            # Wrap-aware index-distance gate: on a closed loop, reject jumps
            # to a geometrically-near-but-arc-far segment.
            n = self.line_n
            idx_dist = min((j_full - hint) % n, (hint - j_full) % n)
            if idx_dist < n // 4 and float(all_d2[j_full]) + 1e-6 < best_d2:
                j = j_full

        psi_j = self.l_psi[j]
        cpsi, spsi = math.cos(psi_j), math.sin(psi_j)
        ex, ey = x - self.l_x[j], y - self.l_y[j]
        along = ex * cpsi + ey * spsi
        lateral = -ex * spsi + ey * cpsi
        return self.l_s[j] + along, lateral, j, psi_j, 0.0

    def _curvature_at(self, j: int) -> float:
        if 0 < j < self.line_n - 1:
            dpsi = _wrap(self.l_psi[j + 1] - self.l_psi[j - 1])
            ds = max(1e-3, self.l_s[j + 1] - self.l_s[j - 1])
            return float(dpsi / ds)
        return 0.0

    @staticmethod
    def _build_line_arrays(line: list) -> dict:
        """Convert waypoint rows (s,x,y,psi,kappa,vx) into flat numpy arrays
        with a finite-difference psi tangent recomputed from xy."""
        L = np.asarray([list(r[:6]) for r in line], dtype=np.float64)
        n = L.shape[0]
        l_s = L[:, 0]
        l_x = L[:, 1]
        l_y = L[:, 2]
        l_vx = L[:, 5]
        dx = np.empty_like(l_x)
        dy = np.empty_like(l_y)
        dx[1:-1] = l_x[2:] - l_x[:-2]
        dy[1:-1] = l_y[2:] - l_y[:-2]
        dx[0] = l_x[1] - l_x[0]
        dy[0] = l_y[1] - l_y[0]
        dx[-1] = l_x[-1] - l_x[-2]
        dy[-1] = l_y[-1] - l_y[-2]
        l_psi = np.arctan2(dy, dx)
        # Closed if first and last waypoints sit on top of each other (within
        # 2m). Used by _frenet to force a full-line resync near the seam.
        closed = bool(
            n >= 4
            and math.hypot(float(l_x[0] - l_x[-1]), float(l_y[0] - l_y[-1])) < 2.0
        )
        return {
            "n": n, "s": l_s, "x": l_x, "y": l_y, "vx": l_vx, "psi": l_psi,
            "closed": closed,
        }

    def _set_active_line(self, arrays: dict) -> None:
        self.line_n = arrays["n"]
        self.l_s = arrays["s"]
        self.l_x = arrays["x"]
        self.l_y = arrays["y"]
        self.l_vx = arrays["vx"]
        self.l_psi = arrays["psi"]
        self.line_closed = bool(arrays.get("closed", False))

    def _reset_warm_start(self) -> None:
        self.u_mean[:] = 0.0
        self.delta_prev = 0.0
        self.a_prev = 0.0
        self._pose_hist.clear()

    def _frenet_velocity(self) -> Tuple[float, float]:
        """Finite-difference (s, d) over `v_window_s` of pose history."""
        if len(self._pose_hist) < 2:
            return 0.0, 0.0
        now_ns, s_now, d_now = self._pose_hist[-1]
        target = now_ns - int(self.v_window_s * 1e9)
        ref = self._pose_hist[0]
        for entry in self._pose_hist:
            if entry[0] >= target:
                ref = entry
                break
        dt = (now_ns - ref[0]) / 1e9
        if dt <= 0.0:
            return 0.0, 0.0
        return (s_now - ref[1]) / dt, (d_now - ref[2]) / dt

    def _update_alpha_estimate(self, yaw, v, delta_cmd, now_ns):
        """Estimate live actuator_gain from finite-diff yaw rate and δ_cmd.

        Telemetry-only — never written back into self.actuator_gain. Publishes
        [current_setting, median_estimate, n_samples] on mpc/actuator_gain_status.
        """
        now_s = now_ns / 1e9
        if (self._last_yaw_for_alpha is not None
                and self._last_t_for_alpha is not None):
            dt = now_s - self._last_t_for_alpha
            if 1e-3 < dt < 0.1:
                dyaw = math.atan2(
                    math.sin(yaw - self._last_yaw_for_alpha),
                    math.cos(yaw - self._last_yaw_for_alpha),
                )
                yaw_rate = dyaw / dt
                # Filter: only sample when the kart is moving and steering is
                # nontrivial, so finite-diff noise doesn't dominate.
                if v > 2.0 and abs(delta_cmd) > math.radians(5.0):
                    try:
                        alpha = math.atan(yaw_rate * self.wheelbase / v) / delta_cmd
                    except (ZeroDivisionError, ValueError):
                        alpha = float("nan")
                    if math.isfinite(alpha) and 0.0 < alpha < 5.0:
                        self._alpha_buf.append(alpha)
        self._last_yaw_for_alpha = yaw
        self._last_t_for_alpha = now_s

        if self._alpha_pub is not None:
            n = len(self._alpha_buf)
            median = float(np.median(self._alpha_buf)) if n else float("nan")
            self._alpha_pub.publish(Float32MultiArray(
                data=[float(self.actuator_gain), median, float(n)]
            ))

    def _wheel(self, delta):
        """Wheel angle (rad) the kart reaches for a commanded δ (rad)."""
        if self._map_cmd.size:
            return np.interp(delta, self._map_cmd, self._map_wheel)
        return self.actuator_gain * delta

    def _command(self, wheel):
        """Command (rad) to ask for a wheel angle (rad) — the map, backwards.

        Reading the same knots with the axes swapped. The table is monotone so
        this is single valued, and it is steep through the dead band, which is
        the point: asking for a degree of wheel angle near centre costs several
        degrees of command and the solver is told so up front.
        """
        if self._map_cmd.size:
            return np.interp(wheel, self._map_wheel, self._map_cmd)
        return wheel / max(self.actuator_gain, 1e-6)

    def _hold_rollout(self, x, y, yaw, v, delta, a, n):
        """Single-trajectory bicycle rollout used by the residual learner."""
        wheel = float(self._wheel(delta))
        for _ in range(n):
            x += self.dt * v * math.cos(yaw)
            y += self.dt * v * math.sin(yaw)
            yaw += self.dt * v / self.wheelbase * math.tan(wheel)
            v = max(0.0, min(self.v_max, v + self.dt * a))
        return x, y

    # solve
    def _solve(self, x0, y0, yaw0, v0, j_now, v_target, v_cap):
        K, N, dt = self.K, self.N, self.dt
        rng = self._rng

        # Sample around warm-start mean. Sample[0] is the warm-start itself
        # so we never get worse than holding the previous plan.
        if self.steer_noise_tau > 0.0:
            # Ornstein-Uhlenbeck along the horizon, scaled so the per-step
            # spread is still steer_sigma and the two schemes are compared at
            # the same marginal width.
            a = dt / (self.steer_noise_tau + dt)
            w = rng.normal(0.0, self.steer_sigma * math.sqrt(a * (2.0 - a)),
                           size=(K, N))
            noise_d = np.empty((K, N))
            # Seeded from the stationary distribution, not from zero. Starting
            # the walk at zero leaves the FIRST horizon step barely perturbed
            # (sd 2.8 against 5.0) and that is the step whose action is
            # published, so a zero start handicaps the scheme exactly where it
            # is measured.
            e = rng.normal(0.0, self.steer_sigma, size=K)
            for k in range(N):
                e = (1.0 - a) * e + w[:, k]
                noise_d[:, k] = e
        else:
            noise_d = rng.normal(0.0, self.steer_sigma, size=(K, N))
        noise_a = rng.normal(0.0, self.accel_sigma, size=(K, N))
        noise_d[0] = 0.0
        noise_a[0] = 0.0
        delta_seq = self.u_mean[0, :] + noise_d
        accel_seq = self.u_mean[1, :] + noise_a
        # The sampled sequence is a wheel angle when a map is configured, so
        # turn it into the command that asks for it before the actuator's own
        # limits are applied. Those limits belong to the command: the rack
        # slews in published degrees, not in wheel degrees.
        cmd_seq = self._command(delta_seq) if self._map_cmd.size else delta_seq
        # Steering rate limit enforced sequentially per step
        dr_max = self.steer_rate_max * dt
        prev_d = np.full(K, self.delta_prev)
        for k in range(N):
            lo = prev_d - dr_max
            hi = prev_d + dr_max
            np.clip(cmd_seq[:, k], np.maximum(lo, -self.steer_max),
                    np.minimum(hi, self.steer_max), out=cmd_seq[:, k])
            prev_d = cmd_seq[:, k]
        # What the kart will actually reach, after the limits bit. The cost
        # scores this rather than the request, so a sample asking for more
        # than the rack can give is scored on what it gets.
        if self._map_cmd.size:
            delta_seq = self._wheel(cmd_seq)
        np.clip(accel_seq, self.a_min, self.a_max, out=accel_seq)

        # Vectorized bicycle rollout
        tx = np.empty((K, N))
        ty = np.empty((K, N))
        tpsi = np.empty((K, N))
        tv = np.empty((K, N))
        x = np.full(K, x0)
        y = np.full(K, y0)
        psi = np.full(K, yaw0)
        v = np.full(K, v0)
        # First-order steer-lag state. Initialise actual=delta_prev so the
        # rollout starts from the wheel angle the previous command would have
        # converged to (best proxy when wheel angle isn't measured).
        delta_actual = np.full(K, self.delta_prev)
        alpha_lag = dt / (self.steer_tau_s + dt) if self.steer_tau_s > 0.0 else 1.0
        for k in range(N):
            d_k = cmd_seq[:, k]
            a_k = accel_seq[:, k]
            delta_actual = delta_actual + alpha_lag * (d_k - delta_actual)
            x = x + dt * v * np.cos(psi)
            y = y + dt * v * np.sin(psi)
            psi = psi + dt * v / self.wheelbase * np.tan(self._wheel(delta_actual))
            v = np.clip(v + dt * a_k, 0.0, v_cap)
            tx[:, k] = x
            ty[:, k] = y
            tpsi[:, k] = psi
            tv[:, k] = v

        # Batched Frenet projection onto the windowed line slice ahead of j_now
        lo = max(0, j_now - self.proj_back)
        hi = min(self.line_n, j_now + self.proj_fwd)
        lx = self.l_x[lo:hi]
        ly = self.l_y[lo:hi]
        lp = self.l_psi[lo:hi]
        ls = self.l_s[lo:hi]
        lv = self.l_vx[lo:hi]
        # dx,dy: [K, N, M]
        dx = tx[..., None] - lx
        dy = ty[..., None] - ly
        d2 = dx * dx + dy * dy
        jmin = np.argmin(d2, axis=-1)
        j_psi = lp[jmin]
        j_s = ls[jmin]
        j_vx = lv[jmin]
        ex = tx - lx[jmin]
        ey = ty - ly[jmin]
        cpsi = np.cos(j_psi)
        spsi = np.sin(j_psi)
        s = j_s + ex * cpsi + ey * spsi
        d = -ex * spsi + ey * cpsi

        # Cost terms
        psi_err = _wrap(tpsi - j_psi)
        v_ref = np.minimum(j_vx, v_target)
        v_err = tv - v_ref

        d_diff = np.diff(cmd_seq, axis=1, prepend=self.delta_prev)

        boundary = np.maximum(0.0, np.abs(d) - self.corridor_half)

        progress = s[:, -1] - s[:, 0]

        # Named cost components also exported for diagnostics.
        a_lat = tv * tv * np.tan(
            delta_seq if self._map_cmd.size else self._wheel(delta_seq)
        ) / self.wheelbase
        edge_excess = np.maximum(0.0, np.abs(d) - self.edge_inner)
        c_d = self.w_d * np.sum(d * d, axis=1)
        c_h = self.w_heading * np.sum(psi_err * psi_err, axis=1)
        c_v = self.w_speed * np.sum(v_err * v_err, axis=1)
        c_delta = self.w_delta * np.sum(cmd_seq * cmd_seq, axis=1)
        c_drate = self.w_drate * np.sum(d_diff * d_diff, axis=1)
        c_accel = self.w_accel * np.sum(accel_seq * accel_seq, axis=1)
        c_bnd = self.w_boundary * np.sum(boundary * boundary, axis=1)
        c_edge = self.w_edge * np.sum(edge_excess * edge_excess, axis=1)
        c_prog = -self.w_progress * progress
        c_term_d = self.w_term_d * d[:, -1] ** 2
        c_term_h = self.w_term_h * psi_err[:, -1] ** 2
        c_alat = self.w_a_lat * np.sum(
            np.maximum(0.0, np.abs(a_lat) - self.a_lat_max) ** 2, axis=1,
        )

        cost = (c_d + c_h + c_v + c_delta + c_drate + c_accel
                + c_prog + c_term_h + c_alat + c_edge)

        # Corridor enforcement comes from c_d alone (c_bnd is reported, not
        # scored). Pick the action by averaging the top `mppi_elite_frac × K` samples'
        # control sequences (CEM-style elite mean). The reported "best"
        # cost is still the elite minimum so feasibility / failsafe still
        # gate on the truly-best trajectory.
        n_elite = max(1, int(round(self.mppi_elite_frac * self.K)))
        elite_idx = np.argpartition(cost, n_elite - 1)[:n_elite]
        best = int(elite_idx[np.argmin(cost[elite_idx])])
        best_cost = float(cost[best])
        feasible = best_cost < self.feasibility_threshold
        u = np.stack([
            np.mean(delta_seq[elite_idx, :], axis=0),
            np.mean(accel_seq[elite_idx, :], axis=0),
        ], axis=0)
        # Handed over on the instance rather than returned: `_solve`'s
        # signature has other callers (sim/apex_repro.py sweeps it directly).
        self._cmd_out = float(np.mean(cmd_seq[elite_idx, 0]))
        traj = {"d": d[best], "s": s[best]}
        breakdown = (
            float(c_d[best]), float(c_h[best]), float(c_v[best]),
            float(c_delta[best]), float(c_drate[best]), float(c_accel[best]),
            float(c_bnd[best]), float(c_prog[best]),
            float(c_term_d[best]), float(c_term_h[best]), float(c_alat[best]),
            float(c_edge[best]),
        )
        return u, best_cost, traj, feasible, breakdown

    def set_racing_line(self, racing_line: list) -> None:
        """Swap in a new racing line. Residual state is preserved (features
        are kart-state relative); Frenet / warm-start are reset."""
        self.racing_line = racing_line
        self._static_arrays = self._build_line_arrays(racing_line)
        self._bezier_arrays = None
        self._set_active_line(self._static_arrays)
        self.line_manager = DynamicLineManager(racing_line, logger=self.logger)
        self.line_manager.register(RejoinStrategy(**self._rejoin_kwargs))
        self.closest_idx = 0
        self.static_closest_idx = 0
        self.initial_sync_done = False
        self._reset_warm_start()
        self._pose_hist.clear()

    def train_step(self, motor_mps: float, steering_deg: float,
                   x: float, y: float, yaw: float, v: float, now_ns: int) -> None:
        """Drive the residual learner from another planner's commands so it
        keeps training when MPC isn't the active planner."""
        # Sync MPC's internal control state to what was actually executed.
        self.delta_prev = math.radians(steering_deg)
        self.u_mean[0, :] = self.delta_prev
        if not self.residual.enabled:
            return
        t0 = time.perf_counter()
        self._set_active_line(self._static_arrays)
        s, d, j_now, psi_track, _ = self._frenet(x, y, self.closest_idx)
        self.closest_idx = j_now
        self._pose_hist.append((now_ns, s, d))
        v_s, v_d = self._frenet_velocity()
        delta_rad = math.radians(steering_deg)
        accel = (motor_mps - self._last_motor_mps) / max(self._train_dt, 1e-3)
        self._last_motor_mps = motor_mps
        self._steer_hist.appendleft(delta_rad)
        self._throttle_hist.appendleft(accel)
        nom_x, nom_y = self._hold_rollout(x, y, yaw, v, delta_rad, accel, self._nom_steps)
        nom_s, nom_d, _, _, _ = self._frenet(nom_x, nom_y, j_now)
        kappa_local = self._curvature_at(j_now)
        phi = _features(
            d, v_s, v_d, kappa_local,
            tuple(self._steer_hist), tuple(self._throttle_hist),
            nom_s - s, nom_d - d,
        )
        self.residual.push(phi, s, d, nom_s - s, nom_d - d, v)
        self.residual.step(s, d)
        res_ps, res_pd, _residual_source = self.residual.predict(phi)
        # effective_mode gating: zero the residual any time we're not yet in apply
        if self.residual.effective_mode() != "apply":
            res_ps, res_pd = 0.0, 0.0
        self._publish_status(
            self.mode, True, t0, s, d, psi_track, v, self.target_speed,
            delta_rad, accel, 0.0, self.corridor_half,
            res_ps, res_pd,
            breakdown=(0.0,) * 12,
            x=x, y=y, yaw_rad=yaw,
            throttle_mps_out=motor_mps, v_s=v_s, v_d=v_d,
            closest_idx_active=j_now,
            closest_idx_static=j_now,
            rejoin_active=False, merge_idx=-1,
            kappa_local=kappa_local,
            consec_failures=0,
            corridor_half=self.corridor_half,
        )

    #  telemetry
    def _publish_status(self, mode, success, t0, s, d, psi_track, v, v_target,
                        delta_cmd, accel_cmd, cost, margin_min,
                        res_s=0.0, res_d=0.0, breakdown=(0.0,) * 12,
                        x=0.0, y=0.0, yaw_rad=0.0,
                        throttle_mps_out=0.0, v_s=0.0, v_d=0.0,
                        closest_idx_active=0, closest_idx_static=0,
                        rejoin_active=False, merge_idx=-1,
                        kappa_local=0.0, consec_failures=0,
                        corridor_half=0.0):
        if self.status_pub is None:
            return
        solve_ms = (time.perf_counter() - t0) * 1000.0
        nom_s, nom_d, res_es, res_ed = self.residual.mean_error()
        trainer = self.residual.trainer
        last = trainer.snapshot_models()[2] if trainer else None
        payload = [
            float(mode), 1.0 if success else 0.0, solve_ms,
            float(s), float(d), float(psi_track), float(v), float(v_target),
            float(math.degrees(delta_cmd)), float(accel_cmd),
            float(cost) if math.isfinite(cost) else -1.0,
            float(margin_min),
            float(res_s), float(res_d),
            nom_s, nom_d, res_es, res_ed,
            float(self.residual.samples_trained),
            *breakdown,
            float(x), float(y), float(yaw_rad),
            float(throttle_mps_out), float(v_s), float(v_d),
            float(closest_idx_active), float(closest_idx_static),
            1.0 if rejoin_active else 0.0, float(merge_idx),
            float(kappa_local), float(consec_failures),
            float(corridor_half),
            float({"off": 0, "shadow": 1, "apply": 2}.get(self.residual.mode, 1)),
            float(np.linalg.norm(self.residual.theta_s)),
            float(np.linalg.norm(self.residual.theta_d)),
            # Raw GPS pose (EKF-vs-GPS diff). NaN until first /gps fix.
            float(self._gps_x), float(self._gps_y),
            float(self._gps_yaw), float(self._gps_v),
            # Wheel speed + last-GPS-event EKF prior/posterior. seq lets the
            # analyst dedupe MPC ticks (60 Hz) against GPS events (10 Hz).
            float(self._wheel_v), float(self._gps_event_seq),
            float(self._gps_have_yaw), float(self._gps_have_speed),
            float(self._ekf_prior_x), float(self._ekf_prior_y),
            float(self._ekf_prior_yaw), float(self._ekf_prior_v),
            float(self._ekf_post_x), float(self._ekf_post_y),
            float(self._ekf_post_yaw), float(self._ekf_post_v),
            # ---- Phase 1 residual telemetry (Phase 2 fields stay 0/NaN here) ----
            float(self.residual.buffer.size if self.residual.buffer is not None else 0),
            float(self.residual.buffer.capacity if self.residual.buffer is not None else 0),
            float(last.train_wall_ms if last is not None else float("nan")),
            float(last.n_samples if last is not None else 0),
            float(trainer.train_seq if trainer else 0),
            float(last.train_mae_s if last is not None else float("nan")),
            float(last.train_mae_d if last is not None else float("nan")),
            float(last.val_mae_s if last is not None else float("nan")),
            float(last.val_mae_d if last is not None else float("nan")),
            float(last.rls_val_mae_s if last is not None else float("nan")),
            float(last.rls_val_mae_d if last is not None else float("nan")),
            float(self.residual.last_active_model),  # active_model
            float(self.residual.last_pred_clipped),
            float(self.residual.clip_rate()),
            float(self.residual.outliers_dropped),
            float(self.residual.off_line_skipped),
            float(self.residual.divergence_resets),
            float(self.residual.samples_trained),
            float(self.residual.revert_count),
            float(self.residual.checkpoint_ring.best_recent().val_mae_s
                  if self.residual.checkpoint_ring and not self.residual.checkpoint_ring.is_empty()
                  else float("nan")),
            float(self.residual.checkpoint_ring.best_recent().val_mae_d
                  if self.residual.checkpoint_ring and not self.residual.checkpoint_ring.is_empty()
                  else float("nan")),
            float(1.0 if self.residual.cache_loaded else 0.0),
            float(1.0 if self.residual.samples_trained >= self.residual.rls_warmup_samples else 0.0),
            float({"off": 0, "shadow": 1, "apply": 2}.get(self.residual.effective_mode(), 1)),
            float(self.residual.samples_accepted_this_run),
        ]
        assert len(payload) == 88, f"mpc/status payload must be 88 floats, got {len(payload)}"
        self.status_pub.publish(Float32MultiArray(data=payload))

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
