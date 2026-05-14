"""
NORMAL  : track racing line inside the corridor
REJOIN  : kart outside the corridor
FAILSAFE: stale state / no feasible plan

Solves every pathfinder tick (60 Hz) by rolling out K random control
sequences with the kinematic bicycle model and picking the lowest-cost
sequence whose trajectory stays inside the corridor.
"""

import math
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np
from std_msgs.msg import Float32MultiArray

from autonomous_kart.nodes.pathfinder.planners.base import (
    KartConstants, Planner, PlannerInputs,
)
from autonomous_kart.nodes.pathfinder.planners.mpc_residual import (
    NUM_STEER_HIST, NUM_THROTTLE_HIST, ResidualLearner, _features,
)

MODE_NORMAL, MODE_REJOIN, MODE_FAILSAFE = 0, 1, 2


def _wrap(angle):
    """Wrap to (-pi, pi] — works on scalars or numpy arrays."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MPCPlanner(Planner):
    name = "mpc"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list,
                 logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)

        if not racing_line:
            raise ValueError("MPCPlanner requires a non-empty racing line")

        # Horizon / sampling
        self.N = int(params["horizon_steps"])
        self.dt = float(params["dt_s"])
        self.K = int(params["num_samples"])
        self.steer_sigma = math.radians(float(params["steer_sigma_deg"]))
        self.accel_sigma = float(params["accel_sigma_mps2"])
        self.proj_back = int(params["proj_window_back"])
        self.proj_fwd = int(params["proj_window_fwd"])
        self.v_window_s = float(params["frenet_v_window_s"])

        # Physical limits pulled from kart constants (kart-wide /**: block)
        self.steer_max = math.radians(kart.steer_max_deg)
        self.steer_rate_max = math.radians(kart.steer_rate_max_degps)
        self.v_max = kart.v_max_mps
        self.wheelbase = kart.wheelbase_m
        self.a_min = kart.a_min_mps2
        self.a_max = kart.a_max_mps2
        self.a_lat_max = kart.a_lat_max_mps2
        self.target_speed = float(params["target_speed_pct"]) * self.v_max

        # Corridor (centerline = racing line)
        tw = float(params["track_half_width_m"])
        sm = float(params["safety_margin_m"])
        kw = float(params["kart_half_width_m"])
        self.corridor_half = tw - sm - kw

        # Costs
        self.w_d = float(params["w_d"])
        self.w_heading = float(params["w_heading"])
        self.w_speed = float(params["w_speed"])
        self.w_delta = float(params["w_delta"])
        self.w_drate = float(params["w_delta_rate"])
        self.w_accel = float(params["w_accel"])
        self.w_boundary = float(params["w_boundary"])
        self.w_progress = float(params["w_progress"])
        self.w_term_d = float(params["w_terminal_d"])
        self.w_term_h = float(params["w_terminal_heading"])
        self.w_a_lat = float(params["w_a_lat"])

        # Rejoin
        self.rejoin_in = float(params["rejoin_activate_d_m"])
        self.rejoin_out = float(params["rejoin_deactivate_d_m"])
        self.rejoin_v = float(params["rejoin_v_max_mps"])
        self.max_failures = int(params["max_consecutive_solver_failures"])

        # Feasibility sentinels
        self.infeasible_cost = float(params["infeasible_cost"])
        self.rejoin_terminal_penalty = float(params["rejoin_terminal_penalty"])
        self.feasibility_threshold = float(params["feasibility_threshold"])

        # Racing line as flat numpy arrays with derived tangent for Frenet
        L = np.asarray([list(r[:6]) for r in racing_line], dtype=np.float64)
        self.l_s = L[:, 0]
        self.l_x = L[:, 1]
        self.l_y = L[:, 2]
        self.l_vx = L[:, 5]
        self.line_n = L.shape[0]
        dx = np.empty_like(self.l_x)
        dy = np.empty_like(self.l_y)
        dx[1:-1] = self.l_x[2:] - self.l_x[:-2]
        dy[1:-1] = self.l_y[2:] - self.l_y[:-2]
        dx[0] = self.l_x[1] - self.l_x[0]
        dy[0] = self.l_y[1] - self.l_y[0]
        dx[-1] = self.l_x[-1] - self.l_x[-2]
        dy[-1] = self.l_y[-1] - self.l_y[-2]
        self.l_psi = np.arctan2(dy, dx)

        # State carried between ticks
        self.closest_idx = 0
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
        self.residual = ResidualLearner(residual_params, solve_dt)
        self._nom_steps = max(1, int(round(self.residual.target_horizon_s / self.dt)))
        self._steer_hist = deque([0.0] * NUM_STEER_HIST, maxlen=NUM_STEER_HIST)
        self._throttle_hist = deque([0.0] * NUM_THROTTLE_HIST, maxlen=NUM_THROTTLE_HIST)
        # Pose history covers v_window_s of past samples at the solve cadence.
        pose_hist_len = max(2, int(round(self.v_window_s / solve_dt)) + 2)
        self._pose_hist = deque(maxlen=pose_hist_len)  # (now_ns, s, d)

        # Telemetry publisher
        self.status_pub = None
        if node is not None:
            self.status_pub = node.create_publisher(Float32MultiArray, "mpc/status", 5)

        # Reusable rng
        self._rng = np.random.default_rng(0)

    # plan
    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        t0 = time.perf_counter()
        x, y = inputs.pose_xy
        yaw = inputs.yaw_rad
        v = max(0.0, inputs.speed_mps)

        s, d, j_now, psi_track, _ = self._frenet(x, y, self.closest_idx)
        self.closest_idx = j_now

        # Track pose history for v_s / v_d estimates
        self._pose_hist.append((inputs.now_ns, s, d))
        v_s, v_d = self._frenet_velocity()

        # Decide mode
        abs_d = abs(d)
        if self.mode != MODE_REJOIN and abs_d > self.rejoin_in:
            self.mode = MODE_REJOIN
        elif self.mode == MODE_REJOIN and abs_d < self.rejoin_out:
            self.mode = MODE_NORMAL

        v_cap = self.rejoin_v if self.mode == MODE_REJOIN else self.v_max
        v_target = min(self.target_speed, v_cap)

        # Build candidate trajectories, evaluate, pick best
        try:
            best_u, best_cost, traj, feasible = self._solve(
                x, y, yaw, v, j_now, v_target, v_cap,
            )
        except Exception:
            # NaN propagation - skip
            best_u, best_cost, traj, feasible = None, float("inf"), None, False

        if not feasible or best_u is None:
            self.consec_failures += 1
            if self.consec_failures >= self.max_failures:
                self.mode = MODE_FAILSAFE
            self._publish_status(self.mode, False, t0, s, d, psi_track, v, v_target,
                                 0.0, 0.0, best_cost, 0.0)
            return 0.0, 0.0

        self.consec_failures = 0
        delta_cmd = float(best_u[0, 0])
        accel_cmd = float(best_u[1, 0])

        # Convert to publish convention: throttle [0..100], steering [deg]
        v_next = max(0.0, min(self.v_max, v + self.dt * accel_cmd))
        throttle_pct = 100.0 * v_next / self.v_max if self.v_max > 1e-6 else 0.0
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
        res_ps, res_pd = self.residual.predict(phi)

        # Telemetry
        margin_min = self.corridor_half - float(np.max(np.abs(traj["d"])))
        self._publish_status(self.mode, True, t0, s, d, psi_track, v, v_target,
                             delta_cmd, accel_cmd, best_cost, margin_min,
                             res_ps, res_pd)

        return throttle_pct, steering_deg

    # helpers
    def _frenet(self, x: float, y: float, hint: int):
        """Returns (s, d, idx, psi_track, kappa). d is signed (left positive)."""
        lo = max(0, hint - self.proj_back)
        hi = min(self.line_n, hint + self.proj_fwd)
        dx = self.l_x[lo:hi] - x
        dy = self.l_y[lo:hi] - y
        j_rel = int(np.argmin(dx * dx + dy * dy))
        j = lo + j_rel
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

    def _hold_rollout(self, x, y, yaw, v, delta, a, n):
        """Single-trajectory bicycle rollout — used by the residual learner."""
        for _ in range(n):
            x += self.dt * v * math.cos(yaw)
            y += self.dt * v * math.sin(yaw)
            yaw += self.dt * v / self.wheelbase * math.tan(delta)
            v = max(0.0, min(self.v_max, v + self.dt * a))
        return x, y

    # solve
    def _solve(self, x0, y0, yaw0, v0, j_now, v_target, v_cap):
        K, N, dt = self.K, self.N, self.dt
        rng = self._rng

        # Sample around warm-start mean. Sample[0] is the warm-start itself
        # so we never get worse than holding the previous plan.
        noise_d = rng.normal(0.0, self.steer_sigma, size=(K, N))
        noise_a = rng.normal(0.0, self.accel_sigma, size=(K, N))
        noise_d[0] = 0.0
        noise_a[0] = 0.0
        delta_seq = self.u_mean[0, :] + noise_d
        accel_seq = self.u_mean[1, :] + noise_a
        # Steering rate limit enforced sequentially per step
        dr_max = self.steer_rate_max * dt
        prev_d = np.full(K, self.delta_prev)
        for k in range(N):
            lo = prev_d - dr_max
            hi = prev_d + dr_max
            np.clip(delta_seq[:, k], np.maximum(lo, -self.steer_max),
                    np.minimum(hi, self.steer_max), out=delta_seq[:, k])
            prev_d = delta_seq[:, k]
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
        for k in range(N):
            d_k = delta_seq[:, k]
            a_k = accel_seq[:, k]
            x = x + dt * v * np.cos(psi)
            y = y + dt * v * np.sin(psi)
            psi = psi + dt * v / self.wheelbase * np.tan(d_k)
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

        d_diff = np.diff(delta_seq, axis=1, prepend=self.delta_prev)

        boundary = np.maximum(0.0, np.abs(d) - self.corridor_half)
        hard_violation = np.any(boundary > 0.0, axis=1)

        progress = s[:, -1] - s[:, 0]

        cost = (
                self.w_d * np.sum(d * d, axis=1)
                + self.w_heading * np.sum(psi_err * psi_err, axis=1)
                + self.w_speed * np.sum(v_err * v_err, axis=1)
                + self.w_delta * np.sum(delta_seq * delta_seq, axis=1)
                + self.w_drate * np.sum(d_diff * d_diff, axis=1)
                + self.w_accel * np.sum(accel_seq * accel_seq, axis=1)
                + self.w_boundary * np.sum(boundary * boundary, axis=1)
                - self.w_progress * progress
                + self.w_term_d * d[:, -1] ** 2
                + self.w_term_h * psi_err[:, -1] ** 2
        )

        a_lat = tv * tv * np.tan(delta_seq) / self.wheelbase
        cost += self.w_a_lat * np.sum(
            np.maximum(0.0, np.abs(a_lat) - self.a_lat_max) ** 2, axis=1,
        )

        # In rejoin mode allow corridor violations
        if self.mode == MODE_NORMAL:
            cost = np.where(hard_violation, self.infeasible_cost, cost)
        else:
            terminal_outside = np.abs(d[:, -1]) > self.corridor_half
            cost = np.where(terminal_outside, cost + self.rejoin_terminal_penalty, cost)

        best = int(np.argmin(cost))
        best_cost = float(cost[best])
        feasible = best_cost < self.feasibility_threshold
        u = np.stack([delta_seq[best], accel_seq[best]], axis=0)
        traj = {"d": d[best], "s": s[best]}
        return u, best_cost, traj, feasible

    #  telemetry
    def _publish_status(self, mode, success, t0, s, d, psi_track, v, v_target,
                        delta_cmd, accel_cmd, cost, margin_min,
                        res_s=0.0, res_d=0.0):
        if self.status_pub is None:
            return
        solve_ms = (time.perf_counter() - t0) * 1000.0
        nom_s, nom_d, res_es, res_ed = self.residual.mean_error()
        payload = [
            float(mode), 1.0 if success else 0.0, solve_ms,
            float(s), float(d), float(psi_track), float(v), float(v_target),
            float(math.degrees(delta_cmd)), float(accel_cmd),
            float(cost) if math.isfinite(cost) else -1.0,
            float(margin_min),
            float(res_s), float(res_d),
            nom_s, nom_d, res_es, res_ed,
            float(self.residual.samples_trained),
        ]
        self.status_pub.publish(Float32MultiArray(data=payload))
