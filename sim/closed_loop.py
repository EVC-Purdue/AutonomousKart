"""Standalone closed-loop simulator for the autonomous kart MPC planner.

This module is a library. Use sim/runner.py for the CLI.
"""
from __future__ import annotations

import math
import os
import sys
import types
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# ROS stubs — MPCPlanner imports std_msgs.msg.Float32MultiArray at module level.
# ---------------------------------------------------------------------------
def _install_ros_stubs() -> None:
    if "std_msgs" not in sys.modules:
        std_msgs = types.ModuleType("std_msgs")
        msg_mod = types.ModuleType("std_msgs.msg")

        class Float32MultiArray:  # noqa: D401
            def __init__(self, data=None):
                self.data = list(data) if data is not None else []

        msg_mod.Float32MultiArray = Float32MultiArray
        std_msgs.msg = msg_mod
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = msg_mod


_install_ros_stubs()

# Add the package source to sys.path so we can import the MPC planner directly.
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PKG_SRC = os.path.join(REPO, "src", "autonomous_kart")
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from autonomous_kart.nodes.localization.sim_bicycle import BicycleModel  # noqa: E402
from autonomous_kart.nodes.pathfinder.planners.base import (  # noqa: E402
    KartConstants,
    PlannerInputs,
)
from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner  # noqa: E402


# ---------------------------------------------------------------------------
# Mock node — MPCPlanner needs node.get_parameter and node.create_publisher.
# ---------------------------------------------------------------------------
class _MockParam:
    def __init__(self, value):
        self.value = value


class _MockPub:
    def publish(self, _msg):
        pass


class MockNode:
    def __init__(self, system_frequency: float):
        self._params = {"system_frequency": system_frequency}

    def get_parameter(self, name: str) -> _MockParam:
        return _MockParam(self._params[name])

    def create_publisher(self, _msg_type, _topic: str, _qos: int) -> _MockPub:
        return _MockPub()


# ---------------------------------------------------------------------------
# SimResult
# ---------------------------------------------------------------------------
@dataclass
class SimResult:
    completed_laps: int
    total_time_s: float
    avg_lap_time_s: float
    max_abs_d: float
    avg_speed: float
    safe: bool
    aborted: bool
    xs: np.ndarray       # (N,) m
    ys: np.ndarray       # (N,) m
    yaws: np.ndarray     # (N,) rad
    vs: np.ndarray       # (N,) m/s
    ds: np.ndarray       # (N,) signed lateral offset from racing line
    times: np.ndarray    # (N,) seconds
    # Diagnostics captured from the planner's residual learner at end of run.
    # Empty dict if planner/learner missing or capture failed.
    residual_stats: dict = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Default MPC params (from pathfinder.yaml).
# ---------------------------------------------------------------------------
# Pure pursuit as params/pathfinder.yaml ships it. `planner: pure_pursuit`
# there, so this is the controller the kart actually drove all season.
DEFAULT_PP = {
    "use_velocity_scaled_lookahead": True,
    "lookahead_time_s": 0.6,
    "min_lookahead_m": 3.0,
    "max_lookahead_m": 8.0,
    "use_curvature_regulation": True,
    "min_radius_m": 7.0,
    "min_reg_speed_mps": 7.0,
    "approach_dist_m": 1.0,
    "min_approach_speed_mps": 1.0,
    "search_window": 80,
    "initial_sync_done": False,
    "max_resync_dist": 80,
    "max_closed_dist": 2,
    "rejoin_cte_activate": 3.0,
    "rejoin_cte_deactivate": 1.0,
    "rejoin_merge_lookahead_m": 15.0,
    "latency_s": 0.3,
    "steering_gain": 2.0,
}

DEFAULT_MPC = {
    "horizon_steps": 20,
    "dt_s": 0.05,
    "num_samples": 48,
    "steer_sigma_deg": 6.0,
    "accel_sigma_mps2": 1.0,
    "proj_window_back": 5,
    "proj_window_fwd": 60,
    "frenet_v_window_s": 0.3,
    # NOTE: the planner reads `target_speed_mps` (a literal m/s cap), NOT a
    # percentage. The kart's actual speed is min(line's per-tick vx_mps,
    # target_speed_mps). For line6.csv vx_mps=6 throughout, the line drives
    # the speed unless target_speed_mps is set lower.
    "target_speed_mps": 12.0,
    "target_speed_pct": 1.0,  # unused by planner; kept for backward compat
    "track_half_width_m": 2.0,
    "safety_margin_m": 0.5,
    "kart_half_width_m": 0.5025,
    "w_d": 100.0,
    "w_heading": 30.0,
    "w_speed": 5.0,
    "w_delta": 5.0,
    "w_delta_rate": 50.0,
    "w_accel": 2.0,
    "w_boundary": 1000.0,
    "w_progress": 1.0,
    "w_terminal_d": 200.0,
    "w_terminal_heading": 100.0,
    "w_a_lat": 50.0,
    "w_edge": 200.0,
    "edge_inner_frac": 0.5,
    "rejoin_cte_activate": 2.5,
    "rejoin_cte_deactivate": 1.0,
    "rejoin_merge_lookahead_m": 15.0,
    "rejoin_min_turning_radius": 3.0,
    "max_consecutive_solver_failures": 3,
    "infeasible_cost": 1.0e9,
    "feasibility_threshold": 5.0e8,
    # residual learner runs in shadow mode by default.
    # cache disabled so the script runs without a writable /root/.cache.
    "residual.mode": "shadow",
    "residual.cache_enabled": False,
    "residual.target_horizon_s": 0.5,
    "residual.forgetting_factor": 0.99,
    "residual.initial_cov": 1000.0,
    "residual.min_train_speed_mps": 0.5,
    "residual.error_window": 200,
}


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------
def _line_xy(line: List[Tuple[float, ...]]) -> Tuple[np.ndarray, np.ndarray]:
    arr = np.asarray([[r[1], r[2]] for r in line], dtype=np.float64)
    return arr[:, 0], arr[:, 1]


def _line_tangent_normal(
    lx: np.ndarray, ly: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    dx = np.empty_like(lx)
    dy = np.empty_like(ly)
    dx[1:-1] = lx[2:] - lx[:-2]
    dy[1:-1] = ly[2:] - ly[:-2]
    dx[0] = lx[1] - lx[0]
    dy[0] = ly[1] - ly[0]
    dx[-1] = lx[-1] - lx[-2]
    dy[-1] = ly[-1] - ly[-2]
    L = np.hypot(dx, dy)
    L = np.where(L < 1e-9, 1.0, L)
    tx = dx / L
    ty = dy / L
    # Left-hand normal (positive d side, matches MPC Frenet convention).
    nx = -ty
    ny = tx
    return nx, ny


def _line_length(line: List[Tuple[float, ...]]) -> float:
    s_last = float(line[-1][0])
    x0, y0 = float(line[0][1]), float(line[0][2])
    xn, yn = float(line[-1][1]), float(line[-1][2])
    seg = math.hypot(xn - x0, yn - y0)
    return s_last + seg


def _signed_d_at(
    x: float, y: float, j: int,
    lx: np.ndarray, ly: np.ndarray,
    nx: np.ndarray, ny: np.ndarray,
) -> float:
    return float((x - lx[j]) * nx[j] + (y - ly[j]) * ny[j])


def _kart_from_yaml() -> KartConstants:
    """KartConstants for the MPC planner — sourced from KartPhysics."""
    from sim.kart_params import KartPhysics
    return KartPhysics().to_kart_constants()


# ---------------------------------------------------------------------------
# Main simulate() function
# ---------------------------------------------------------------------------
def simulate(
    line: List[Tuple[float, ...]],
    mpc_params: dict,
    *,
    planner_kind: str = "mpc",
    sim_backend: str = "datasim",
    datasim_model_dir: str = "sim/model",
    datasim_use_mlp: bool = True,
    noise_mode: str = "none",
    rng_seed: int = 0,
    n_laps: int = 2,
    dt: float = 1.0 / 60.0,
    max_steps: int = 30000,
    track_half_width: float = 2.0,
    kart_half_width: float = 0.5025,
    safety_margin: float = 0.2,
    steering_gain: float = 1.0,
    actuation_delay_s: float = 0.0,
    delay_comp_s: float = 0.0,
) -> SimResult:
    """Run a closed-loop MPC simulation over `n_laps` of the racing line.

    Parameters
    ----------
    line:
        Racing line as a list of tuples ``(s, x, y, ...)``.
    mpc_params:
        Planner parameter dict (see DEFAULT_MPC / DEFAULT_PP).
    planner_kind:
        ``"mpc"`` for MPCPlanner, ``"pure_pursuit"`` for PurePursuitPlanner.
    sim_backend:
        ``"bicycle"`` for BicycleModel, ``"datasim"`` for DataSim, ``"learned"``
        for LearnedPlant, ``"mujoco"`` for the MuJoCo kart.
    datasim_model_dir:
        Directory containing the DataSim model files
        (``bicycle_params.json``, ``noise.json``, ``mlp.pt``, etc.).
    noise_mode:
        Passed through to DataSim (``"none"``, ``"gaussian"``, ``"bootstrap"``).
    rng_seed:
        RNG seed for both MPC and DataSim noise.
    n_laps:
        Number of laps to complete before stopping.
    dt:
        Simulation timestep in seconds.
    max_steps:
        Hard cap on simulation steps.
    track_half_width:
        Half-width of the track in metres.
    kart_half_width:
        Half-width of the kart in metres.
    safety_margin:
        Additional margin from track edge (safety check).
    actuation_delay_s:
        Pure transport delay between a command being computed and the plant
        seeing it. The plant's own first-order actuator lag is separate.
    delay_comp_s:
        If > 0, the planner is handed a state propagated this far forward
        through the commands already in flight, using the planner's own
        bicycle model. Set it equal to actuation_delay_s to compensate.
    """
    kart = _kart_from_yaml()

    # Build sim backend
    if sim_backend == "bicycle":
        model = BicycleModel(kart.wheelbase_m, kart.v_max_mps, kart.steer_max_deg)
    elif sim_backend == "datasim":
        from sim.data_sim import DataSim
        # Resolve relative model dir from repo root
        mdir = datasim_model_dir if os.path.isabs(datasim_model_dir) else os.path.join(REPO, datasim_model_dir)
        # Bicycle physics come from KartPhysics (yaml-aligned), not from a
        # bag-fit JSON. The MLP residual / clamp files (if present) layer on
        # top to model anything the structured bicycle can't explain. Disable
        # with datasim_use_mlp=False — recommended in closed loop since the
        # currently-trained MLP has a positive dv bias that compounds.
        mlp_path = os.path.join(mdir, "mlp.pt") if datasim_use_mlp else None
        norm_path = os.path.join(mdir, "norm.json") if datasim_use_mlp else None
        clamp_path = os.path.join(mdir, "clamp.json") if datasim_use_mlp else None
        # Optional: a user-supplied kart_params.json overlay in the model dir
        # bypasses the code-level defaults.  Format: same as KartPhysics fields.
        kart_params_json = os.path.join(mdir, "kart_params.json")
        model = DataSim(
            params_path=os.path.join(mdir, "bicycle_params.json"),
            noise_path=os.path.join(mdir, "noise.json"),
            mlp_path=mlp_path,
            norm_path=norm_path,
            clamp_path=clamp_path,
            residual_emp_path=os.path.join(mdir, "residual_emp.npz"),
            noise_mode=noise_mode,
            rng_seed=rng_seed,
        )
    elif sim_backend == "linear":
        from sim.learned_plant import LearnedPlant
        mdir = datasim_model_dir if os.path.isabs(datasim_model_dir) \
            else os.path.join(REPO, datasim_model_dir)
        model = LearnedPlant.from_linear(os.path.join(mdir, "plant_linear.json"))
    elif sim_backend == "learned":
        from sim.learned_plant import LearnedPlant
        mdir = datasim_model_dir if os.path.isabs(datasim_model_dir) \
            else os.path.join(REPO, datasim_model_dir)
        model = LearnedPlant.from_files(os.path.join(mdir, "plant_nn.pt"),
                                        os.path.join(mdir, "plant_nn.json"))
    elif sim_backend == "mujoco":
        from sim.mj.kart import KartSpec
        from sim.mj.plant import MuJoCoPlant
        mdir = datasim_model_dir if os.path.isabs(datasim_model_dir) \
            else os.path.join(REPO, datasim_model_dir)
        spec_path = os.path.join(mdir, "mj_kart_september_spec.json")
        if os.path.exists(spec_path):
            import json as _json
            with open(spec_path) as fh:
                spec = KartSpec.from_dict(_json.load(fh))
        else:
            spec = KartSpec()
        model = MuJoCoPlant(spec)
    else:
        raise ValueError(f"unknown sim_backend {sim_backend!r}")

    node = MockNode(system_frequency=1.0 / dt)
    if planner_kind == "pure_pursuit":
        from autonomous_kart.nodes.pathfinder.planners.pure_pursuit import (
            PurePursuitPlanner)
        planner = PurePursuitPlanner(mpc_params, kart, line, logger=None, node=node)
    elif planner_kind == "mpc":
        planner = MPCPlanner(mpc_params, kart, line, logger=None, node=node)
        # Only the sampling planner draws; seeding a deterministic one would
        # attach an attribute nothing reads.
        planner._rng = np.random.default_rng(rng_seed)
    else:
        raise ValueError(f"unknown planner_kind {planner_kind!r}")

    x0, y0 = float(line[0][1]), float(line[0][2])
    yaw0 = math.atan2(float(line[1][2]) - y0, float(line[1][1]) - x0)
    model.reset(x0, y0, yaw0)

    lx, ly = _line_xy(line)
    nx_arr, ny_arr = _line_tangent_normal(lx, ly)
    line_L = _line_length(line)
    s_arr = np.asarray([r[0] for r in line], dtype=np.float64)
    n_line = len(line)

    xs: List[float] = []
    ys: List[float] = []
    yaws: List[float] = []
    vs: List[float] = []
    ds_list: List[float] = []
    times: List[float] = []
    speeds: List[float] = []

    laps = 0
    safety_limit = track_half_width - kart_half_width - safety_margin

    # Forward-only progress tracker (wrap-aware).
    progress_idx = 0
    cumulative_s = 0.0
    s_prev = float(s_arr[0])
    win_back = 3
    win_fwd = 25

    t0_ns = 0
    completion_step = max_steps

    n_delay = int(round(max(0.0, actuation_delay_s) / dt))
    cmd_queue: List[Tuple[float, float]] = [(0.0, 0.0)] * n_delay
    n_comp = int(round(max(0.0, delay_comp_s) / dt))
    pred_delta = 0.0  # controller-side estimate of the actual wheel angle

    for step in range(max_steps):
        now_ns = t0_ns + int(step * dt * 1e9)
        px, py, pyaw, pv = model.x, model.y, model.yaw, model.speed
        if n_comp > 0:
            px, py, pyaw, pv, pred_delta = _predict_forward(
                planner, px, py, pyaw, pv, pred_delta, cmd_queue, n_comp, dt
            )
        inputs = PlannerInputs(
            pose_xy=(px, py),
            yaw_rad=pyaw,
            speed_mps=pv,
            track_angles=None,
            now_ns=now_ns,
        )
        try:
            cmd = planner.plan(inputs)
        except Exception:
            return SimResult(
                completed_laps=laps,
                total_time_s=step * dt,
                avg_lap_time_s=float("inf"),
                max_abs_d=float(max(abs(d) for d in ds_list)) if ds_list else 99.0,
                avg_speed=float(np.mean(speeds) if speeds else 0.0),
                safe=False,
                aborted=True,
                xs=np.asarray(xs),
                ys=np.asarray(ys),
                yaws=np.asarray(yaws),
                vs=np.asarray(vs),
                ds=np.asarray(ds_list),
                times=np.asarray(times),
                residual_stats=_capture_residual_stats(planner),
            )

        if n_delay > 0:
            cmd_queue.append(cmd)
            motor_pct, steer_deg = cmd_queue.pop(0)
        else:
            motor_pct, steer_deg = cmd
        model.step(motor_pct, steer_deg * steering_gain, dt)

        # Forward-windowed nearest-point search
        idxs = (progress_idx + np.arange(-win_back, win_fwd + 1)) % n_line
        wx = lx[idxs] - model.x
        wy = ly[idxs] - model.y
        wj = int(np.argmin(wx * wx + wy * wy))
        j = int(idxs[wj])
        progress_idx = j

        d_signed = _signed_d_at(model.x, model.y, j, lx, ly, nx_arr, ny_arr)
        t_now = step * dt

        xs.append(model.x)
        ys.append(model.y)
        yaws.append(model.yaw)
        vs.append(model.speed)
        ds_list.append(d_signed)
        times.append(t_now)
        speeds.append(model.speed)

        # Bail if completely off the rails.
        if abs(d_signed) > 10.0:
            return SimResult(
                completed_laps=laps,
                total_time_s=step * dt,
                avg_lap_time_s=float("inf"),
                max_abs_d=float(max(abs(d) for d in ds_list)),
                avg_speed=float(np.mean(speeds)),
                safe=False,
                aborted=True,
                xs=np.asarray(xs),
                ys=np.asarray(ys),
                yaws=np.asarray(yaws),
                vs=np.asarray(vs),
                ds=np.asarray(ds_list),
                times=np.asarray(times),
                residual_stats=_capture_residual_stats(planner),
            )

        # Cumulative arc-length with wrap-aware delta.
        s_now = float(s_arr[j])
        ds_step = s_now - s_prev
        if ds_step > line_L * 0.5:
            ds_step -= line_L
        elif ds_step < -line_L * 0.5:
            ds_step += line_L
        cumulative_s += ds_step
        s_prev = s_now
        new_laps = int(cumulative_s // line_L)
        if new_laps > laps:
            laps = new_laps
            if laps >= n_laps:
                completion_step = step + 1
                break

    total_time = completion_step * dt
    avg_lap_time = total_time / max(1, laps)
    xs_a = np.asarray(xs)
    ys_a = np.asarray(ys)
    yaws_a = np.asarray(yaws)
    vs_a = np.asarray(vs)
    ds_a = np.asarray(ds_list)
    times_a = np.asarray(times)
    max_abs_d = float(np.max(np.abs(ds_a))) if ds_a.size else 99.0
    avg_speed = float(np.mean(speeds)) if speeds else 0.0
    safe = (laps >= n_laps) and (max_abs_d <= safety_limit)
    return SimResult(
        completed_laps=laps,
        total_time_s=total_time,
        avg_lap_time_s=avg_lap_time if laps >= n_laps else float("inf"),
        max_abs_d=max_abs_d,
        avg_speed=avg_speed,
        safe=safe,
        aborted=False,
        xs=xs_a,
        ys=ys_a,
        yaws=yaws_a,
        vs=vs_a,
        ds=ds_a,
        times=times_a,
        residual_stats=_capture_residual_stats(planner),
    )


def _predict_forward(planner, x, y, yaw, v, delta, cmd_queue, n, dt):
    """Roll the measured state forward through the commands already in flight.

    Uses the planner's own bicycle model and first-order steer lag, so the
    controller only ever relies on what it already believes about the plant.
    """
    tau = float(getattr(planner, "steer_tau_s", 0.0))
    alpha_lag = dt / (tau + dt) if tau > 0.0 else 1.0
    gain = float(getattr(planner, "actuator_gain", 1.0))
    wb = float(getattr(planner, "wheelbase", 1.05))
    for k in range(n):
        if cmd_queue:
            _, steer_deg = cmd_queue[k] if k < len(cmd_queue) else cmd_queue[-1]
        else:
            steer_deg = 0.0
        delta = delta + alpha_lag * (math.radians(steer_deg) - delta)
        x += dt * v * math.cos(yaw)
        y += dt * v * math.sin(yaw)
        yaw += dt * v / wb * math.tan(gain * delta)
        # Speed is held: the window is tens of milliseconds and the planner has
        # no model of the speed loop beyond the setpoint it just asked for.
    return x, y, yaw, v, delta


def _capture_residual_stats(planner) -> dict:
    """Snapshot the planner's residual learner at end of run.

    Returns a dict suitable for JSON serialisation. Best-effort: any attribute
    that's missing on the user's local fork is silently skipped. Empty dict if
    the planner has no residual learner at all.
    """
    out: dict = {}
    r = getattr(planner, "residual", None)
    if r is None:
        return out
    for attr in (
        "mode", "use_gbm", "samples_trained", "samples_accepted_this_run",
        "outliers_dropped", "off_line_skipped", "divergence_resets",
        "revert_count", "rls_warmup_samples", "apply_min_samples_this_run",
        "model_size", "uses_batch_model", "last_active_model",
        "last_pred_clipped",
    ):
        if hasattr(r, attr):
            v = getattr(r, attr)
            try:
                out[attr] = bool(v) if isinstance(v, (bool, np.bool_)) else (
                    int(v) if isinstance(v, (int, np.integer)) else
                    float(v) if isinstance(v, (float, np.floating)) else
                    str(v)
                )
            except Exception:
                pass
    # Effective mode (gated)
    if hasattr(r, "effective_mode"):
        try:
            out["effective_mode"] = str(r.effective_mode())
        except Exception:
            pass
    # Theta norms
    try:
        import numpy as _np
        if hasattr(r, "theta_s"):
            out["theta_s_norm"] = float(_np.linalg.norm(r.theta_s))
        if hasattr(r, "theta_d"):
            out["theta_d_norm"] = float(_np.linalg.norm(r.theta_d))
    except Exception:
        pass
    # Mean error window (nominal vs residual)
    if hasattr(r, "mean_error"):
        try:
            nom_s, nom_d, res_s, res_d = r.mean_error()
            out["mean_err_nom_s"] = float(nom_s)
            out["mean_err_nom_d"] = float(nom_d)
            out["mean_err_res_s"] = float(res_s)
            out["mean_err_res_d"] = float(res_d)
        except Exception:
            pass
    # Last-prediction snapshot (helpful to see what magnitude residual is firing)
    for attr in ("last_pred_s", "last_pred_d"):
        if hasattr(r, attr):
            try:
                out[attr] = float(getattr(r, attr))
            except Exception:
                pass
    return out


if __name__ == "__main__":
    print("sim/closed_loop.py is a library. Use sim/runner.py.")
