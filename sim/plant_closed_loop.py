"""Closed-loop acceptance for a candidate plant.

Seeds the real planner with 0.5 s of recorded data, hands the plant the kart's
own state, and closes the loop until the driver took RC back. The four
statistics are the ones the spec gates on.
"""
from __future__ import annotations

import inspect
import math
import os
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

# AUTONOMOUS to RC transitions in run_20260913_211133_auto, seconds into the run.
ENGAGEMENTS = {1: (10.98, 17.90), 2: (42.24, 45.21)}

# Measured on the real kart over the same windows.
REAL_STATS = {
    1: {"mean_abs_cte": 0.43, "max_abs_cte": 1.06, "sd_steer": 14.3, "p90_steer_rate": 102.0},
    2: {"mean_abs_cte": 0.48, "max_abs_cte": 0.91, "sd_steer": 20.9, "p90_steer_rate": 129.0},
}

DIVERGENCE_M = 10.0


def trace_statistics(t: np.ndarray, cte: np.ndarray, steer: np.ndarray) -> Dict[str, float]:
    dt = np.diff(t)
    rate = np.abs(np.diff(steer)) / np.maximum(dt, 1e-3)
    return {
        "mean_abs_cte": float(np.abs(cte).mean()),
        "max_abs_cte": float(np.abs(cte).max()),
        "sd_steer": float(steer.std()),
        "p90_steer_rate": float(np.percentile(rate, 90)) if rate.size else 0.0,
    }


def is_stable(cte: np.ndarray) -> bool:
    """A run diverges when cross-track error leaves the track entirely."""
    return bool(np.all(np.abs(np.asarray(cte)) < DIVERGENCE_M))


def build_planner(run_dir: str, line_csv: str):
    """MPCPlanner rebuilt from the run's own params_live.yaml.

    The run records the parameters the node was actually using, which is the
    only way to reproduce a bag: the committed YAML and the running node have
    disagreed before.
    """
    import yaml

    from sim.closed_loop import _install_ros_stubs
    _install_ros_stubs()

    from autonomous_kart.nodes.pathfinder.planners.base import KartConstants
    from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner

    def flat(d, prefix=""):
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                out.update(flat(v, prefix + k + "."))
            else:
                out[prefix + k] = v
        return out

    # params_live.yaml is what the node was really running, but the recorder
    # wrote a 0-byte one for run_20260913_195301_auto, so the params/ snapshot
    # is the fallback. It spells the node key without a leading slash, flattens
    # the mpc block into dotted keys, and keeps the kart constants under the
    # `/**` wildcard, so each shape has to be read on its own terms.
    params = {}
    for path, keys in ((f"{run_dir}/params_live.yaml", ("/pathfinder_node",)),
                       (f"{run_dir}/params/pathfinder.yaml",
                        ("/pathfinder_node", "pathfinder_node", "/**"))):
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        for key in keys:
            block = (doc.get(key) or {}).get("ros__parameters") or {}
            for k, v in block.items():
                params.setdefault(k, v)
    if not params:
        raise ValueError(f"no pathfinder params under {run_dir}")
    mpc = flat(params.get("mpc", {}))
    for k, v in params.items():
        if k.startswith("mpc."):
            mpc.setdefault(k[len("mpc."):], v)
    mpc["residual.cache_enabled"] = False
    # These runs predate `residual.model_size`, and were recorded with the
    # batch model off. Pin the recursive learner so the replay matches.
    mpc["residual.model_size"] = "s"
    kart = KartConstants(
        v_max_mps=float(params["v_max_mps"]), wheelbase_m=float(params["wheelbase_m"]),
        steer_max_deg=float(params["steer_max_deg"]),
        steer_rate_max_degps=float(params["steer_rate_max_degps"]),
        a_max_mps2=float(params["a_max_mps2"]), a_min_mps2=float(params["a_min_mps2"]),
        a_lat_max_mps2=float(params["a_lat_max_mps2"]))

    rows = []
    for line in open(line_csv):
        try:
            rows.append(tuple(float(x) for x in line.strip().split(",")))
        except ValueError:
            continue

    class _Param:
        def __init__(self, value):
            self.value = value

    class _Node:
        def get_parameter(self, name):
            return _Param(60.0 if name == "system_frequency" else 0.0)

        def create_publisher(self, *_a, **_k):
            class _Pub:
                def publish(self, _msg):
                    pass
            return _Pub()

    return MPCPlanner(mpc, kart, rows, logger=None, node=_Node())


# Indices into the 88-float /mpc/status payload (see MPCPlanner._publish_status).
_STATUS_V, _STATUS_STEER_DEG = 6, 8
_STATUS_X, _STATUS_Y, _STATUS_YAW = 31, 32, 33


def _read_mpc_status(run_dir: str):
    """(t, x, y, yaw, v, steer) from /mpc/status, across mcap splits.

    t is seconds since the run's first /mpc/status message, keyed on
    `msg.log_time` (the recorder's receipt time — /mpc/status has no header).
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    from sim.sensor_noise import run_mcaps

    rows = []
    for path in run_mcaps(run_dir):
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, _channel, msg, ros in reader.iter_decoded_messages(topics=["/mpc/status"]):
                rows.append((msg.log_time, ros.data))
    if not rows:
        raise ValueError(f"no /mpc/status messages under {run_dir}")
    rows.sort(key=lambda r: r[0])
    t0 = rows[0][0]
    t = np.asarray([(log_time - t0) * 1e-9 for log_time, _ in rows])
    data = np.asarray([list(d) for _, d in rows], dtype=float)
    return (t, data[:, _STATUS_X], data[:, _STATUS_Y], data[:, _STATUS_YAW],
            data[:, _STATUS_V], data[:, _STATUS_STEER_DEG])


def seed_planner_index(planner, x: float, y: float, index=None) -> int:
    """Point a fresh planner's Frenet index at the kart.

    MPCPlanner walks its index through a local window, so it is only ever a
    few points from where it already was; built fresh it starts at 0 and can
    never reach the kart. A global nearest-point search is not enough either,
    because line6 runs back alongside itself: over run_20260913_203929 the
    nearest point alternates between index 15 and index 226 on successive
    ticks, and the planner that drove stayed on the 15 branch. Pass `index`
    (the recorded closest_idx) to put it on the branch the kart was really on.
    """
    if index is None:
        lx = planner._static_arrays["x"]
        ly = planner._static_arrays["y"]
        index = int(np.argmin((lx - x) ** 2 + (ly - y) ** 2))
    idx = int(index) % len(planner._static_arrays["x"])
    planner.closest_idx = idx
    planner.static_closest_idx = idx
    return idx


def _yaw_rate(t: np.ndarray, yaw: np.ndarray, i: int, half: int = 3) -> float:
    """Least-squares slope of the unwrapped yaw over +-`half` ticks.

    A single backward difference on a 22 Hz EKF heading is mostly noise; the
    short fit is what makes the seeded psi_dot usable.
    """
    lo, hi = max(0, i - half), min(len(t), i + half + 1)
    if hi - lo < 2:
        return 0.0
    return float(np.polyfit(t[lo:hi] - t[i], np.unwrap(yaw[lo:hi]), 1)[0])


# How far back the wheel angle is integrated before a handoff. At 33.7 deg/s
# the wheel needs about a second to cross its range, so a shorter lead-in
# leaves it somewhere it never was.
WHEEL_SEED_S = 2.0


def _handoff_history(t, x, y, yaw, v, i0, steer=None):
    """The HISTORY rows the plant is handed at the switch, oldest first.

    Throttle setpoint is not recorded on /mpc/status, and at steady state the
    command equals the speed, so the recorded speed stands in for it -- the
    same convention LearnedPlant.reset uses for a synthetic window.

    `steer` is the recorded command stream. The plant's steering feature is
    the wheel angle, so the command is integrated through the actuator's rate
    limit from `WHEEL_SEED_S` before the handoff; without it the sim starts
    centred while the kart was hard over. With no stream the wheel is centred,
    which is the old behaviour.
    """
    from sim.plant_dataset import HISTORY, slew_limit
    wheel = np.zeros(len(t))
    if steer is not None:
        lead = max(0, int(np.searchsorted(t, t[i0] - WHEEL_SEED_S)))
        hz = 1.0 / max(float(np.median(np.diff(t[lead:i0 + 1]))), 1e-6) \
            if i0 > lead else 60.0
        seg = slew_limit(np.asarray(steer)[lead:i0 + 1], hz,
                         start=float(steer[lead]))
        wheel[lead:i0 + 1] = seg
    rows = []
    for j in range(i0 - HISTORY + 1, i0 + 1):
        k = max(0, j)
        rows.append((float(v[k]), _yaw_rate(t, yaw, k), float(wheel[k]),
                     float(v[k])))
    return rows


def _closed_loop_trace(planner, t: np.ndarray, x: np.ndarray, y: np.ndarray,
                       yaw: np.ndarray, v: np.ndarray, window: Tuple[float, float],
                       plant_factory: Callable, warmup_s: float,
                       steer: Optional[np.ndarray] = None):
    """Warm `planner` on recorded ticks in `window`, hand off to a fresh plant
    at `warmup_s`, then close the loop to the end of the window.

    Pure of any bag I/O so the handoff logic can be driven by a synthetic
    trace in tests. Returns (t, cte, steer) for the simulated segment, cte
    measured with `planner._frenet` against the static racing line.
    """
    from sim.closed_loop import _install_ros_stubs
    _install_ros_stubs()
    from autonomous_kart.nodes.pathfinder.planners.base import PlannerInputs

    start_s, end_s = window
    handoff_t = start_s + warmup_s
    in_window = np.flatnonzero((t >= start_s) & (t <= end_s))
    if in_window.size == 0:
        raise ValueError(f"no ticks in window {window}")
    warmup = [int(i) for i in in_window if t[i] < handoff_t]
    closed = [int(i) for i in in_window if t[i] >= handoff_t]
    if not closed:
        raise ValueError(f"window {window} shorter than warmup_s={warmup_s}")

    if warmup:
        seed_planner_index(planner, float(x[warmup[0]]), float(y[warmup[0]]))
    for i in warmup:
        planner.plan(PlannerInputs(
            pose_xy=(float(x[i]), float(y[i])), yaw_rad=float(yaw[i]),
            speed_mps=float(v[i]), track_angles=None, now_ns=int(t[i] * 1e9)))

    i0 = closed[0]
    plant = plant_factory()
    # The whole recorded window goes through reset, not just the speed: a
    # learned plant reads HISTORY ticks, and psi_dot is plant state. Seeding
    # only `v` leaves a kart handed over mid-corner at 0.88 rad/s being told
    # it is going straight, which is what turned three recorded off-track
    # excursions back into clean laps in sim.
    kwargs = {"v": float(v[i0])}
    if "history" in inspect.signature(plant.reset).parameters:
        kwargs["history"] = _handoff_history(t, x, y, yaw, v, i0, steer=steer)
    plant.reset(float(x[i0]), float(y[i0]), float(yaw[i0]), **kwargs)

    lx, ly = planner._static_arrays["x"], planner._static_arrays["y"]
    hint = int(np.argmin((lx - plant.x) ** 2 + (ly - plant.y) ** 2))

    ts: List[float] = []
    ctes: List[float] = []
    steers: List[float] = []
    for k, i in enumerate(closed):
        inputs = PlannerInputs(pose_xy=(plant.x, plant.y), yaw_rad=plant.yaw,
                               speed_mps=plant.speed, track_angles=None,
                               now_ns=int(t[i] * 1e9))
        throttle_mps, steering_deg = planner.plan(inputs)

        planner._set_active_line(planner._static_arrays)
        _, d, hint, _, _ = planner._frenet(plant.x, plant.y, hint)
        ts.append(float(t[i]))
        ctes.append(float(d))
        steers.append(float(steering_deg))

        if k + 1 < len(closed):
            dt = float(t[closed[k + 1]] - t[i])
            plant.step(throttle_mps, steering_deg, dt)

    return np.asarray(ts), np.asarray(ctes), np.asarray(steers)


def engagement_stats(run_dir: str, window: Tuple[float, float],
                     plant_factory: Callable, line_csv: str,
                     warmup_s: float = 0.5) -> Dict[str, float]:
    """Closed-loop acceptance stats for one AUTONOMOUS engagement.

    Rebuilds the real planner from the run's own params, replays the first
    `warmup_s` of the engagement window through it so its Frenet index and
    warm-start match what the kart actually saw, then hands off to
    `plant_factory()` at the kart's own recorded pose/speed and closes the
    loop against the planner for the rest of the window.
    """
    planner = build_planner(run_dir, line_csv)
    t, x, y, yaw, v, steer = _read_mpc_status(run_dir)
    ts, ctes, steers = _closed_loop_trace(
        planner, t, x, y, yaw, v, window, plant_factory, warmup_s, steer=steer)
    return trace_statistics(ts, ctes, steers)


def build_factories(weights_path: str, meta_path: str) -> Dict[str, Callable]:
    """Plant factories for the closed-loop table: the gray-box always, the
    network when its weights are on disk. Training is deliberately deferred,
    so a missing sim/model/plant_nn.pt is not an error here."""
    from sim.data_sim import DataSim
    factories: Dict[str, Callable] = {"graybox": DataSim}
    if os.path.isfile(weights_path):
        from sim.learned_plant import LearnedPlant
        factories["learned_nn"] = lambda: LearnedPlant.from_files(weights_path, meta_path)
    return factories


_STAT_COLUMNS = ("mean_abs_cte", "max_abs_cte", "sd_steer", "p90_steer_rate")


def closed_loop_table(run_dir: str, line_csv: str, factories: Dict[str, Callable],
                      warmup_s: float = 0.5) -> str:
    """The spec's second acceptance table: every candidate on every engagement,
    against the statistics the real kart produced over the same windows."""
    lines = [f"{'engagement':>11}{'plant':>13}{'mean |cte|':>12}{'max |cte|':>11}"
             f"{'sd steer':>10}{'p90 rate':>10}"]

    def row(engagement, name, stats):
        return (f"{engagement:>11}{name:>13}"
                + "".join(f"{stats[c]:>{w}.3f}" for c, w in
                          zip(_STAT_COLUMNS, (12, 11, 10, 10))))

    for engagement, window in sorted(ENGAGEMENTS.items()):
        lines.append(row(engagement, "real", REAL_STATS[engagement]))
        for name, factory in factories.items():
            stats = engagement_stats(run_dir, window, factory, line_csv, warmup_s)
            lines.append(row(engagement, name, stats))
    return "\n".join(lines)


def stability_check(line_csv: str, sim_backend: str = "learned",
                    seconds: float = 120.0, seeds: int = 5) -> List[bool]:
    """Run the planner on the racing line from a standing start, per seed."""
    from sim.closed_loop import DEFAULT_MPC, simulate
    out = []
    rows = []
    for line in open(line_csv):
        try:
            rows.append(tuple(float(x) for x in line.strip().split(",")))
        except ValueError:
            continue
    for seed in range(seeds):
        result = simulate(rows, DEFAULT_MPC, sim_backend=sim_backend, rng_seed=seed,
                          n_laps=99, max_steps=int(seconds * 60))
        out.append(is_stable(result.ds) and not result.aborted)
    return out


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", default="new_mpc_bags/20260913/run_20260913_211133_auto")
    ap.add_argument("--line", default="data/racing_line/line6.csv")
    ap.add_argument("--weights", default="sim/model/plant_nn.pt")
    ap.add_argument("--meta", default="sim/model/plant_nn.json")
    ap.add_argument("--warmup-s", type=float, default=0.5)
    ap.add_argument("--stability", action="store_true",
                    help="also run the 120 s standing-start check per seed")
    ap.add_argument("--stability-backend", default="learned")
    args = ap.parse_args()

    factories = build_factories(args.weights, args.meta)
    if "learned_nn" not in factories:
        print(f"learned_nn weights not found at {args.weights} "
              f"(training deliberately deferred) -- reporting the gray-box only.")
    print("=== closed-loop acceptance, run "
          f"{os.path.basename(args.run_dir.rstrip('/'))} ===")
    print(closed_loop_table(args.run_dir, args.line, factories, args.warmup_s))

    if args.stability:
        ok = stability_check(args.line, sim_backend=args.stability_backend)
        print(f"\nstability ({args.stability_backend}): "
              f"{sum(ok)}/{len(ok)} seeds held the line")


if __name__ == "__main__":
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
