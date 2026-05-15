"""
Headless MPC param sweep. Runs MPCPlanner against the kinematic bicycle in a
tight Python loop (no ROS, no launch overhead) and scores each combo by laps
completed, lateral error, and average speed.

Usage:
  python3 scripts/sweep_mpc.py
  python3 scripts/sweep_mpc.py --line /ws/data/racing_line/line14.csv --laps 2

The sweep grid lives at the bottom of `main()`. Edit it for whatever subset of
weights you want to explore — combos run linearly so 200+ entries is fine
(~3-5 s/combo with the defaults below).

Notes:
  * The bicycle is the same one in sim_bicycle.BicycleModel — actuator lag,
    no noise. Deterministic given the sweep seed.
  * Throttle command is interpreted as target velocity (matches the actuator
    in both sim and e_comms).
  * Lap detection wraps around the racing line's closed s coordinate.
"""
import argparse
import itertools
import math
import os
import sys
import time
import types

# Stub ROS deps so MPCPlanner imports cleanly without rclpy / std_msgs on
# PYTHONPATH. The planner only uses Float32MultiArray to publish telemetry,
# which the sweep doesn't care about.
_std = types.ModuleType("std_msgs")
_std.msg = types.ModuleType("std_msgs.msg")


class _MultiArrayStub:
    def __init__(self, data=None):
        self.data = data or []


_std.msg.Float32MultiArray = _MultiArrayStub
sys.modules.setdefault("std_msgs", _std)
sys.modules.setdefault("std_msgs.msg", _std.msg)

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "src", "autonomous_kart"))

import numpy as np

from autonomous_kart.nodes.localization.sim_bicycle import BicycleModel
from autonomous_kart.nodes.pathfinder.planners.base import (
    KartConstants, PlannerInputs,
)
from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner


def load_line(path):
    line = []
    with open(path, "r") as f:
        for row in f:
            parts = row.strip().split(",")
            if len(parts) >= 6:
                try:
                    line.append(tuple(float(x) for x in parts))
                except ValueError:
                    continue
    return line


def lateral_error(line_xy, x, y):
    """Min Euclidean distance from (x,y) to any racing-line waypoint."""
    dx = line_xy[:, 0] - x
    dy = line_xy[:, 1] - y
    return float(np.sqrt(np.min(dx * dx + dy * dy)))


def closest_idx(line_xy, x, y):
    dx = line_xy[:, 0] - x
    dy = line_xy[:, 1] - y
    return int(np.argmin(dx * dx + dy * dy))


def run_combo(line, weights, base, max_sim_s=120.0, laps_target=2):
    """Run one param combo. Returns dict of metrics."""
    # Compose param dict for MPCPlanner (mpc.* keys, residual.* keys, etc.)
    params = dict(base)
    params.update(weights)

    kart = KartConstants(
        v_max_mps=params["v_max_mps"],
        wheelbase_m=params["wheelbase_m"],
        steer_max_deg=params["steer_max_deg"],
        steer_rate_max_degps=params["steer_rate_max_degps"],
        a_max_mps2=params["a_max_mps2"],
        a_min_mps2=params["a_min_mps2"],
        a_lat_max_mps2=params["a_lat_max_mps2"],
    )

    try:
        planner = MPCPlanner(params, kart, line, logger=None, node=None)
    except Exception as e:
        return {"error": f"init failed: {e}"}

    # Spawn at start of racing line, heading along the first segment.
    x0 = float(line[0][1])
    y0 = float(line[0][2])
    yaw0 = math.atan2(float(line[1][2]) - y0, float(line[1][1]) - x0)
    bike = BicycleModel(kart.wheelbase_m, kart.v_max_mps, kart.steer_max_deg)
    bike.reset(x0, y0, yaw0)

    dt = 1.0 / 60.0
    line_xy = np.array([(r[1], r[2]) for r in line])
    n = len(line)
    quarter = n // 4

    laps_done = 0
    last_q = 0  # 0..3, which quarter of the line we're in
    cte_sum = 0.0
    cte_max = 0.0
    cte_n = 0
    speed_sum = 0.0
    stuck_ticks = 0  # consecutive zero-throttle ticks
    planner_errors = 0
    t_start = time.perf_counter()

    for step_idx in range(int(max_sim_s / dt)):
        sim_t = step_idx * dt
        inputs = PlannerInputs(
            pose_xy=(bike.x, bike.y),
            yaw_rad=bike.yaw,
            speed_mps=bike.speed,
            track_angles=None,
            now_ns=int(sim_t * 1e9),
        )
        try:
            proposed = planner.plan(inputs)
        except Exception:
            planner_errors += 1
            proposed = (0.0, 0.0)

        if proposed is None:
            throttle, steer = 0.0, 0.0
        else:
            throttle, steer = proposed

        bike.step(throttle, steer, dt)

        cte = lateral_error(line_xy, bike.x, bike.y)
        cte_sum += cte
        cte_max = max(cte_max, cte)
        cte_n += 1
        speed_sum += bike.speed

        # Lap counter via quartile transitions
        cidx = closest_idx(line_xy, bike.x, bike.y)
        q = (cidx * 4) // n  # 0..3
        if last_q == 3 and q == 0:
            laps_done += 1
            if laps_done >= laps_target:
                break
        last_q = q

        # Stuck detection: 3+ seconds of near-zero speed = abort
        if bike.speed < 0.1 and throttle < 1.0:
            stuck_ticks += 1
            if stuck_ticks > int(3.0 / dt):
                break
        else:
            stuck_ticks = 0

        # Diverged-off-line: abort if 25 m+ off line for sanity
        if cte > 25.0:
            break

    wall = time.perf_counter() - t_start
    return {
        "laps": laps_done,
        "avg_cte": cte_sum / max(1, cte_n),
        "max_cte": cte_max,
        "avg_speed": speed_sum / max(1, cte_n),
        "planner_errors": planner_errors,
        "sim_s": cte_n * dt,
        "wall_s": wall,
    }


def score(metrics, laps_target):
    """Lower is better. Hard-penalise incomplete laps, then sum CTE costs."""
    if "error" in metrics:
        return 1e9
    lap_penalty = max(0, laps_target - metrics["laps"]) * 200.0
    return (
        lap_penalty
        + metrics["avg_cte"]
        + 0.3 * metrics["max_cte"]
        - 0.05 * metrics["avg_speed"]
        + 50.0 * metrics["planner_errors"]
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--line", default=os.path.join(_REPO, "data/racing_line/line14.csv"))
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--max-sim-s", type=float, default=180.0)
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    line = load_line(args.line)
    if len(line) < 10:
        print(f"Error: racing line too short or not found at {args.line}")
        return

    # Kart-wide + structural params copied from pathfinder.yaml. These do NOT
    # vary in the sweep. Adjust to match your yaml if you've tweaked them.
    base = {
        # KartConstants
        "v_max_mps": 20.0,
        "wheelbase_m": 1.05,
        "steer_max_deg": 25.0,
        "steer_rate_max_degps": 180.0,
        "a_max_mps2": 2.0,
        "a_min_mps2": -3.0,
        "a_lat_max_mps2": 4.0,
        # MPC sampling / horizon
        "horizon_steps": 20,
        "dt_s": 0.05,
        "num_samples": 48,
        "steer_sigma_deg": 6.0,
        "accel_sigma_mps2": 1.0,
        "proj_window_back": 5,
        "proj_window_fwd": 60,
        "frenet_v_window_s": 0.3,
        "frenet_resync_m": 5.0,
        # Corridor
        "track_half_width_m": 2.0,
        "safety_margin_m": 0.5,
        "kart_half_width_m": 0.5025,
        # Failsafe / feasibility
        "max_consecutive_solver_failures": 3,
        "infeasible_cost": 1.0e9,
        "feasibility_threshold": 5.0e8,
        # Rejoin
        "rejoin_cte_activate": 2.5,
        "rejoin_cte_deactivate": 1.0,
        "rejoin_merge_lookahead_m": 15.0,
        "rejoin_min_turning_radius": 3.0,
        # Edge barrier (geometry; weight in sweep)
        "edge_inner_frac": 0.5,
        # Residual learner (off-ish; "shadow" mode = passive)
        "residual.mode": "shadow",
        "residual.target_horizon_s": 0.5,
        "residual.forgetting_factor": 0.99,
        "residual.initial_cov": 1000.0,
        "residual.min_train_speed_mps": 0.5,
        "residual.error_window": 200,
        # Fixed cost weights (only sweep keys are the interesting ones)
        "w_heading": 30.0,
        "w_delta": 5.0,
        "w_delta_rate": 50.0,
        "w_accel": 2.0,
        "w_boundary": 1000.0,
        "w_progress": 1.0,
        "w_terminal_d": 200.0,
        "w_terminal_heading": 100.0,
    }

    # The sweep grid. Edit me. Cost-weight tuning is the high-value lever;
    # don't sweep too many axes at once or you'll wait a long time.
    sweep = {
        "w_d":              [100.0, 200.0, 300.0, 500.0],
        "w_edge":           [100.0, 300.0, 500.0],
        "w_speed":          [0.5, 1.0, 5.0],
        "w_a_lat":          [25.0, 50.0, 100.0],
        "target_speed_pct": [0.7, 1.0],
    }

    keys = list(sweep.keys())
    combos = list(itertools.product(*[sweep[k] for k in keys]))
    print(f"Running {len(combos)} param combos × up to {args.laps} laps "
          f"each (max {args.max_sim_s:.0f}s sim time per combo)...\n")

    rows = []
    t0 = time.perf_counter()
    for i, vals in enumerate(combos):
        weights = {k: v for k, v in zip(keys, vals)}
        m = run_combo(line, weights, base,
                      max_sim_s=args.max_sim_s, laps_target=args.laps)
        s = score(m, args.laps)
        rows.append((s, weights, m))
        # Live progress; one line per combo.
        if "error" in m:
            print(f"[{i + 1:>3}/{len(combos)}] score=inf      ({m['error']})")
        else:
            print(f"[{i + 1:>3}/{len(combos)}] score={s:7.2f}  "
                  f"laps={m['laps']}/{args.laps}  "
                  f"avgCTE={m['avg_cte']:5.2f}m  "
                  f"maxCTE={m['max_cte']:5.2f}m  "
                  f"avgV={m['avg_speed']:5.2f}m/s  "
                  f"({m['wall_s']:.1f}s wall)  "
                  f"{weights}")

    rows.sort(key=lambda r: r[0])
    elapsed = time.perf_counter() - t0
    print(f"\nDone in {elapsed:.1f}s wall. Top {args.top}:\n")
    header = f"{'rank':>4} {'score':>8} {'laps':>5} {'avgCTE':>7} {'maxCTE':>7} {'avgV':>6}  weights"
    print(header)
    print("-" * len(header))
    for rank, (s, w, m) in enumerate(rows[: args.top], start=1):
        if "error" in m:
            continue
        wstr = "  ".join(f"{k}={v}" for k, v in w.items())
        print(f"{rank:>4} {s:8.2f} {m['laps']:>2}/{args.laps:<2} "
              f"{m['avg_cte']:7.3f} {m['max_cte']:7.3f} {m['avg_speed']:6.2f}  "
              f"{wstr}")

    if rows and "error" not in rows[0][2]:
        print("\nBest yaml stanza (paste into params/pathfinder.yaml):")
        for k, v in rows[0][1].items():
            print(f"    mpc.{k}: {v}")


if __name__ == "__main__":
    main()
