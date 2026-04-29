"""
Headless param sweep for pure pursuit tuning.
Runs bicycle model + pathfinder (no ROS), scores by cross-track error + lap completion.

Usage:
  python3 scripts/optimize_sim.py
  python3 scripts/optimize_sim.py --line /ws/data/racing_line/line.csv --laps 3
"""
import argparse
import math
import itertools
import sys
import os
sys.path.insert(0, "/ws/src/autonomous_kart")

from autonomous_kart.nodes.localization.sim_bicycle import BicycleModel
from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder



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


def closest_idx_wrap(line, x, y, hint, window=300):
    n = len(line)
    best_i, best_d2 = hint, float("inf")
    for off in range(-50, window):
        i = (hint + off) % n
        dx = line[i][1] - x
        dy = line[i][2] - y
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i, math.sqrt(best_d2)


def pick_lookahead(line, closest, lookahead_m, v_max):
    n = len(line)
    s0 = line[closest][0]
    s_end = line[-1][0]
    s_target = s0 + lookahead_m

    if s_target > s_end:
        s_target -= s_end
        j = 0
        while j < n and line[j][0] < s_target:
            j += 1
        j = j % n
    else:
        j = closest
        steps = 0
        while line[j % n][0] < s_target and steps < n:
            j += 1
            steps += 1
        j = j % n

    tx, ty = line[j][1], line[j][2]
    spd_ref = min(1.0, line[j][5] / v_max) if v_max > 0 else 0.0
    return (tx, ty), spd_ref


def run_sim(line, params, dt=1.0/60, max_steps=100000, n_laps=2):
    """Returns (completed_laps, avg_cross_track_error, max_cross_track_error, avg_speed)."""
    wb = params["wheelbase_m"]
    v_max = params["v_max_mps"]
    steer_max = params["steer_max_deg"]

    model = BicycleModel(wb, v_max, steer_max)
    # Spawn at start
    x0, y0 = line[0][1], line[0][2]
    dx = line[1][1] - x0
    dy = line[1][2] - y0
    yaw0 = math.atan2(dy, dx)
    model.reset(x0, y0, yaw0)

    n = len(line)
    closest = 0
    cross_errors = []
    speeds = []
    laps = 0
    prev_closest = 0
    wrap_count = 0

    for step in range(max_steps):
        # Find closest point
        closest, cross_err = closest_idx_wrap(line, model.x, model.y, closest)
        cross_errors.append(cross_err)
        speeds.append(model.speed)

        # Detect lap completion (index wraps from high to low)
        if closest < n // 4 and prev_closest > 3 * n // 4:
            laps += 1
            if laps >= n_laps:
                break
        prev_closest = closest

        # Bail if way off track
        if cross_err > 30.0:
            break

        # Lookahead
        speed_pct = model.speed / v_max if v_max > 0 else 0.0
        speed_pct = max(0.0, min(1.0, speed_pct))

        if params["use_velocity_scaled_lookahead"]:
            la = model.speed * params["lookahead_time_s"]
            la = max(params["min_lookahead_m"], min(params["max_lookahead_m"], la))
        else:
            la = params["min_lookahead_m"]

        target_xy, spd_ref = pick_lookahead(line, closest, la, v_max)

        motor_pct, steer_deg = pathfinder(
            current_xy=(model.x, model.y),
            target_xy=target_xy,
            yaw_rad=model.yaw,
            speed_pct=speed_pct,
            wheelbase_m=wb,
            v_max_mps=v_max,
            steer_max_deg=steer_max,
            desired_speed_pct=spd_ref,
            use_velocity_scaled_lookahead=params["use_velocity_scaled_lookahead"],
            lookahead_time_s=params["lookahead_time_s"],
            min_lookahead_m=params["min_lookahead_m"],
            max_lookahead_m=params["max_lookahead_m"],
            use_curvature_regulation=params["use_curvature_regulation"],
            min_radius_m=params["min_radius_m"],
            min_reg_speed_pct=params["min_reg_speed_pct"],
            approach_dist_m=params["approach_dist_m"],
            min_approach_speed_pct=params["min_approach_speed_pct"],
        )

        model.step(motor_pct, steer_deg, dt)

    avg_cte = sum(cross_errors) / len(cross_errors) if cross_errors else 999
    max_cte = max(cross_errors) if cross_errors else 999
    avg_spd = sum(speeds) / len(speeds) if speeds else 0

    return laps, avg_cte, max_cte, avg_spd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--line", default="/ws/data/racing_line/line.csv")
    parser.add_argument("--laps", type=int, default=2)
    args = parser.parse_args()

    line = load_line(args.line)
    if len(line) < 10:
        print("Error: racing line too short or not found")
        return

    # Fixed params
    base = {
        "wheelbase_m": 1.05,
        "v_max_mps": 15.0,
        "steer_max_deg": 25.0,
        "use_velocity_scaled_lookahead": True,
        "use_curvature_regulation": True,
        "min_reg_speed_pct": 0.20,
        "approach_dist_m": 1.0,
        "min_approach_speed_pct": 0.05,
    }

    # Sweep these
    sweep = {
        "lookahead_time_s":  [0.6, 0.8, 1.0, 1.2],
        "min_lookahead_m":   [3.0, 5.0, 7.0],
        "max_lookahead_m":   [8.0, 10.0, 12.0, 15.0],
        "min_radius_m":      [3.0, 5.0, 8.0, 12.0],
    }

    keys = list(sweep.keys())
    combos = list(itertools.product(*[sweep[k] for k in keys]))
    print(f"Running {len(combos)} param combos x {args.laps} laps...\n")

    results = []
    for vals in combos:
        params = {**base}
        for k, v in zip(keys, vals):
            params[k] = v

        # Skip invalid combos
        if params["min_lookahead_m"] >= params["max_lookahead_m"]:
            continue

        laps, avg_cte, max_cte, avg_spd = run_sim(line, params, n_laps=args.laps)

        # Score: lower is better. Penalize incomplete laps heavily.
        lap_penalty = (args.laps - laps) * 100
        score = avg_cte + 0.3 * max_cte + lap_penalty - 0.1 * avg_spd

        results.append((score, laps, avg_cte, max_cte, avg_spd, {k: v for k, v in zip(keys, vals)}))

    results.sort(key=lambda r: r[0])

    print(f"{'Score':>8} {'Laps':>4} {'AvgCTE':>8} {'MaxCTE':>8} {'AvgSpd':>8}  Params")
    print("-" * 90)
    for score, laps, avg_cte, max_cte, avg_spd, p in results[:15]:
        pstr = "  ".join(f"{k}={v}" for k, v in p.items())
        print(f"{score:8.2f} {laps:4d} {avg_cte:8.3f} {max_cte:8.3f} {avg_spd:8.2f}  {pstr}")

    if results:
        best = results[0]
        print(f"\n Best params for pathfinder.yaml:")
        for k, v in best[5].items():
            print(f"    {k}: {v}")


if __name__ == "__main__":
    main()