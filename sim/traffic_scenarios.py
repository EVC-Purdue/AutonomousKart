"""Closed-loop opponent scenarios for the MPC's avoidance / passing.

Runs each scenario with the shipped pathfinder.yaml MPC params on line1:
  blind   planner gets no opponents (today's behaviour)
  truth   planner gets perfect opponent states
  tracker planner gets them through FOV + range noise + KartTracker + latency

    python3 sim/traffic_scenarios.py                    # all scenarios, bicycle plant
    python3 sim/traffic_scenarios.py --sim datasim --only slow_kart brake_check
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from sim.closed_loop import simulate  # noqa: E402  (installs ROS stubs + package path)
from sim.runner import load_line  # noqa: E402
from sim.traffic import ScriptedKart, Traffic  # noqa: E402
from autonomous_kart.nodes.pathfinder.opponents import TrackBounds  # noqa: E402


def _step(t0, v0, v1):
    return lambda t: v0 if t < t0 else v1


def _ramp(t0, t1, a, b):
    return lambda t: a + (b - a) * min(1.0, max(0.0, (t - t0) / (t1 - t0)))


# name -> (karts factory, seconds, expectation)
SCENARIOS = {
    "slow_kart": (lambda: [ScriptedKart(1, 30.0, 3.5)], 40.0, "pass, no contact"),
    "stalled_kart": (lambda: [ScriptedKart(1, 60.0, 0.0)], 40.0, "pass or stop, no contact"),
    "brake_check": (lambda: [ScriptedKart(1, 15.0, _step(10.0, 6.0, 2.0))], 40.0, "brake, then pass; no contact"),
    "side_by_side": (lambda: [ScriptedKart(1, 30.0, 4.0, 1.1), ScriptedKart(2, 30.0, 4.0, -1.1)], 40.0,
                     "no room: follow, no contact"),
    "train": (lambda: [ScriptedKart(1, 25.0, 4.5), ScriptedKart(2, 45.0, 4.5, 0.6),
                       ScriptedKart(3, 70.0, 4.0, -0.5)], 60.0, "pass where there is room, no contact"),
    "cut_in": (lambda: [ScriptedKart(1, 12.0, 5.0, _ramp(4.0, 6.0, 1.4, 0.0))], 40.0,
               "yield to the merging kart, no contact"),
}


def load_params(kml: str) -> dict:
    y = yaml.safe_load(open(os.path.join(REPO, "src/autonomous_kart/autonomous_kart/params/pathfinder.yaml")))
    p = {k[4:]: v for k, v in y["pathfinder_node"]["ros__parameters"].items() if k.startswith("mpc.")}
    p["residual.cache_enabled"] = False
    # The yaml points at the kart's /ws/data path; use the repo copy here.
    # (A missing KML silently means symmetric bounds = no room to pass.)
    p["opp.track_kml"] = kml
    return p


def offtrack_m(res, line, kml) -> float:
    """Worst excursion of the ego kart's edge past the real track edge."""
    from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import TrackFrame
    fr = TrackFrame(line)
    b = TrackBounds.from_kml(kml, fr, 40.4380475, -86.9442826)
    if res.xs.size == 0:
        return 0.0
    sd, _ = fr.to_frenet(np.column_stack([res.xs, res.ys]), np.repeat(np.eye(2)[None], res.xs.size, 0))
    left, right = b.room(sd[:, 0])
    over = np.maximum(sd[:, 1] + 0.5 - left, -sd[:, 1] + 0.5 - right)
    return float(max(0.0, over.max()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--line", default=os.path.join(REPO, "data/racing_line/line1.csv"))
    ap.add_argument("--kml", default=os.path.join(REPO, "data/track/purdue_grand_prix.kml"))
    ap.add_argument("--sim", choices=["bicycle", "datasim"], default="bicycle")
    ap.add_argument("--modes", nargs="+", default=["blind", "truth", "tracker"])
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    line = load_line(args.line)
    params = load_params(args.kml)
    dt = 1.0 / 60.0
    print(f"{'scenario':13s} {'mode':8s} {'contact':>7s} {'min gap':>8s} {'passed':>7s} "
          f"{'off-trk':>7s} {'progress':>8s}  expectation")
    for name, (make, secs, expect) in SCENARIOS.items():
        if args.only and name not in args.only:
            continue
        for mode in args.modes:
            karts = make()
            traffic = Traffic(line, karts, perception="truth" if mode == "blind" else mode, rng_seed=args.seed)
            if mode == "blind":
                traffic.observe = lambda *a, **k: ()  # planner sees nothing
            t0 = time.time()
            res = simulate(line, dict(params), sim_backend=args.sim, datasim_use_mlp=False, n_laps=1,
                           dt=dt, max_steps=int(secs / dt), steering_gain=params.get("steering_gain", 1.0),
                           rng_seed=args.seed, traffic=traffic)
            st = res.traffic_stats or {}
            prog = float(np.sum(np.hypot(np.diff(res.xs), np.diff(res.ys)))) if res.xs.size > 1 else 0.0
            contact = "YES" if st.get("collision_steps", 0) else "no"
            print(f"{name:13s} {mode:8s} {contact:>7s} {st.get('min_gap_m', math.inf):8.2f} "
                  f"{len(st.get('passed_ids', [])):>3d}/{len(karts):<3d} {offtrack_m(res, line, args.kml):7.2f} "
                  f"{prog:7.0f}m  {expect if mode != 'blind' else ''}"
                  + ("  [ABORTED]" if res.aborted else "") + f"  ({time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main()
