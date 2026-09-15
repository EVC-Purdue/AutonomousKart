"""Closed-loop benchmark: score an MPC config against a perfect solver.

Three scenarios, each a 3 s window from bag 20260519, starting at the last point
where the kart was within 0.5 m of the line before it left. Each controller is
warmed up open loop on 20 s of logged poses so its warm start arrives in a
realistic state, then drives the plant through the window. Speed comes from the
log, so only steering differs.

The reference is `perfect`: a solver that sweeps the first action and takes the
argmin of MPC's own cost every tick. It is the best any first-action policy
could do under this cost function, so a config's score is

    score = mean_cost(config) / mean_cost(perfect)

averaged over scenarios. 1.00 means the sampler is extracting everything the
objective has to offer. The shipped config sits well above that.

RUN IT OVER SEEDS. The sampler's output at these corners is close to noise
(the first steering action explains under 4% of the cost variance among
candidates), so a single seed says nothing about a config.

PLANT CAVEAT. `steer_gain` maps cmd_drive degrees to wheel angle. It is
identifiable only where the logged commands are large, which is the runway, not
the corner: fitted on the 05/19 lead-in it lands at 0.116-0.16. Through the
corner the logged commands were near zero, so every alternative controller here
is extrapolating to command levels the bag has no data for. Sweep `--gain` and
trust rankings that survive it, not absolute metres.

CLI:
    python sim/mpc_bench.py                                  # shipped config
    python sim/mpc_bench.py --set steer_sigma_deg=3.5
    python sim/mpc_bench.py --sweep steer_sigma_deg=2.5,3,3.5,4
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# Runnable as a script (`python sim/mpc_bench.py`) as well as importable.
if __package__ in (None, ""):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.closed_loop import _install_ros_stubs

_install_ros_stubs()

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

from autonomous_kart.nodes.pathfinder.planners.base import (  # noqa: E402
    KartConstants, PlannerInputs,
)
from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner  # noqa: E402
from autonomous_kart.nodes.pathfinder.planners.pure_pursuit import (  # noqa: E402
    PurePursuitPlanner,
)
from sim.apex_repro import load_baseline, load_line  # noqa: E402

# Plant: kinematic bicycle, cmd_drive deg -> wheel angle via `gain`. 0.116 is the
# 05/19 lead-in fit; 0.30 is the 05/20-21 kart. Rate limit from the yaml.
DEFAULT_GAIN = 0.116
WHEELBASE = 1.05
STEER_RATE_MAX_DEGPS = 180.0
WARMUP_S = 20.0
# Sweeping the first action for the `perfect` reference and for cost readout.
SWEEP_DEG = np.linspace(-60.0, 60.0, 61)
# Noise is zeroed inside the cost probe, so every rollout is identical and a
# small K gives the same number far more cheaply.
PROBE_K = 16
DEFAULT_SEEDS = (0, 1, 2, 3)


@dataclass(frozen=True)
class Scenario:
    bag: str
    label: str

    @property
    def path(self) -> str:
        return os.path.join(HERE, "fixtures", f"bench_{self.bag}.npz")


SCENARIOS = (
    Scenario("20260519_214619", "214619  apex"),
    Scenario("20260519_220545", "220545  long corner"),
    Scenario("20260519_211017", "211017  mid-run drift"),
)


@dataclass
class Run:
    """One controller, one scenario, one seed."""
    xs: np.ndarray
    ys: np.ndarray
    cmds: np.ndarray
    costs: np.ndarray
    cte: np.ndarray

    @property
    def mean_cost(self) -> float:
        return float(np.mean(self.costs))

    @property
    def max_cte(self) -> float:
        return float(np.max(self.cte))


@dataclass
class BenchResult:
    label: str
    per_scenario: Dict[str, List[Run]] = field(default_factory=dict)
    perfect: Dict[str, Run] = field(default_factory=dict)

    def ratios(self) -> Dict[str, float]:
        out = {}
        for name, runs in self.per_scenario.items():
            ref = self.perfect[name].mean_cost
            out[name] = float(np.mean([r.mean_cost for r in runs]) / ref)
        return out

    @property
    def score(self) -> float:
        """Mean cost ratio to perfect, averaged over scenarios. 1.00 is perfect."""
        return float(np.mean(list(self.ratios().values())))

    @property
    def mean_max_cte(self) -> float:
        return float(np.mean([np.mean([r.max_cte for r in runs])
                              for runs in self.per_scenario.values()]))

    def summary(self) -> str:
        lines = [f"{'scenario':<24}{'cost ratio':>12}{'max|CTE| m':>12}"
                 f"{'perfect CTE':>13}{'seed spread':>13}"]
        rat = self.ratios()
        for sc in SCENARIOS:
            runs = self.per_scenario.get(sc.label)
            if not runs:
                continue
            ctes = [r.max_cte for r in runs]
            lines.append(f"{sc.label:<24}{rat[sc.label]:12.2f}{np.mean(ctes):12.2f}"
                         f"{self.perfect[sc.label].max_cte:13.2f}"
                         f"{np.std(ctes):13.2f}")
        lines.append("")
        lines.append(f"{'SCORE (1.00 = perfect)':<24}{self.score:12.2f}"
                     f"{self.mean_max_cte:12.2f}")
        return "\n".join(lines)


def _load(sc: Scenario) -> Dict[str, np.ndarray]:
    z = np.load(sc.path)
    return {k: z[k] for k in z.files}


def _cost_of(pl: MPCPlanner, accel_mean, delta_prev, x, y, yaw, v, deg) -> float:
    """MPC's own cost of holding `deg` as the first action, noise off.

    `_solve` draws from `pl._rng` even at sigma 0, so RNG state is restored —
    otherwise measuring would perturb the run being measured.
    """
    keep = (pl.steer_sigma, pl.accel_sigma, pl.u_mean.copy(), pl.delta_prev, pl.K)
    rng_state = pl._rng.bit_generator.state
    pl.steer_sigma = 0.0
    pl.accel_sigma = 0.0
    pl.K = PROBE_K
    try:
        pl.u_mean[0, :] = math.radians(deg)
        pl.u_mean[1, :] = accel_mean
        pl.delta_prev = delta_prev
        _, cost, _, _, _ = pl._solve(x, y, yaw, max(v, 1e-3), pl.closest_idx,
                                     pl.target_speed, pl.v_max)
    finally:
        (pl.steer_sigma, pl.accel_sigma, pl.u_mean, pl.delta_prev, pl.K) = keep
        pl._rng.bit_generator.state = rng_state
    return float(cost)


def drive(sc: Scenario, kind: str, mpc_params: dict, pp_params: dict,
          kart: KartConstants, line, seed: int, gain: float,
          speed_scale: float = 1.0) -> Run:
    fx = _load(sc)
    t = fx["t"]
    t0, t1 = float(fx["t_start"]), float(fx["t_end"])
    i0 = int(np.argmin(np.abs(t - t0)))
    i1 = int(np.argmin(np.abs(t - t1)))
    la = np.asarray(line)
    lx, ly = la[:, 1], la[:, 2]

    pl = MPCPlanner(dict(mpc_params), kart, line, logger=None, node=None)
    pl._rng = np.random.default_rng(seed)
    pp = PurePursuitPlanner(dict(pp_params), kart, line, logger=None, node=None)
    for i in range(i0):                       # runway, open loop on logged poses
        inp = PlannerInputs(pose_xy=(float(fx["x"][i]), float(fx["y"][i])),
                            yaw_rad=float(fx["yaw"][i]),
                            speed_mps=float(fx["v"][i]) * speed_scale,
                            track_angles=None, now_ns=int(t[i] * 1e9))
        pl.plan(inp)
        pp.plan(inp)

    x, y, yaw = float(fx["x"][i0]), float(fx["y"][i0]), float(fx["yaw"][i0])
    xs, ys, cmds, costs = [x], [y], [], []
    actual = 0.0
    rate = math.radians(STEER_RATE_MAX_DEGPS)
    for i in range(i0, i1):
        v = float(fx["v"][i]) * speed_scale
        dt = float(t[i + 1] - t[i])
        inp = PlannerInputs(pose_xy=(x, y), yaw_rad=yaw, speed_mps=v,
                            track_angles=None, now_ns=int(t[i] * 1e9))
        accel_mean = pl.u_mean[1, :].copy()
        delta_prev = pl.delta_prev
        proposed = pl.plan(inp)
        pp_out = pp.plan(inp)
        if kind == "perfect":
            cand = [_cost_of(pl, accel_mean, delta_prev, x, y, yaw, v, d)
                    for d in SWEEP_DEG]
            deg = float(SWEEP_DEG[int(np.argmin(cand))])
            cost = float(np.min(cand))
        else:
            if kind == "pp":
                deg = pp_out[1] if pp_out else 0.0
            elif kind == "logged":
                deg = float(fx["logged_cmd_deg"][i])
            else:
                deg = proposed[1] if proposed else 0.0
            cost = _cost_of(pl, accel_mean, delta_prev, x, y, yaw, v, deg)
        cmds.append(deg)
        costs.append(cost)
        actual += float(np.clip(math.radians(gain * 2.0 * deg) - actual,
                                -rate * dt, rate * dt))
        x += dt * v * math.cos(yaw)
        y += dt * v * math.sin(yaw)
        yaw += dt * v / WHEELBASE * math.tan(actual)
        xs.append(x)
        ys.append(y)
    xs_a, ys_a = np.array(xs), np.array(ys)
    cte = np.array([np.min(np.hypot(lx - a, ly - b)) for a, b in zip(xs_a, ys_a)])
    return Run(xs_a, ys_a, np.array(cmds), np.array(costs), cte)


def run_config(overrides: Optional[dict] = None,
               seeds: Sequence[int] = DEFAULT_SEEDS,
               gain: float = DEFAULT_GAIN,
               label: Optional[str] = None,
               speed_scale: float = 1.0) -> BenchResult:
    """Score one MPC config across all three scenarios."""
    mpc_params, pp_params, kart = load_baseline()
    mpc_params = dict(mpc_params)
    if overrides:
        mpc_params.update(overrides)
    mpc_params.setdefault("residual.mode", "off")
    mpc_params["residual.cache_enabled"] = False
    line = load_line()

    if label is None:
        label = ("shipped 05/19" if not overrides else
                 ", ".join(f"{k}={v}" for k, v in sorted(overrides.items())))
    res = BenchResult(label=label)
    for sc in SCENARIOS:
        res.per_scenario[sc.label] = [
            drive(sc, "mpc", mpc_params, pp_params, kart, line, s, gain,
                  speed_scale)
            for s in seeds
        ]
        res.perfect[sc.label] = drive(sc, "perfect", mpc_params, pp_params,
                                      kart, line, seeds[0], gain, speed_scale)
    return res


def main() -> None:
    import argparse
    import json

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL",
                    help="override one mpc param, repeatable")
    ap.add_argument("--sweep", default=None, metavar="KEY=V1,V2,...",
                    help="score one param at several values")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--speed-scale", type=float, default=1.0,
                    help="scale the logged speed; 1.15 emulates the 05/17 light kart")
    ap.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                    help=f"plant cmd->wheel gain (default {DEFAULT_GAIN}; "
                         "05/20-21 kart is ~0.30)")
    args = ap.parse_args()

    def parse(v):
        try:
            return json.loads(v)
        except json.JSONDecodeError:
            return v

    base = {}
    for item in args.set:
        k, v = item.split("=", 1)
        base[k] = parse(v)
    seeds = tuple(range(args.seeds))

    if args.sweep:
        key, vals = args.sweep.split("=", 1)
        print(f"sweep {key} over {vals}   gain={args.gain}  seeds={args.seeds}\n")
        rows = []
        for raw in vals.split(","):
            ov = dict(base)
            ov[key] = parse(raw)
            r = run_config(ov, seeds=seeds, gain=args.gain,
                           label=f"{key}={raw}")
            rows.append((raw, r))
            print(f"--- {key}={raw}")
            print(r.summary())
            print()
        print(f"{'value':>12}{'SCORE':>10}{'mean max|CTE|':>16}")
        for raw, r in rows:
            print(f"{raw:>12}{r.score:10.2f}{r.mean_max_cte:16.2f}")
        best = min(rows, key=lambda kv: kv[1].score)
        print(f"\nclosest to perfect: {key}={best[0]}  (score {best[1].score:.2f})")
    else:
        r = run_config(base or None, seeds=seeds, gain=args.gain,
                       speed_scale=args.speed_scale)
        print(f"config: {r.label}   gain={args.gain}  seeds={args.seeds}  speed_scale={args.speed_scale}\n")
        print(r.summary())


if __name__ == "__main__":
    main()
