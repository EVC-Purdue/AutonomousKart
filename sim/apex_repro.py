"""Replay the 20260519_214619 apex failure against the live MPC params.

Through the corner at t=17.0-18.0 s the kart's MPC commanded roughly zero while
its own cost minimum and pure pursuit both asked for +20 to +24 deg. The kart
left the corridor and ended up 4.7 m off the line.

This module replays the logged poses from that run through MPCPlanner so a
param change can be scored without going back to the kart. Poses come from the
bag (open loop); the planner's internal state — warm start, delta_prev, Frenet
index — evolves tick to tick, which is what reproduces the failure: the sampler
ratchets too slowly to follow a step in the required steering.

Three numbers per tick:
  commanded  what MPCPlanner.plan() returns
  optimum    argmin over the first action of MPCPlanner's OWN cost
  pursuit    what PurePursuitPlanner asks for at the same pose

`shortfall = optimum - commanded` over the apex window is the score.

RUN IT OVER SEEDS, and read the ensemble, never a named seed. With the 05/19
params the per-seed apex command spans roughly -35 to +20 deg on an identical
cost landscape with an identical Frenet index. Why: among the K=164 candidates
at an apex tick, the first steering action — the only number that gets
executed — explains under 4% of the cost variance (the first accel action
explains 15%, and the rest comes from the other 38 dimensions of the sequence).
Elite selection is therefore almost uncorrelated with the executed action, so
it random-walks instead of being pulled to the optimum.

Both aggregate numbers matter. A fix should pull `mean_abs_shortfall_deg` down
AND `seed_spread_deg` with it, because a solver whose corner behaviour depends
on its RNG draw is not fixed however good its mean looks.

This module is a library. Use `sim/runner.py apex` for the CLI.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

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

FIXTURE = os.path.join(HERE, "fixtures", "apex_20260519_214619.npz")
BASELINE_JSON = os.path.join(HERE, "fixtures", "apex_params_20260519.json")
PARAMS_YAML = os.path.join(
    REPO, "src", "autonomous_kart", "autonomous_kart", "params", "pathfinder.yaml"
)
LINE_CSV = os.path.join(REPO, "data", "racing_line", "line6.csv")

# The corner. From the bag: cross-track leaves the corridor at 17.0 s and the
# kart is 2.5 m out by 18.0 s.
APEX_T0, APEX_T1 = 17.0, 18.0
PASS_SHORTFALL_DEG = 8.0
PASS_SPREAD_DEG = 6.0
DEFAULT_SEEDS = tuple(range(8))


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------
def load_fixture(path: str = FIXTURE) -> Dict[str, np.ndarray]:
    z = np.load(path)  # plain arrays + a "U80" meta column; no pickle needed
    return {k: z[k] for k in z.files}


def load_line(path: str = LINE_CSV) -> List[List[float]]:
    import csv
    with open(path) as fh:
        return [[float(v) for v in row] for row in list(csv.reader(fh))[1:]]


def _kart_from(d: dict) -> KartConstants:
    return KartConstants(
        v_max_mps=float(d.get("v_max_mps", 12.0)),
        wheelbase_m=float(d.get("wheelbase_m", 1.05)),
        steer_max_deg=float(d.get("steer_max_deg", 60.0)),
        steer_rate_max_degps=float(d.get("steer_rate_max_degps", 180.0)),
        a_max_mps2=float(d.get("a_max_mps2", 2.0)),
        a_min_mps2=float(d.get("a_min_mps2", -3.0)),
        a_lat_max_mps2=float(d.get("a_lat_max_mps2", 8.0)),
    )


def load_baseline() -> Tuple[dict, dict, KartConstants]:
    """The params exactly as they ran on 05/19. Running the repro with these
    reproduces the bag, which is what makes it a repro rather than a demo."""
    import json
    with open(BASELINE_JSON) as fh:
        doc = json.load(fh)
    return doc["mpc"], doc["pure_pursuit"], _kart_from(doc["kart"])


def load_params(yaml_path: str = PARAMS_YAML) -> Tuple[dict, dict, KartConstants]:
    """Read the live pathfinder.yaml, the way the node reads it: flat dotted
    keys under `pathfinder_node`, kart constants under the `/**:` wildcard."""
    import yaml
    with open(yaml_path) as fh:
        doc = yaml.safe_load(fh)
    wild = doc.get("/**", {}).get("ros__parameters", {}) or {}
    node = doc.get("pathfinder_node", {}).get("ros__parameters", {}) or {}
    mpc, pp = {}, {}
    for key, val in node.items():
        if key.startswith("mpc."):
            mpc[key[4:]] = val
        elif key.startswith("pure_pursuit."):
            pp[key[len("pure_pursuit."):]] = val
    return mpc, pp, _kart_from(wild)


# --------------------------------------------------------------------------
# results
# --------------------------------------------------------------------------
@dataclass
class ApexResult:
    """One seed."""
    seed: int
    t: np.ndarray
    commanded: np.ndarray      # deg, MPCPlanner.plan() output
    optimum: np.ndarray        # deg, argmin of MPC's own cost (NaN where not probed)
    pursuit: np.ndarray        # deg, PurePursuitPlanner at the same pose
    logged: np.ndarray         # deg, what the kart's MPC actually output on 05/19
    cte: np.ndarray            # m, logged cross-track
    v: np.ndarray              # m/s
    sweep_limit_deg: float = 60.0

    @property
    def apex(self) -> np.ndarray:
        return (self.t >= APEX_T0) & (self.t <= APEX_T1)

    @property
    def shortfall_deg(self) -> float:
        """Signed: positive = under-steering the optimum, negative = past it."""
        m = self.apex
        return float(np.nanmean(self.optimum[m] - self.commanded[m]))

    @property
    def abs_shortfall_deg(self) -> float:
        """How far the command sits from the optimum, either side. Gating on
        the signed mean would pass a solver that blows through the optimum."""
        m = self.apex
        return float(np.nanmean(np.abs(self.optimum[m] - self.commanded[m])))

    @property
    def optimum_clipped(self) -> bool:
        """True when the cost minimum sits at the edge of the sweep, i.e. at
        steer_max_deg. The planner cannot command past its own clamp, so the
        real optimum is further out and every gap below is a lower bound.
        Happens whenever actuator_gain shrinks the command units enough that
        the needed wheel angle no longer fits inside steer_max_deg."""
        m = self.apex & np.isfinite(self.optimum)
        if not m.any():
            return False
        return bool(np.mean(np.abs(self.optimum[m]) >= self.sweep_limit_deg - 1e-6) > 0.5)

    @property
    def apex_command_deg(self) -> float:
        return float(np.nanmean(self.commanded[self.apex]))

    @property
    def lead_in_tracking_deg(self) -> float:
        """Mean |optimum - commanded| before the corner. Control: small even
        with the shipped params, so a regression here is unrelated to the apex."""
        m = self.t < APEX_T0
        if not m.any():
            return float("nan")
        return float(np.nanmean(np.abs(self.optimum[m] - self.commanded[m])))


@dataclass
class ApexReport:
    """An ensemble over seeds. This is what you read."""
    results: List[ApexResult] = field(default_factory=list)
    label: str = "live pathfinder.yaml"

    @property
    def shortfalls(self) -> np.ndarray:
        return np.array([r.shortfall_deg for r in self.results])

    @property
    def mean_shortfall_deg(self) -> float:
        """Signed, for diagnosis: positive = under, negative = over."""
        return float(np.mean(self.shortfalls))

    @property
    def mean_abs_shortfall_deg(self) -> float:
        """The gate. Distance from the optimum regardless of side."""
        return float(np.mean([r.abs_shortfall_deg for r in self.results]))

    @property
    def seed_spread_deg(self) -> float:
        """Std of the per-seed apex command. A solver whose corner behaviour
        depends on its RNG draw is not fixed, however good the mean looks."""
        return float(np.std([r.apex_command_deg for r in self.results]))

    @property
    def passed(self) -> bool:
        return (self.mean_abs_shortfall_deg < PASS_SHORTFALL_DEG
                and self.seed_spread_deg < PASS_SPREAD_DEG)

    def summary(self) -> str:
        r0 = self.results[0]
        m = r0.apex
        lines = [
            f"apex repro — {self.label}",
            f"bag 20260519_214619, window {APEX_T0:.1f}-{APEX_T1:.1f} s "
            f"({int(m.sum())} ticks), {len(self.results)} seeds",
            "",
            f"  {'seed':>5}{'apex cmd':>11}{'signed':>10}{'|gap|':>9}{'lead-in':>10}",
        ]
        for r in self.results:
            lines.append(f"  {r.seed:5d}{r.apex_command_deg:+11.2f}"
                         f"{r.shortfall_deg:+10.2f}{r.abs_shortfall_deg:9.2f}"
                         f"{r.lead_in_tracking_deg:10.2f}")
        lines += [
            "",
            f"  cost minimum wanted   {np.nanmean(r0.optimum[m]):+7.2f} deg",
            f"  pure pursuit wanted   {np.nanmean(r0.pursuit[m]):+7.2f} deg",
            f"  the kart commanded    {np.mean(r0.logged[m]):+7.2f} deg   (05/19 log)",
            "",
            f"  mean |gap to optimum| {self.mean_abs_shortfall_deg:7.2f} deg   "
            f"(pass < {PASS_SHORTFALL_DEG:.1f})",
            f"  mean signed gap       {self.mean_shortfall_deg:+7.2f} deg   "
            f"({'under' if self.mean_shortfall_deg > 0 else 'past'} the optimum)",
            f"  seed spread           {self.seed_spread_deg:7.2f} deg   "
            f"(pass < {PASS_SPREAD_DEG:.1f})",
            "",
            *(["  NOTE: the cost minimum is pinned at steer_max_deg "
               f"({r0.sweep_limit_deg:.0f} deg), so the gap above is a LOWER BOUND —",
               "        the planner's own clamp is below what this corner asks for.", ""]
              if r0.optimum_clipped else []),
            ("PASS — the solver reaches its own optimum through the corner, "
             "repeatably" if self.passed else
             "FAIL — the solver does not reliably reach its own optimum "
             "through the corner"),
        ]
        return "\n".join(lines)


# --------------------------------------------------------------------------
# the probe
# --------------------------------------------------------------------------
def _cost_minimum(planner: MPCPlanner, accel_mean: np.ndarray, delta_prev: float,
                  x: float, y: float, yaw: float, v: float,
                  sweep_deg: np.ndarray) -> float:
    """argmin over the first action of the planner's own cost.

    Zeroes the sampling noise so each rollout is the held candidate, then reads
    `_solve`'s reported cost. Restores the planner, so this is a read-only
    probe of the tick the solver just faced.

    `_solve` draws from `planner._rng` even at sigma=0, so the RNG state is
    saved and restored too. Without that the probe advances the stream and the
    replayed commands depend on how often we measure them.
    """
    keep = (planner.steer_sigma, planner.accel_sigma,
            planner.u_mean.copy(), planner.delta_prev)
    rng_state = planner._rng.bit_generator.state
    planner.steer_sigma = 0.0
    planner.accel_sigma = 0.0
    best_cost, best_deg = float("inf"), 0.0
    try:
        for deg in sweep_deg:
            planner.u_mean[0, :] = math.radians(deg)
            planner.u_mean[1, :] = accel_mean
            planner.delta_prev = delta_prev
            _, cost, _, _, _ = planner._solve(
                x, y, yaw, max(v, 1e-3), planner.closest_idx,
                planner.target_speed, planner.v_max,
            )
            if cost < best_cost:
                best_cost, best_deg = cost, float(deg)
    finally:
        (planner.steer_sigma, planner.accel_sigma,
         planner.u_mean, planner.delta_prev) = keep
        planner._rng.bit_generator.state = rng_state
    return best_deg


def run_one(seed: int, mpc_params: dict, pp_params: dict, kart: KartConstants,
            fx: Dict[str, np.ndarray], line: List[List[float]],
            sweep_deg: np.ndarray, optimum_stride: int) -> ApexResult:
    mpc = MPCPlanner(dict(mpc_params), kart, line, logger=None, node=None)
    mpc._rng = np.random.default_rng(seed)
    pp = PurePursuitPlanner(dict(pp_params), kart, line, logger=None, node=None)

    n = len(fx["t"])
    commanded = np.full(n, np.nan)
    optimum = np.full(n, np.nan)
    pursuit = np.full(n, np.nan)
    for i in range(n):
        inputs = PlannerInputs(
            pose_xy=(float(fx["x"][i]), float(fx["y"][i])),
            yaw_rad=float(fx["yaw"][i]),
            speed_mps=float(fx["v"][i]),
            track_angles=None,
            now_ns=int(fx["t"][i] * 1e9),
        )
        accel_before = mpc.u_mean[1, :].copy()
        delta_before = mpc.delta_prev
        out = mpc.plan(inputs)
        if out is not None:
            commanded[i] = out[1]
        if (APEX_T0 <= fx["t"][i] <= APEX_T1) or i % optimum_stride == 0:
            optimum[i] = _cost_minimum(
                mpc, accel_before, delta_before,
                inputs.pose_xy[0], inputs.pose_xy[1], inputs.yaw_rad,
                inputs.speed_mps, sweep_deg,
            )
        pp_out = pp.plan(inputs)
        if pp_out is not None:
            pursuit[i] = pp_out[1]

    return ApexResult(
        seed=seed, t=fx["t"], commanded=commanded, optimum=optimum,
        pursuit=pursuit, logged=fx["logged_delta_deg"],
        cte=fx["logged_cte_m"], v=fx["v"],
        sweep_limit_deg=float(np.max(np.abs(sweep_deg))),
    )


def run(mpc_overrides: Optional[dict] = None,
        seeds: Sequence[int] = DEFAULT_SEEDS,
        baseline: bool = False,
        fixture: str = FIXTURE,
        yaml_path: str = PARAMS_YAML,
        line_csv: str = LINE_CSV,
        sweep_deg: Optional[np.ndarray] = None,
        optimum_stride: int = 10) -> ApexReport:
    """Replay the fixture through MPCPlanner and PurePursuitPlanner, per seed.

    `baseline=True` uses the 05/19 params instead of the live yaml, which is how
    the harness is checked against the bag.

    The cost-minimum probe is a sweep per tick, so it runs on every apex tick
    but only every `optimum_stride` ticks in the lead-in, where it only feeds
    the control metric. Lead-in ticks are still replayed in full: the warm start
    has to evolve, and that is what carries the failure into the corner.
    """
    fx = load_fixture(fixture)
    line = load_line(line_csv)
    mpc_params, pp_params, kart = (
        load_baseline() if baseline else load_params(yaml_path)
    )
    if mpc_overrides:
        mpc_params = dict(mpc_params)
        mpc_params.update(mpc_overrides)
    # The learner is telemetry-only inside plan(); keep it off so the repro does
    # not depend on a cache directory existing.
    mpc_params.setdefault("residual.mode", "off")
    mpc_params["residual.cache_enabled"] = False

    if sweep_deg is None:
        # Cover the planner's whole range. With actuator_gain < 1 the command
        # units inflate, so a fixed +-30 window clips the optimum.
        lim = float(kart.steer_max_deg)
        sweep_deg = np.linspace(-lim, lim, 41)

    label = "05/19 baseline" if baseline else "live pathfinder.yaml"
    if mpc_overrides:
        label += " + " + ", ".join(f"{k}={v}" for k, v in sorted(mpc_overrides.items()))
    return ApexReport(
        results=[run_one(s, mpc_params, pp_params, kart, fx, line,
                         sweep_deg, optimum_stride) for s in seeds],
        label=label,
    )


def plot(report: ApexReport, path: str) -> None:
    """Per-seed commands against the cost minimum and pure pursuit."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    r0 = report.results[0]
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(10.5, 5.6), dpi=110, sharex=True,
        gridspec_kw=dict(height_ratios=[1.8, 1], hspace=.18),
    )
    for s in (ax, ax2):
        for sp in s.spines.values():
            sp.set_visible(False)
        s.tick_params(colors="#8a8a8a", labelsize=9, length=0)
        s.axvspan(APEX_T0, APEX_T1, color="#fdecea", zorder=0)

    for r in report.results:
        ax.plot(r.t, r.commanded, color="#1f6feb", lw=1.0, alpha=.45, zorder=3)
    ok = np.isfinite(r0.optimum)
    ax.plot(r0.t[ok], r0.optimum[ok], color="#6b6b6b", lw=1.8, ls=(0, (4, 3)),
            zorder=4, label="MPC's own cost minimum")
    ax.plot(r0.t, r0.pursuit, color="#7b2ff2", lw=1.8, zorder=4, label="pure pursuit")
    ax.plot(r0.t, r0.logged, color="#111111", lw=1.6, zorder=5,
            label="what the kart actually commanded")
    ax.plot([], [], color="#1f6feb", lw=1.6, label=f"MPC, {len(report.results)} seeds")
    ax.axhline(0, color="#ddd", lw=.8, zorder=1)
    ax.set_ylabel("planner output (deg)", fontsize=9.5, color="#555")
    ax.legend(frameon=False, fontsize=9, loc="upper left", ncol=3)
    ax.set_title(f"apex repro — {report.label}", fontsize=11, color="#1b1b1b", loc="left")

    ax2.plot(r0.t, r0.cte, color="#1f6feb", lw=2.0)
    ax2.axhspan(-1.0, 1.0, color="#e9eef5", zorder=0)
    ax2.set_ylabel("logged cross-track (m)", fontsize=9.5, color="#555")
    ax2.set_xlabel("time (s)", fontsize=9.5, color="#555")
    fig.savefig(path, facecolor="white", bbox_inches="tight")
    plt.close(fig)
