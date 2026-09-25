"""Offline RL-residual training against the learned plant model.

Phase 3 of docs/rl_residual_plan.md. This is where RLResidualLearner.update()
actually gets called — deliberately NOT from live ticks (see the plan doc's
"reward-validity finding": without a real command-blending path, a live
reward would be statistically independent of the sampled action, and the
policy-gradient estimate would be zero in expectation). Here the residual's
proposed correction is actually added to the command fed into LearnedPlant,
so the simulated outcome genuinely responds to the action — the gradient
estimate is valid.

Uses the real `PurePursuitPlanner` as the "baseline planner" standing in
for MPC (replicating the sampling-MPC solve itself here would be a large
duplication of mpc.py — out of scope for this script; PurePursuitPlanner
has zero ROS dependency and is already a proven, tuned component in this
codebase, unlike the from-scratch PD+feedforward controller firing 4 wrote
and then spent an entire firing debugging — see docs/rl_residual_plan.md's
firing 4/5 log entries). The RL residual learns a correction on top of
that baseline; the point is whether adding a learned residual improves
tracking over the baseline alone, which is the question that matters
regardless of which baseline it's layered onto.

Known simplification: RLResidualLearner's "accel correction" is applied
here as a direct m/s nudge to LearnedPlant's speed setpoint, not a literal
acceleration term — the live system commands a speed setpoint too (see
mpc.py's `throttle_mps = v_ref_now`, not an integrated accel_cmd), so this
matches how a correction would actually reach the plant once Phase 2's
command-blending TODO is closed. Worth re-checking when that happens.

Status as of the last firing (see docs/rl_residual_plan.md for the full
trail): real linear residual signal exists here (confirmed independently
via sim/rl_residual_signal_check.py's direct-fit oracle regression,
R^2=0.75) and `RLResidualLearner.closed_loop_gain` fixes the mechanism
that was causing regressions (full-magnitude per-tick reapplication of a
correction sized for a 0.5s hold is a closed-loop stability problem, not a
missing-signal problem). Actual RL training through this script still
doesn't cleanly beat baseline end-to-end, most likely because damping the
applied action also damps the reward-difference signal REINFORCE learns
from — untested next steps are more episodes and/or a higher
`learning_rate`, not a different mechanism. `--steer-only` isolates the
accel dimension if you want to rule it back in/out again.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_AK_PKG_ROOT = os.path.join(REPO, "src", "autonomous_kart")
if REPO not in sys.path:
    sys.path.insert(0, REPO)
if _AK_PKG_ROOT not in sys.path:
    sys.path.insert(0, _AK_PKG_ROOT)

from sim.learned_plant import LearnedPlant  # noqa: E402
from autonomous_kart.nodes.pathfinder.planners.base import (  # noqa: E402
    KartConstants, PlannerInputs,
)
from autonomous_kart.nodes.pathfinder.planners.mpc_residual import (  # noqa: E402
    NUM_FEATURES, NUM_STEER_HIST, NUM_THROTTLE_HIST, _features,
)
from autonomous_kart.nodes.pathfinder.planners.pure_pursuit import (  # noqa: E402
    PurePursuitPlanner,
)
from autonomous_kart.nodes.pathfinder.planners.rl_residual import (  # noqa: E402
    RLResidualLearner,
)

# Reward weights mirror mpc.py's own w_d/w_heading defaults (100.0, 30.0) so
# "reward" reflects the same priorities the live cost function has, not an
# arbitrary invented scale.
W_D = 100.0
W_HEADING = 30.0

# Kart-wide constants, copied from pathfinder.yaml's `/**:` block (the
# values actually deployed, not the code's own fallback defaults).
KART = KartConstants(
    v_max_mps=12.0, wheelbase_m=1.05, steer_max_deg=60.0,
    steer_rate_max_degps=33.7, a_max_mps2=5.3, a_min_mps2=-3.0,
    a_lat_max_mps2=5.3,
)

# PurePursuitPlanner params, copied from pathfinder.yaml's pure_pursuit.*
# block — the team's own tuned values, not guesses.
PURE_PURSUIT_PARAMS = {
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
    "max_resync_dist": 80.0,
    "max_closed_dist": 2.0,
    "rejoin_cte_activate": 3.0,
    "rejoin_cte_deactivate": 1.0,
    "rejoin_merge_lookahead_m": 15.0,
    "latency_s": 0.3,
}


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class RacingLine:
    """Minimal standalone Frenet helper — pure numpy, no ROS dependency.
    Full nearest-point search each call (line is ~200 points; a windowed
    search like mpc.py's isn't worth the complexity for an offline script)."""

    def __init__(self, csv_path: str):
        rows = np.loadtxt(csv_path, delimiter=",", skiprows=1)
        self.s = rows[:, 0]
        self.x = rows[:, 1]
        self.y = rows[:, 2]
        self.psi = rows[:, 3]
        self.kappa = rows[:, 4]
        self.vx = rows[:, 5]
        self.n = len(self.s)
        # PurePursuitPlanner wants a list of row-tuples, same shape as what
        # pathfinder_node loads from CSV (_load_line_csv).
        self.rows = [tuple(float(v) for v in row) for row in rows]

    def frenet(self, x: float, y: float):
        d2 = (self.x - x) ** 2 + (self.y - y) ** 2
        j = int(np.argmin(d2))
        dx, dy = x - self.x[j], y - self.y[j]
        psi = self.psi[j]
        # d is signed, left positive (matches mpc.py's _frenet convention).
        d = -dx * math.sin(psi) + dy * math.cos(psi)
        s = self.s[j] + dx * math.cos(psi) + dy * math.sin(psi)
        return s, d, j, psi, self.kappa[j]


def clone_plant(plant: LearnedPlant) -> LearnedPlant:
    """Deep-copy the mutable rollout state; model/whitener refs are
    read-only during step() so a shallow share of those would also be
    fine, but deepcopy is simplest to get right in a training script
    where correctness matters more than speed."""
    return copy.deepcopy(plant)


def nominal_rollout(plant: LearnedPlant, target_mps: float, steer_deg: float,
                     dt: float, n_steps: int):
    """Hold (target_mps, steer_deg) constant for n_steps on a cloned plant —
    mirrors mpc.py's _hold_rollout, backed by the learned plant instead of
    the closed-form bicycle model."""
    clone = clone_plant(plant)
    x = y = yaw = v = None
    for _ in range(n_steps):
        x, y, yaw, v = clone.step(target_mps, steer_deg, dt)
    return x, y


def run_episode(plant: LearnedPlant, line: RacingLine, learner: RLResidualLearner,
                 dt: float, n_steps: int, nom_steps: int, start_idx: int,
                 explore: bool, off_track_d: float = 2.5, steer_only: bool = False):
    """Runs one episode; returns (mean_abs_d, ticks_survived)."""
    x0, y0, yaw0 = line.x[start_idx], line.y[start_idx], line.psi[start_idx]
    v0 = float(line.vx[start_idx])
    plant.reset(x0, y0, yaw0, v0)
    # Fresh instance per episode: PurePursuitPlanner is stateful
    # (closest_idx, line_manager) and episodes start at different points on
    # the line, so reusing one instance across episodes would leak state.
    baseline = PurePursuitPlanner(PURE_PURSUIT_PARAMS, KART, line.rows)

    steer_hist = [0.0] * NUM_STEER_HIST
    throttle_hist = [0.0] * NUM_THROTTLE_HIST
    abs_d_hist = []

    x, y, yaw, v = x0, y0, yaw0, v0
    for _ in range(n_steps):
        s, d, j, psi_track, kappa = line.frenet(x, y)
        heading_err = _wrap(yaw - psi_track)
        v_s = v * math.cos(heading_err)
        v_d = v * math.sin(heading_err)

        base_inputs = PlannerInputs(
            pose_xy=(x, y), yaw_rad=yaw, speed_mps=v, track_angles=None, now_ns=0)
        target_base, steer_base_deg = baseline.plan(base_inputs)

        nom_x, nom_y = nominal_rollout(plant, target_base, steer_base_deg, dt, nom_steps)
        nom_s, nom_d, _, _, _ = line.frenet(nom_x, nom_y)

        phi = _features(d, v_s, v_d, kappa, tuple(steer_hist), tuple(throttle_hist),
                        nom_s - s, nom_d - d)
        steer_corr_deg, speed_corr_mps = learner.act(phi, explore=explore)
        if steer_only:
            # Isolates whether the accel/speed dimension is the problem —
            # firing 7 found grid-searching speed was pure reward hacking
            # (the reward here has no speed-tracking term, so "slow down"
            # trivially looks good one-shot regardless of state); the RL
            # policy's accel dimension could be learning the same exploit.
            speed_corr_mps = 0.0

        steer_cmd_deg = steer_base_deg + steer_corr_deg
        target_mps = max(0.0, target_base + speed_corr_mps)

        x, y, yaw, v = plant.step(target_mps, steer_cmd_deg, dt)

        s_new, d_new, _, psi_track_new, _ = line.frenet(x, y)
        heading_err_new = _wrap(yaw - psi_track_new)
        reward = -(W_D * d_new ** 2 + W_HEADING * heading_err_new ** 2)
        if explore:
            learner.update(reward)

        steer_hist = [math.radians(steer_cmd_deg)] + steer_hist[:-1]
        throttle_hist = [target_mps - target_base] + throttle_hist[:-1]
        abs_d_hist.append(abs(d_new))

        if abs(d_new) > off_track_d:
            break

    return float(np.mean(abs_d_hist)) if abs_d_hist else float("nan"), len(abs_d_hist)


def evaluate(plant: LearnedPlant, line: RacingLine, learner: RLResidualLearner,
             dt: float, n_steps: int, nom_steps: int, n_episodes: int, rng: np.random.Generator,
             steer_only: bool = False):
    errs = []
    for _ in range(n_episodes):
        start_idx = int(rng.integers(0, line.n))
        mean_abs_d, _ = run_episode(plant, line, learner, dt, n_steps, nom_steps,
                                     start_idx, explore=False, steer_only=steer_only)
        errs.append(mean_abs_d)
    return float(np.nanmean(errs))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plant-meta", default=os.path.join(REPO, "sim/model/plant_linear.json"))
    ap.add_argument("--line", default=os.path.join(REPO, "data/racing_line/center.csv"))
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--episode-seconds", type=float, default=5.0)
    ap.add_argument("--eval-episodes", type=int, default=30)
    ap.add_argument("--out", default=os.path.join(REPO, "sim/model/rl_residual_checkpoint.json"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--learning-rate", type=float, default=None,
                     help="Override RLResidualLearner's default learning_rate "
                          "(1e-3). Untested hypothesis from firing 8: "
                          "closed_loop_gain=0.03 damps the applied action and "
                          "with it the reward-difference signal REINFORCE sees, "
                          "so a higher lr may be needed to compensate.")
    ap.add_argument("--steer-only", action="store_true",
                     help="Zero the accel/speed correction before it reaches the plant "
                          "(the reward has no speed-tracking term, so that dimension can "
                          "learn the same 'always slow down' exploit firing 7 found and "
                          "ruled out for the oracle check — isolates whether it's "
                          "dragging down the steer dimension's otherwise-real signal).")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    np.random.seed(args.seed)  # RLResidualLearner.act() uses the global RNG

    line = RacingLine(args.line)
    plant = LearnedPlant.from_linear(args.plant_meta)
    dt = plant.dt
    n_steps = int(round(args.episode_seconds / dt))
    nom_steps = max(1, int(round(0.5 / dt)))  # matches mpc.residual.target_horizon_s default

    learner_params = {"mode": "apply"}
    if args.learning_rate is not None:
        learner_params["learning_rate"] = args.learning_rate
    learner = RLResidualLearner(learner_params, feature_dim=NUM_FEATURES)

    print(f"plant: {args.plant_meta} (hz={1.0/dt:.0f})")
    print(f"line: {args.line} ({line.n} pts)")
    print(f"training: {args.episodes} episodes x {n_steps} ticks"
          + (" (steer-only)" if args.steer_only else ""))

    baseline_err = evaluate(plant, line, RLResidualLearner({"mode": "off"}, feature_dim=NUM_FEATURES),
                            dt, n_steps, nom_steps, args.eval_episodes, rng)
    print(f"baseline (no residual) mean |d|: {baseline_err:.4f} m")

    survived_fracs = []
    for ep in range(args.episodes):
        start_idx = int(rng.integers(0, line.n))
        _, ticks = run_episode(plant, line, learner, dt, n_steps, nom_steps,
                               start_idx, explore=True, steer_only=args.steer_only)
        survived_fracs.append(ticks / n_steps)
        if (ep + 1) % 50 == 0:
            recent = np.mean(survived_fracs[-50:])
            print(f"  episode {ep+1}/{args.episodes}  "
                  f"survived={recent:.2f}  samples_trained={learner.samples_trained}")

    trained_err = evaluate(plant, line, learner, dt, n_steps, nom_steps,
                           args.eval_episodes, rng, steer_only=args.steer_only)
    print(f"trained (RL residual) mean |d|: {trained_err:.4f} m")
    delta = baseline_err - trained_err
    print(f"delta: {delta:+.4f} m ({'improved' if delta > 0 else 'regressed'})")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    state = learner.state_dict()
    with open(args.out, "w") as f:
        json.dump({
            "weights": state["weights"].tolist(),
            "log_sigma": state["log_sigma"].tolist(),
            "samples_trained": state["samples_trained"],
            "baseline_mean_abs_d": baseline_err,
            "trained_mean_abs_d": trained_err,
            "feature_dim": NUM_FEATURES,
        }, f, indent=2)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
