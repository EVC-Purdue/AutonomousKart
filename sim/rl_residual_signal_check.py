"""Sanity check: does *any* exploitable linear residual signal exist here?

docs/rl_residual_plan.md's firing 5 ended with a clean but negative result:
a converged (no longer diverging) linear-policy RL residual reliably
regressed tracking performance on top of the PurePursuit baseline. Firing
5's own conclusion was not to guess at more RL hyperparameters, but to
check whether there's anything useful to learn at all, with a method that
doesn't carry policy-gradient's own training-dynamics failure modes.

Method: at each tick, grid-search a small set of candidate residual
corrections around the baseline command, roll exactly ONE step forward per
candidate on a *cloned* plant (never touches the real trajectory), and
keep whichever minimizes next-tick tracking cost — a one-step-lookahead
oracle label, not a learned policy. Fit a plain closed-form linear
regression (ordinary least squares, no training dynamics to go wrong)
from _features() to that oracle correction, over data collected along the
baseline-only trajectory (shadow-style: observe what the baseline does,
label what would have improved it, without acting on it). Then:

  1. Report R^2, and how often the oracle actually picked a
     nonzero correction — does a linear map explain real variance, or is
     the "best" grid choice usually just "do nothing" (no room to help)?
  2. Actually evaluate the FITTED regression as a deterministic residual
     policy in the same harness, vs. baseline alone. If a directly-fitted
     (not RL-trained) correction also fails to help, that's strong
     evidence there's no useful linear residual signal here at all — not
     that RL specifically failed to find one that exists.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from sim.learned_plant import LearnedPlant  # noqa: E402
from sim.train_rl_residual import (  # noqa: E402
    KART, PURE_PURSUIT_PARAMS, W_D, W_HEADING, RacingLine, _wrap, clone_plant,
    nominal_rollout,
)
from autonomous_kart.nodes.pathfinder.planners.base import PlannerInputs  # noqa: E402
from autonomous_kart.nodes.pathfinder.planners.mpc_residual import (  # noqa: E402
    NUM_STEER_HIST, NUM_THROTTLE_HIST, _features,
)
from autonomous_kart.nodes.pathfinder.planners.pure_pursuit import (  # noqa: E402
    PurePursuitPlanner,
)

STEER_GRID = (-2.0, -1.0, -0.3, 0.0, 0.3, 1.0, 2.0)   # deg
OFF_TRACK_D = 2.5

# Speed/accel correction deliberately dropped from this check (first
# version included it and the oracle always picked "slow toward zero" —
# with a one-step-lookahead, tracking-error-only reward, near-zero
# velocity trivially minimizes next-tick positional error regardless of
# steering, so it always won: reward hacking, not a real finding. R^2=1.0
# on that run was a degenerate artifact of the label being a literal
# constant (std=0.000), not evidence of real signal. Steering-only isolates
# the actual question that matters for this project anyway.


def oracle_correction(plant, line, target_base, steer_base_deg, dt, horizon_steps):
    """Best steer_corr from a small fixed grid (speed pinned to baseline),
    evaluated by holding it constant for `horizon_steps` on a cloned plant
    and scoring the terminal tracking error. A ground-truth label for
    regression, not a policy.

    Originally rolled just ONE tick forward — every grid point tied at
    exactly reward=0.0 (a 2deg steering change over 16.7ms is far too
    short a horizon to produce any measurable position difference; the
    wheel is rate-limited and lateral position responds to steering over
    many accumulating ticks, not one). That degenerate tie was silently
    "won" by whichever grid point happened to be tried first — not a real
    preference. horizon_steps should match a timescale steering can
    actually act over (0.5s / 30 ticks, matching the existing
    nominal_rollout / mpc.residual.target_horizon_s convention elsewhere
    in this codebase)."""
    best_reward = -float("inf")
    best = 0.0
    for dsteer in STEER_GRID:
        clone = clone_plant(plant)
        nx = ny = nyaw = None
        for _ in range(horizon_steps):
            nx, ny, nyaw, _ = clone.step(target_base, steer_base_deg + dsteer, dt)
        _, d_new, _, psi_new, _ = line.frenet(nx, ny)
        heading_err_new = _wrap(nyaw - psi_new)
        reward = -(W_D * d_new ** 2 + W_HEADING * heading_err_new ** 2)
        if reward > best_reward:
            best_reward = reward
            best = dsteer
    return best


def collect_oracle_dataset(plant: LearnedPlant, line: RacingLine, dt: float,
                            n_steps: int, nom_steps: int, n_episodes: int,
                            rng: np.random.Generator):
    X, Y = [], []
    for _ in range(n_episodes):
        start_idx = int(rng.integers(0, line.n))
        x, y, yaw = line.x[start_idx], line.y[start_idx], line.psi[start_idx]
        v = float(line.vx[start_idx])
        plant.reset(x, y, yaw, v)
        baseline = PurePursuitPlanner(PURE_PURSUIT_PARAMS, KART, line.rows)
        steer_hist = [0.0] * NUM_STEER_HIST
        throttle_hist = [0.0] * NUM_THROTTLE_HIST

        for _ in range(n_steps):
            s, d, j, psi_track, kappa = line.frenet(x, y)
            heading_err = _wrap(yaw - psi_track)
            v_s, v_d = v * math.cos(heading_err), v * math.sin(heading_err)

            base_inputs = PlannerInputs(
                pose_xy=(x, y), yaw_rad=yaw, speed_mps=v, track_angles=None, now_ns=0)
            target_base, steer_base_deg = baseline.plan(base_inputs)

            nom_x, nom_y = nominal_rollout(plant, target_base, steer_base_deg, dt, nom_steps)
            nom_s, nom_d, _, _, _ = line.frenet(nom_x, nom_y)
            phi = _features(d, v_s, v_d, kappa, tuple(steer_hist), tuple(throttle_hist),
                            nom_s - s, nom_d - d)

            best_dsteer = oracle_correction(plant, line, target_base, steer_base_deg, dt, nom_steps)
            X.append(phi)
            Y.append(best_dsteer)

            # Advance the real trajectory via baseline-only — shadow-style
            # data collection, not the oracle's pick (keeps the state
            # distribution simple and doesn't compound a noisy grid choice
            # through the rest of the episode).
            x, y, yaw, v = plant.step(target_base, steer_base_deg, dt)
            steer_hist = [math.radians(steer_base_deg)] + steer_hist[:-1]
            throttle_hist = [0.0] + throttle_hist[:-1]

            if abs(d) > OFF_TRACK_D:
                break

    return np.array(X), np.array(Y)


def fit_linear(X: np.ndarray, Y: np.ndarray):
    """Y is 1-D (steer_corr only). Returns (weights, scalar R^2)."""
    W, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
    Y_pred = X @ W
    ss_res = float(np.sum((Y - Y_pred) ** 2))
    ss_tot = float(np.sum((Y - Y.mean()) ** 2))
    r2 = 1.0 - ss_res / max(ss_tot, 1e-9)
    return W, r2


def evaluate_fixed_policy(plant: LearnedPlant, line: RacingLine, dt: float,
                           n_steps: int, nom_steps: int, n_episodes: int,
                           rng: np.random.Generator, W=None):
    """W=None -> baseline alone. Otherwise apply (features @ W) as a fixed,
    non-learning residual correction each tick."""
    errs = []
    for _ in range(n_episodes):
        start_idx = int(rng.integers(0, line.n))
        x, y, yaw = line.x[start_idx], line.y[start_idx], line.psi[start_idx]
        v = float(line.vx[start_idx])
        plant.reset(x, y, yaw, v)
        baseline = PurePursuitPlanner(PURE_PURSUIT_PARAMS, KART, line.rows)
        steer_hist = [0.0] * NUM_STEER_HIST
        throttle_hist = [0.0] * NUM_THROTTLE_HIST
        abs_d_hist = []

        for _ in range(n_steps):
            s, d, j, psi_track, kappa = line.frenet(x, y)
            heading_err = _wrap(yaw - psi_track)
            v_s, v_d = v * math.cos(heading_err), v * math.sin(heading_err)

            base_inputs = PlannerInputs(
                pose_xy=(x, y), yaw_rad=yaw, speed_mps=v, track_angles=None, now_ns=0)
            target_base, steer_base_deg = baseline.plan(base_inputs)

            if W is not None:
                nom_x, nom_y = nominal_rollout(plant, target_base, steer_base_deg, dt, nom_steps)
                nom_s, nom_d, _, _, _ = line.frenet(nom_x, nom_y)
                phi = _features(d, v_s, v_d, kappa, tuple(steer_hist), tuple(throttle_hist),
                                nom_s - s, nom_d - d)
                dsteer = float(phi @ W)
            else:
                dsteer = 0.0

            steer_cmd_deg = steer_base_deg + dsteer
            x, y, yaw, v = plant.step(target_base, steer_cmd_deg, dt)

            _, d_new, _, _, _ = line.frenet(x, y)
            abs_d_hist.append(abs(d_new))
            steer_hist = [math.radians(steer_cmd_deg)] + steer_hist[:-1]
            throttle_hist = [0.0] + throttle_hist[:-1]

            if abs(d_new) > OFF_TRACK_D:
                break

        errs.append(np.mean(abs_d_hist) if abs_d_hist else float("nan"))
    return float(np.nanmean(errs))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plant-meta", default=os.path.join(REPO, "sim/model/plant_linear.json"))
    ap.add_argument("--line", default=os.path.join(REPO, "data/racing_line/center.csv"))
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--episode-seconds", type=float, default=3.0)
    ap.add_argument("--eval-episodes", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    line = RacingLine(args.line)
    plant = LearnedPlant.from_linear(args.plant_meta)
    dt = plant.dt
    n_steps = int(round(args.episode_seconds / dt))
    nom_steps = max(1, int(round(0.5 / dt)))

    print(f"plant: {args.plant_meta} (hz={1.0/dt:.0f})")
    print(f"line: {args.line} ({line.n} pts)")
    print(f"collecting oracle dataset: {args.episodes} episodes x {n_steps} ticks, "
          f"grid={len(STEER_GRID)} (steer-only; speed pinned to baseline)")

    X, Y = collect_oracle_dataset(plant, line, dt, n_steps, nom_steps, args.episodes, rng)
    n = len(X)
    nonzero_frac = float(np.mean(Y != 0.0))
    print(f"collected {n} (features, oracle_steer_corr) pairs")
    print(f"oracle picked nonzero correction on {nonzero_frac:.1%} of ticks "
          f"(if this is near 0%, the baseline is already near-optimal at this "
          f"grid resolution and there's little room for any residual to help)")
    print(f"oracle steer_corr: mean={Y.mean():+.3f} std={Y.std():.3f} deg "
          f"(std==0 would mean the same grid point always wins — a red flag, "
          f"not real state-dependent signal; see the reward-hacking note above)")

    W, r2 = fit_linear(X, Y)
    print(f"linear regression R^2: {r2:.4f}")
    print("(near 0 = the oracle correction is basically unpredictable from "
          "these features with a linear map; well above 0 = real signal "
          "exists and RL's training method, not the concept, is the "
          "likely problem)")

    print("evaluating baseline vs. oracle-fitted-regression as a fixed policy, at "
          "a few gains (see docs/rl_residual_plan.md's firing 7 log for why this "
          "matters: the oracle solves a held-for-0.5s open-loop question, and "
          "naively reapplying its full magnitude every tick in closed loop is "
          "an over-correction/instability risk distinct from whether the "
          "underlying direction is right)...")
    baseline_err = evaluate_fixed_policy(plant, line, dt, n_steps, nom_steps,
                                          args.eval_episodes, rng, W=None)
    print(f"baseline mean|d|: {baseline_err:.4f} m")
    best_gain, best_err = None, float("inf")
    for gain in (1.0, 0.3, 0.1, 0.03, 0.01):
        err = evaluate_fixed_policy(plant, line, dt, n_steps, nom_steps,
                                     args.eval_episodes, rng, W=W * gain)
        delta = baseline_err - err
        print(f"  gain={gain:<5} mean|d|={err:.4f} m  delta={delta:+.4f} m "
              f"({'improved' if delta > 0 else 'regressed'})")
        if err < best_err:
            best_gain, best_err = gain, err
    print(f"best gain tried: {best_gain} (mean|d|={best_err:.4f} m, "
          f"delta={baseline_err - best_err:+.4f} m)")


if __name__ == "__main__":
    main()
