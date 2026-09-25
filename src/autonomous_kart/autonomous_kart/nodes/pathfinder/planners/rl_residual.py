"""RL residual learner (docs/rl_residual_plan.md).

A small linear-Gaussian policy over a residual correction to (steer_deg,
accel_mps2), trained via REINFORCE (Gaussian policy gradient). Deliberately
simple — this corrects a planner's output, it doesn't replace it, so it
doesn't need to be a large network. Self-contained, no ROS dependency (same
as mpc_residual.py's building blocks), so it's directly unit-testable.
Mirrors mpc_residual.ResidualLearner's public shape (mode gating, act/
update).

Where this actually runs, and why update() behaves differently in each:

  Live (MPCPlanner, mpc.py): act() is called every tick, in every mode,
  purely for telemetry (rl_residual/status -> the frontend HUD). update()
  is deliberately NEVER called from live ticks, in ANY mode — nothing
  currently blends this correction into the executed command, so a live
  reward would be statistically independent of the sampled action and the
  policy-gradient estimate would be zero in expectation (see the plan
  doc's "reward-validity finding"). That's a bigger, separate decision for
  a physical vehicle, not something to wire up implicitly here.

  Offline (sim/train_rl_residual.py): act() AND update() both get called
  against sim/learned_plant.py's learned dynamics model, where the
  correction genuinely is applied to what the plant simulates next, so the
  reward is causally valid. This is where real training happens.
  sim/rl_residual_signal_check.py separately confirmed real linear signal
  exists this way (R^2=0.75 fitting a one-shot oracle correction) — the
  concept works. Actual RL training still doesn't cleanly beat baseline
  end-to-end as of the last firing (see the plan doc); the closed_loop_gain
  fix below addressed the *mechanism* (full-magnitude per-tick
  reapplication of a correction computed for a 0.5s hold is a stability
  problem, not a signal problem) but RL's own sample efficiency in that
  now-smaller-signal regime is still open.

Mode semantics (mirrors mpc.residual.mode):
  off    - act() returns a zero residual to the caller; update() is a
           no-op. (Internally, act() still computes what the policy would
           propose and records it via last_proposed, matching the "report
           what it wants to do" ask even in this mode — it just doesn't
           hand that value to the caller.)
  shadow - act() returns what the policy wants; the caller must not apply
           it to a real/simulated command in this mode. update() trains if
           called, but nothing in this codebase calls update() in shadow
           mode today (see above) — it exists for a future caller that
           wants to train from something other than mpc.py's live ticks or
           the offline sim script.
  apply  - same as shadow from this class's point of view (act()/update()
           behave identically); the caller is expected to actually use the
           correction. Enforcing that split is the caller's job, not
           this class's — see MPCPlanner and sim/train_rl_residual.py for
           the two different ways that plays out in practice.
"""
import math
from typing import Optional, Tuple

import numpy as np

MODES = ("off", "shadow", "apply")

# Action = residual correction to (steer_deg, accel_mps2).
ACTION_DIM = 2


class RLResidualLearner:
    def __init__(self, params: dict, feature_dim: int, logger=None):
        self.params = params
        self.logger = logger
        self.feature_dim = feature_dim

        self.mode = str(params.get("mode", "off"))
        if self.mode not in MODES:
            self.mode = "off"

        self.lr = float(params.get("learning_rate", 1e-3))
        init_sigma_steer = float(params.get("init_sigma_steer_deg", 0.5))
        init_sigma_accel = float(params.get("init_sigma_accel_mps2", 0.1))
        # Clip the correction itself, independent of how confident the
        # policy is about it — a bad gradient step should never be able to
        # propose an arbitrarily large residual.
        self.max_steer_correction_deg = float(params.get("max_steer_correction_deg", 3.0))
        self.max_accel_correction_mps2 = float(params.get("max_accel_correction_mps2", 0.5))
        self.min_sigma = float(params.get("min_sigma", 1e-3))
        # Reward baseline (running mean) + gradient clip. Without these,
        # REINFORCE on an always-negative reward (a tracking-cost penalty
        # never crosses zero) has a systematically-signed but uninformative
        # gradient and diverges — confirmed empirically in
        # sim/train_rl_residual.py: weight norms hit >3000 and mean |d|
        # regressed after 300 episodes before this was added. Baseline
        # subtraction is the standard, unbiased (any action-independent
        # baseline is unbiased for REINFORCE) fix; the grad-norm clip is a
        # cheap extra safety net given how badly it diverged without one.
        self.reward_baseline_beta = float(params.get("reward_baseline_beta", 0.01))
        self.max_grad_norm = float(params.get("max_grad_norm", 50.0))
        self._reward_baseline = 0.0
        # Reward *scale* normalization, on top of the mean-subtraction
        # above — added after firing 5's training run still diverged
        # (weight norms >2500) with a whitened, mean-baselined reward. The
        # reward here is a squared tracking-error penalty, which is
        # heavily heavy-tailed (a near-boundary tick can be ~600x larger
        # than a well-tracked one); subtracting the mean alone doesn't tame
        # that variance, so it still dominates the update. z-scoring
        # (divide by running std too) is the standard companion to mean
        # baselining for exactly this reason.
        self.min_reward_std = float(params.get("min_reward_std", 1e-3))
        self._reward_var = 1.0

        # Online feature whitening (EMA mean/var, not a one-shot fit like
        # sim/plant_dataset.py's Whitener — this also has to work for a
        # never-ending live stream, and EMA tracks a shifting distribution
        # as the policy changes instead of "freezing" like a fit-once or
        # count-averaged normalizer would). Added after firing 3's offline
        # training run diverged even with a reward baseline + grad clip:
        # _features()'s 14 raw values span wildly different scales (d in
        # meters, v_s/v_d in m/s, kappa ~1e-3, history terms in
        # radians/m·s^-1 deltas), which a single global learning rate can't
        # handle well. feature_dim's index `bias_index` is excluded — it's
        # the constant 1.0 bias term in _features()'s layout and normalizing
        # a constant divides by ~0 variance.
        self.feat_norm_beta = float(params.get("feat_norm_beta", 0.001))
        self.min_feat_var = float(params.get("min_feat_var", 1e-6))
        self.bias_index = int(params.get("bias_index", 0))
        self._feat_mean = np.zeros(feature_dim, dtype=np.float64)
        self._feat_var = np.ones(feature_dim, dtype=np.float64)
        self._feat_norm_initialized = False

        # Closed-loop gain — the actual root cause found in firing 7, after
        # the reward/feature normalization work above turned out not to be
        # the real problem. sim/rl_residual_signal_check.py confirmed real
        # linear residual signal exists (R^2=0.75 fitting a one-shot,
        # held-for-0.5s oracle correction) but applying that correction at
        # full magnitude *every tick* in closed loop regressed tracking —
        # each tick's correction is computed as if starting a fresh 0.5s
        # hold, so reapplying it every 16.7ms compounds into an
        # over-correction/instability problem unrelated to whether the
        # direction is right. A gain sweep on the same fitted direction
        # went 1.0 (-0.297m) -> 0.3 (-0.141m) -> 0.1 (-0.064m) -> 0.03
        # (+0.047m, improved) -> 0.01 (+0.003m). Default follows that
        # empirical result, but it's specific to that exact setup
        # (PurePursuit baseline, center.csv, plant_linear.json) — treat as
        # a starting point to re-tune per actual deployment, not a proven
        # universal constant. Applied to the raw sampled action, before
        # the max-correction clip below (so max_steer_correction_deg still
        # means exactly what it says about the final applied value).
        self.closed_loop_gain = float(params.get("closed_loop_gain", 0.03))

        self.weights = np.zeros((ACTION_DIM, feature_dim), dtype=np.float64)
        self.log_sigma = np.log(np.array([init_sigma_steer, init_sigma_accel]))

        self.samples_trained = 0
        self._last_features: Optional[np.ndarray] = None
        self._last_mean: Optional[np.ndarray] = None
        self._last_noise: Optional[np.ndarray] = None
        self._last_sigma: Optional[np.ndarray] = None
        # What the policy most recently proposed, for telemetry (Phase 5)
        # regardless of mode — "record what it wants to do" applies even
        # in shadow/off.
        self.last_proposed: Tuple[float, float] = (0.0, 0.0)

    def set_mode(self, mode: str) -> bool:
        if mode not in MODES:
            return False
        self.mode = mode
        return True

    @property
    def sigma(self) -> np.ndarray:
        return np.exp(self.log_sigma)

    def _whiten(self, features: np.ndarray) -> np.ndarray:
        """Update the running normalizer with this observation, then return
        the whitened features (bias_index passed through unchanged)."""
        if not self._feat_norm_initialized:
            self._feat_mean = features.copy()
            self._feat_norm_initialized = True
        else:
            delta = features - self._feat_mean
            self._feat_mean = self._feat_mean + self.feat_norm_beta * delta
            self._feat_var = (
                (1.0 - self.feat_norm_beta) * self._feat_var
                + self.feat_norm_beta * delta * delta
            )
        std = np.sqrt(np.maximum(self._feat_var, self.min_feat_var))
        whitened = (features - self._feat_mean) / std
        whitened[self.bias_index] = features[self.bias_index]
        return whitened

    def act(self, features: np.ndarray, explore: bool = True) -> Tuple[float, float]:
        """Returns (steer_correction_deg, accel_correction_mps2).

        Always computes and records what the policy wants, even in `off`
        mode, but `off` returns a zero residual to the caller — the policy
        itself doesn't get consulted for the live-driving path in `off`.
        """
        features = np.asarray(features, dtype=np.float64).reshape(-1)
        if features.shape[0] != self.feature_dim:
            raise ValueError(
                f"expected {self.feature_dim} features, got {features.shape[0]}")
        features = self._whiten(features)

        mean = self.weights @ features
        sigma = self.sigma
        if explore and self.mode != "off":
            noise = np.random.normal(0.0, sigma)
        else:
            noise = np.zeros(ACTION_DIM)
        raw_action = mean + noise
        # Damped by closed_loop_gain before clipping — see __init__ comment.
        # The gradient below still uses the unscaled `mean`/`noise`, which
        # remains correct: this is a deterministic transform of the sampled
        # action, not a different distribution, so the score-function
        # estimator for the *sampled* action's log-prob is unaffected by
        # what deterministic function of it the reward is a consequence of.
        gained_action = self.closed_loop_gain * raw_action

        clipped = np.array([
            np.clip(gained_action[0], -self.max_steer_correction_deg, self.max_steer_correction_deg),
            np.clip(gained_action[1], -self.max_accel_correction_mps2, self.max_accel_correction_mps2),
        ])

        self._last_features = features
        self._last_mean = mean
        self._last_noise = noise
        self._last_sigma = sigma
        self.last_proposed = (float(clipped[0]), float(clipped[1]))

        if self.mode == "off":
            return (0.0, 0.0)
        return self.last_proposed

    def update(self, reward: float) -> None:
        """REINFORCE step from the last act() call's (features, noise) and
        an observed scalar reward. No-op if act() hasn't been called yet or
        mode is off."""
        if self.mode == "off":
            return
        if self._last_features is None or not math.isfinite(reward):
            return

        # Gaussian policy gradient: d/dW log pi(a|s) = (noise / sigma^2) * features^T
        # z-scored advantage (mean-subtracted, std-normalized) — see
        # __init__ comments on _reward_baseline and _reward_var for why
        # both matter, not just the mean.
        centered = reward - self._reward_baseline
        reward_std = math.sqrt(max(self._reward_var, self.min_reward_std ** 2))
        advantage = centered / reward_std
        sigma2 = np.maximum(self._last_sigma, self.min_sigma) ** 2
        grad_w = np.outer(self._last_noise / sigma2, self._last_features)
        step = self.lr * advantage * grad_w
        step_norm = float(np.linalg.norm(step))
        if step_norm > self.max_grad_norm:
            step *= self.max_grad_norm / step_norm
        self.weights += step

        self._reward_baseline += self.reward_baseline_beta * centered
        self._reward_var = (
            (1.0 - self.reward_baseline_beta) * self._reward_var
            + self.reward_baseline_beta * centered * centered
        )
        self.samples_trained += 1
        self._last_features = None  # each update consumes exactly one act()

    def state_dict(self) -> dict:
        return {
            "weights": self.weights.copy(),
            "log_sigma": self.log_sigma.copy(),
            "samples_trained": self.samples_trained,
            "feat_mean": self._feat_mean.copy(),
            "feat_var": self._feat_var.copy(),
            "feat_norm_initialized": self._feat_norm_initialized,
            "reward_baseline": self._reward_baseline,
            "reward_var": self._reward_var,
        }

    def load_state_dict(self, state: dict) -> None:
        self.weights = np.array(state["weights"], dtype=np.float64)
        self.log_sigma = np.array(state["log_sigma"], dtype=np.float64)
        self.samples_trained = int(state.get("samples_trained", 0))
        if "feat_mean" in state:
            self._feat_mean = np.array(state["feat_mean"], dtype=np.float64)
            self._feat_var = np.array(state["feat_var"], dtype=np.float64)
            self._feat_norm_initialized = bool(state.get("feat_norm_initialized", True))
        if "reward_baseline" in state:
            self._reward_baseline = float(state["reward_baseline"])
            self._reward_var = float(state.get("reward_var", 1.0))
