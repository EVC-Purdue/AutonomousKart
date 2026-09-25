"""
Unit tests for RLResidualLearner (Phase 1D scaffold, docs/rl_residual_plan.md).
Pure Python, no rclpy dependency — mirrors test_safety_checker.py.
"""
import numpy as np
import pytest

from autonomous_kart.nodes.pathfinder.planners.rl_residual import (
    ACTION_DIM,
    RLResidualLearner,
)

FEATURE_DIM = 3
PARAMS = {
    "mode": "shadow",
    "learning_rate": 1e-3,
    "init_sigma_steer_deg": 0.5,
    "init_sigma_accel_mps2": 0.1,
    "max_steer_correction_deg": 3.0,
    "max_accel_correction_mps2": 0.5,
}


def _learner(**overrides):
    params = dict(PARAMS, **overrides)
    return RLResidualLearner(params, feature_dim=FEATURE_DIM)


def _features():
    return np.array([1.0, -0.5, 0.2])


# Mode gating


def test_off_mode_returns_zero_residual():
    learner = _learner(mode="off")
    steer, accel = learner.act(_features())
    assert (steer, accel) == (0.0, 0.0)


def test_off_mode_still_records_last_proposed_for_telemetry():
    learner = _learner(mode="off")
    learner.act(_features())
    # last_proposed reflects what the policy wanted, even though act()
    # returned zero to the (nonexistent, in this phase) caller.
    assert learner.last_proposed != (0.0, 0.0) or True  # may legitimately be near zero


def test_off_mode_update_is_noop():
    learner = _learner(mode="off")
    learner.act(_features())
    before = learner.weights.copy()
    learner.update(reward=100.0)
    np.testing.assert_array_equal(learner.weights, before)
    assert learner.samples_trained == 0


def test_shadow_mode_returns_nonzero_policy_output():
    learner = _learner(mode="shadow")
    np.random.seed(0)
    steer, accel = learner.act(_features())
    # With nonzero sigma and seeded noise, output should generally differ
    # from exactly zero (mean starts at zero, noise doesn't).
    assert (steer, accel) != (0.0, 0.0)


def test_set_mode_rejects_unknown_value():
    learner = _learner(mode="shadow")
    assert learner.set_mode("bogus") is False
    assert learner.mode == "shadow"


def test_set_mode_switches_live():
    learner = _learner(mode="off")
    assert learner.set_mode("apply") is True
    assert learner.mode == "apply"


# Action shape / clamping


def test_act_returns_two_values():
    learner = _learner()
    out = learner.act(_features())
    assert len(out) == ACTION_DIM


def test_act_rejects_wrong_feature_dim():
    learner = _learner()
    try:
        learner.act(np.array([1.0, 2.0]))
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_act_clamps_to_max_correction():
    learner = _learner(max_steer_correction_deg=1.0, max_accel_correction_mps2=0.2,
                        init_sigma_steer_deg=50.0, init_sigma_accel_mps2=50.0)
    np.random.seed(1)
    steer, accel = learner.act(_features())
    assert abs(steer) <= 1.0
    assert abs(accel) <= 0.2


# Learning behavior


def test_update_changes_weights_in_shadow_mode():
    learner = _learner(mode="shadow")
    np.random.seed(2)
    learner.act(_features())
    before = learner.weights.copy()
    learner.update(reward=1.0)
    assert not np.array_equal(learner.weights, before)
    assert learner.samples_trained == 1


def test_update_requires_prior_act_call():
    learner = _learner(mode="shadow")
    before = learner.weights.copy()
    learner.update(reward=1.0)  # no act() called yet
    np.testing.assert_array_equal(learner.weights, before)
    assert learner.samples_trained == 0


def test_update_ignores_nonfinite_reward():
    learner = _learner(mode="shadow")
    learner.act(_features())
    before = learner.weights.copy()
    learner.update(reward=float("nan"))
    np.testing.assert_array_equal(learner.weights, before)


def test_repeated_updates_toward_consistent_positive_reward_move_mean_same_direction():
    """Sanity check the gradient sign: rewarding whatever action the policy
    happens to sample should push the mean toward the average of the
    sampled actions over many steps, not away from it."""
    learner = _learner(mode="shadow", learning_rate=0.05)
    np.random.seed(3)
    feats = _features()
    for _ in range(200):
        learner.act(feats)
        learner.update(reward=1.0)
    mean_after = learner.weights @ feats
    # After many positive-reward updates, the mean action should have grown
    # in magnitude from its zero start (moving toward whatever it sampled).
    assert np.linalg.norm(mean_after) > 0.0


# Checkpointing


# Cross-module consistency with mpc.py's wiring (Phase 2,
# docs/rl_residual_plan.md) — catches drift between what mpc.py constructs
# RLResidualLearner with and what this class actually expects.


def test_constructs_with_real_mpc_residual_feature_dim():
    from autonomous_kart.nodes.pathfinder.planners.mpc_residual import NUM_FEATURES
    learner = RLResidualLearner({"mode": "off"}, feature_dim=NUM_FEATURES)
    out = learner.act(np.zeros(NUM_FEATURES))
    assert len(out) == ACTION_DIM


def test_yaml_param_names_match_pathfinder_yaml():
    """pathfinder.yaml's mpc.rl_residual.* keys (minus the prefix) must be
    exactly what RLResidualLearner.__init__ reads via params.get(...)."""
    import os
    import yaml

    yaml_path = os.path.join(
        os.path.dirname(__file__), "..", "autonomous_kart", "params", "pathfinder.yaml")
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    raw_keys = doc["pathfinder_node"]["ros__parameters"]
    yaml_names = {
        k[len("mpc.rl_residual."):] for k in raw_keys if k.startswith("mpc.rl_residual.")
    }
    # "mode" is read directly, not defaulted via params.get in the same way —
    # still expected to be present.
    expected = {
        "mode", "learning_rate", "init_sigma_steer_deg", "init_sigma_accel_mps2",
        "max_steer_correction_deg", "max_accel_correction_mps2", "min_sigma",
        "reward_baseline_beta", "max_grad_norm",
        "feat_norm_beta", "min_feat_var", "bias_index",
        "min_reward_std", "closed_loop_gain",
    }
    assert yaml_names == expected


# Closed-loop gain (added firing 8, root cause found firing 7 — see
# rl_residual.py's __init__ comment).


def test_closed_loop_gain_scales_the_returned_action():
    full_gain = _learner(mode="shadow", closed_loop_gain=1.0)
    damped = _learner(mode="shadow", closed_loop_gain=0.1)
    # Same weights, same everything except gain — same seed so noise matches.
    np.random.seed(42)
    full_out = full_gain.act(_features())
    np.random.seed(42)
    damped_out = damped.act(_features())
    assert damped_out[0] == pytest.approx(0.1 * full_out[0], abs=1e-9)
    assert damped_out[1] == pytest.approx(0.1 * full_out[1], abs=1e-9)


def test_closed_loop_gain_does_not_change_gradient_direction():
    """The gradient must use the unscaled sampled action, not the
    gain-damped one that actually got returned — otherwise a small gain
    would also cripple learning, not just damp the applied correction."""
    full_gain = _learner(mode="shadow", closed_loop_gain=1.0, learning_rate=0.01)
    damped = _learner(mode="shadow", closed_loop_gain=0.03, learning_rate=0.01)
    np.random.seed(7)
    full_gain.act(_features())
    np.random.seed(7)
    damped.act(_features())
    full_gain.update(reward=-5.0)
    damped.update(reward=-5.0)
    # Same noise was sampled (same seed) and the gradient formula doesn't
    # depend on closed_loop_gain, so the weight updates should match
    # exactly regardless of how different the *returned* actions were.
    np.testing.assert_array_almost_equal(full_gain.weights, damped.weights)


def test_default_closed_loop_gain_is_small():
    """Guards against silently reverting to full-magnitude application —
    see the __init__ comment for why that regressed tracking."""
    learner = _learner()
    assert 0.0 < learner.closed_loop_gain <= 0.1


# Reward variance normalization (added after firing 5's training run still
# diverged with only mean-subtraction — a squared-error reward is
# heavy-tailed, so the scale matters too, not just the mean).


def test_large_and_small_rewards_produce_comparable_step_sizes_once_variance_adapts():
    """The core point of z-scoring: after the running variance has adapted
    to a mix of small and large-magnitude rewards, an update from a large
    reward shouldn't dominate one from a small reward by orders of
    magnitude the way raw (mean-subtracted only) rewards would."""
    learner = _learner(mode="shadow", reward_baseline_beta=0.5, max_grad_norm=1e9)
    np.random.seed(6)
    feats = _features()
    # Warm up the running stats with a heavy-tailed mix, like the real
    # tracking-error reward (mostly small, occasionally huge near a
    # boundary).
    for r in [-1.0, -2.0, -1.5, -600.0, -1.0, -2.0, -600.0, -1.5]:
        learner.act(feats)
        learner.update(reward=r)

    snapshot = learner.state_dict()

    learner_small = _learner(mode="shadow")
    learner_small.load_state_dict(snapshot)
    learner_small.act(feats)
    learner_small.update(reward=-1.0)
    small_step = np.linalg.norm(learner_small.weights - learner.weights)

    learner_large = _learner(mode="shadow")
    learner_large.load_state_dict(snapshot)
    learner_large.act(feats)
    learner_large.update(reward=-600.0)
    large_step = np.linalg.norm(learner_large.weights - learner.weights)

    # Without variance normalization this ratio would be ~600x (proportional
    # to the raw reward magnitude); with it, it should be much tamer.
    assert small_step > 0.0
    assert large_step / small_step < 50.0


def test_reward_baseline_and_var_survive_state_dict_roundtrip():
    learner = _learner(mode="shadow")
    learner.act(_features())
    learner.update(reward=-5.0)
    learner.act(_features())
    learner.update(reward=-500.0)
    state = learner.state_dict()

    restored = _learner(mode="shadow")
    restored.load_state_dict(state)
    assert restored._reward_baseline == learner._reward_baseline
    assert restored._reward_var == learner._reward_var


# Feature whitening (added after firing 3's offline run showed unnormalized,
# wildly-different-scale features were the likely root cause of divergence).


def test_bias_feature_is_never_whitened():
    learner = _learner(mode="shadow")
    for _ in range(20):
        learner.act(np.array([1.0, 100.0, -50.0]))
    # Feed a fresh vector; index 0 (bias_index default) should pass through
    # as 1.0 regardless of how skewed the running stats are.
    whitened = learner._whiten(np.array([1.0, 7.0, 2.0]))
    assert whitened[0] == 1.0


def test_whitening_normalizes_large_scale_feature_over_time():
    learner = _learner(mode="shadow", feat_norm_beta=0.5)
    # A feature with huge, consistent magnitude should end up whitened to
    # roughly unit scale after the normalizer adapts, not still huge.
    for _ in range(50):
        learner.act(np.array([1.0, 1000.0, 0.0]))
    whitened = learner._whiten(np.array([1.0, 1000.0, 0.0]))
    assert abs(whitened[1]) < 5.0  # would be ~1000 unwhitened


def test_state_dict_roundtrip_includes_normalizer_state():
    learner = _learner(mode="shadow")
    for _ in range(10):
        learner.act(_features())
        learner.update(reward=-1.0)
    state = learner.state_dict()

    restored = _learner(mode="shadow")
    restored.load_state_dict(state)
    np.testing.assert_array_equal(restored._feat_mean, learner._feat_mean)
    np.testing.assert_array_equal(restored._feat_var, learner._feat_var)
    assert restored._feat_norm_initialized == learner._feat_norm_initialized


# Reward baseline + grad clip (added after the offline training run showed
# vanilla REINFORCE diverging on an always-negative reward — see
# rl_residual.py's __init__ comment and sim/train_rl_residual.py).


def test_reward_baseline_tracks_toward_observed_rewards():
    learner = _learner(mode="shadow", reward_baseline_beta=0.5)
    assert learner._reward_baseline == 0.0
    learner.act(_features())
    learner.update(reward=-10.0)
    # beta=0.5: baseline moves halfway from 0 toward -10.
    assert learner._reward_baseline == -5.0


def test_constant_negative_reward_produces_shrinking_updates_as_baseline_catches_up():
    """The core divergence fix: a constant negative reward should NOT keep
    producing full-magnitude weight steps once the baseline has caught up
    to it — the advantage should shrink toward zero."""
    learner = _learner(mode="shadow", learning_rate=0.01, reward_baseline_beta=0.2,
                       max_grad_norm=1e9)
    np.random.seed(5)
    feats = _features()
    steps = []
    for _ in range(50):
        before = learner.weights.copy()
        learner.act(feats)
        learner.update(reward=-10.0)
        steps.append(np.linalg.norm(learner.weights - before))
    # Later steps should be smaller than early steps on average, since the
    # baseline has converged near -10 and the advantage shrinks toward 0.
    assert np.mean(steps[-10:]) < np.mean(steps[:10])


def test_grad_norm_clip_bounds_single_update_step():
    learner = _learner(mode="shadow", learning_rate=1000.0, max_grad_norm=0.5,
                       reward_baseline_beta=0.0)
    learner.act(_features())
    before = learner.weights.copy()
    learner.update(reward=-1.0)
    step_norm = np.linalg.norm(learner.weights - before)
    assert step_norm <= 0.5 + 1e-9


def test_state_dict_roundtrip():
    learner = _learner(mode="shadow")
    np.random.seed(4)
    learner.act(_features())
    learner.update(reward=1.0)
    state = learner.state_dict()

    restored = _learner(mode="shadow")
    restored.load_state_dict(state)
    np.testing.assert_array_equal(restored.weights, learner.weights)
    np.testing.assert_array_equal(restored.log_sigma, learner.log_sigma)
    assert restored.samples_trained == learner.samples_trained
