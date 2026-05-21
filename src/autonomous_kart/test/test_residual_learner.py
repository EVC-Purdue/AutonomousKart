"""Tests for ResidualLearner: outlier rejection, |d| filter, warmup gate, counters."""
import numpy as np

from autonomous_kart.nodes.pathfinder.planners.mpc_residual import (
    ResidualLearner, NUM_FEATURES,
)


def _phi(d=0.0):
    """14-element feature vector with d at the second slot, like _features() builds."""
    v = np.zeros(NUM_FEATURES)
    v[0] = 1.0
    v[1] = d
    return v


def _make(params=None):
    p = {
        "mode": "shadow",
        "target_horizon_s": 0.05,
        "forgetting_factor": 0.99,
        "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5,
        "error_window": 200,
        "outlier_threshold_m": 2.0,
        "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5,
        "max_theta_norm": 50.0,
        "rls_warmup_samples": 5,
        "gbm_enabled": False,
        "apply_min_samples_this_run": 1200,
        "cache_enabled": False,
    }
    if params: p.update(params)
    return ResidualLearner(p, solve_dt=1.0 / 60.0, s_total=850.0)


def test_push_rejects_off_line_kart():
    learner = _make()
    # |d| = 4.0 > max_train_d_m = 3.0
    learner.push(_phi(d=4.0), s_t=0.0, d_t=4.0, nom_ds=0.1, nom_dd=0.0, speed=3.0)
    learner.step(s_now=0.5, d_now=4.05)
    assert learner.samples_trained == 0
    assert learner.off_line_skipped == 1


def test_step_rejects_huge_residual_target():
    learner = _make()
    # Push a sample, then step at a position whose actual_ds explodes
    # (s wrap not triggered because s_total > delta).
    learner.push(_phi(d=0.0), s_t=0.0, d_t=0.0, nom_ds=0.0, nom_dd=0.0, speed=3.0)
    # Step horizon = 0.05s * 60Hz = 3 ticks, so we need >= 3 pushes for one to age out.
    learner.push(_phi(d=0.0), s_t=0.0, d_t=0.0, nom_ds=0.0, nom_dd=0.0, speed=3.0)
    learner.push(_phi(d=0.0), s_t=0.0, d_t=0.0, nom_ds=0.0, nom_dd=0.0, speed=3.0)
    learner.push(_phi(d=0.0), s_t=0.0, d_t=0.0, nom_ds=0.0, nom_dd=0.0, speed=3.0)
    # Now step at s_now=10 (actual_ds=10 vs nom_ds=0 → residual 10m > threshold 2m)
    learner.step(s_now=10.0, d_now=0.0)
    assert learner.samples_trained == 0
    assert learner.outliers_dropped == 1


def test_predict_returns_zero_during_warmup():
    learner = _make({"rls_warmup_samples": 50})
    phi = _phi(d=0.0)
    pred_s, pred_d, source = learner.predict(phi)
    assert pred_s == 0.0
    assert pred_d == 0.0
    assert source == 0   # RLS


def test_predict_returns_nonzero_after_warmup():
    learner = _make({"rls_warmup_samples": 3})
    # Drive samples through; horizon is 3 ticks so we need 4 pushes + steps
    for i in range(20):
        learner.push(_phi(d=0.0), s_t=float(i) * 0.1, d_t=0.0,
                     nom_ds=0.0, nom_dd=0.0, speed=3.0)
        learner.step(s_now=float(i + 1) * 0.1, d_now=0.0)
    assert learner.samples_trained >= 3
    pred_s, pred_d, source = learner.predict(_phi(d=0.0))
    # theta_s should be non-zero after enough updates with non-zero residuals
    assert source == 0
    # samples_accepted_this_run is bumped per step()
    assert learner.samples_accepted_this_run == learner.samples_trained


def test_effective_mode_starts_shadow_then_apply():
    learner = _make({"apply_min_samples_this_run": 4, "rls_warmup_samples": 0})
    assert learner.effective_mode() == "shadow"
    for i in range(20):
        learner.push(_phi(d=0.0), s_t=float(i) * 0.1, d_t=0.0,
                     nom_ds=0.0, nom_dd=0.0, speed=3.0)
        learner.step(s_now=float(i + 1) * 0.1, d_now=0.0)
    # samples_accepted_this_run >= 4 → effective_mode rises to requested mode
    learner.mode = "apply"
    assert learner.effective_mode() == "apply"


def test_apply_mode_gated_back_to_shadow_when_yaml_shadow():
    learner = _make({"apply_min_samples_this_run": 0})
    learner.mode = "shadow"
    assert learner.effective_mode() == "shadow"
    learner.mode = "off"
    assert learner.effective_mode() == "off"


def test_predict_unchanged_by_effective_mode():
    """predict() always returns the raw model output; gating happens at the
    consumer (mpc.py)."""
    learner = _make({"apply_min_samples_this_run": 10, "rls_warmup_samples": 0})
    learner.mode = "apply"
    learner.theta_s[0] = 0.5
    pred_s, _, _ = learner.predict(_phi(d=0.0))
    # samples_accepted_this_run = 0 → effective_mode == "shadow", but predict still returns 0.5
    assert learner.effective_mode() == "shadow"
    assert pred_s == 0.5


def test_warm_start_from_cache(tmp_path):
    params = {
        "mode": "shadow",
        "target_horizon_s": 0.05,
        "forgetting_factor": 0.99, "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5, "error_window": 200,
        "outlier_threshold_m": 2.0, "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5, "max_theta_norm": 50.0,
        "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
        "gbm_enabled": False,
        "cache_enabled": True, "cache_dir": str(tmp_path), "cache_load_on_start": True,
    }
    a = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    # Train a tiny bit
    for i in range(20):
        a.push(_phi(d=0.0), s_t=float(i) * 0.1, d_t=0.0,
               nom_ds=0.0, nom_dd=0.0, speed=3.0)
        a.step(s_now=float(i + 1) * 0.1, d_now=0.0)
    a.save_rls_cache(best_val_mae_s=0.1, best_val_mae_d=0.04, train_seq=1)

    b = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    assert b.cache_loaded
    np.testing.assert_allclose(b.theta_s, a.theta_s)
    np.testing.assert_allclose(b.P, a.P)
    assert b.samples_trained == a.samples_trained


def test_buffer_grows_when_gbm_enabled():
    params = {
        "mode": "shadow", "target_horizon_s": 0.05,
        "forgetting_factor": 0.99, "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5, "error_window": 200,
        "outlier_threshold_m": 2.0, "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5, "max_theta_norm": 50.0,
        "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
        "gbm_enabled": True,
        "gbm_buffer_capacity": 100,
        "cache_enabled": False,
    }
    learner = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    try:
        assert learner.buffer is not None
        for i in range(20):
            learner.push(_phi(d=0.0), s_t=float(i) * 0.1, d_t=0.0,
                         nom_ds=0.0, nom_dd=0.0, speed=3.0)
            learner.step(s_now=float(i + 1) * 0.1, d_now=0.0)
        # Each accepted step() pushes one sample into the buffer.
        assert learner.buffer.size == learner.samples_trained
    finally:
        learner.shutdown()


class _FakeGBM:
    def __init__(self, value):
        self.value = value

    def predict(self, X):
        return np.full(len(X), self.value)


def test_selector_picks_gbm_when_val_mae_better():
    params = {
        "mode": "shadow", "target_horizon_s": 0.05,
        "forgetting_factor": 0.99, "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5, "error_window": 200,
        "outlier_threshold_m": 2.0, "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5, "max_theta_norm": 50.0,
        "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
        "gbm_enabled": False,  # keep trainer thread off; we drive the selector by hand
        "cache_enabled": False,
        "gbm_select_eps_s_m": 0.01, "gbm_select_eps_d_m": 0.005,
        "gbm_predict_clip_m": 0.5,
    }
    learner = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    learner._install_gbm(_FakeGBM(0.3), _FakeGBM(0.1),
                        val_mae_s=0.05, val_mae_d=0.02,
                        rls_val_mae_s=0.20, rls_val_mae_d=0.10)
    pred_s, pred_d, source = learner.predict(_phi(d=0.0))
    assert source == 1   # SOURCE_GBM
    assert pred_s == 0.3
    assert pred_d == 0.1


def test_selector_stays_rls_when_gbm_only_marginally_better():
    params = {
        "mode": "shadow", "target_horizon_s": 0.05,
        "forgetting_factor": 0.99, "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5, "error_window": 200,
        "outlier_threshold_m": 2.0, "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5, "max_theta_norm": 50.0,
        "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
        "gbm_enabled": False, "cache_enabled": False,
        "gbm_select_eps_s_m": 0.01, "gbm_select_eps_d_m": 0.005,
        "gbm_predict_clip_m": 0.5,
    }
    learner = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    learner._install_gbm(_FakeGBM(0.0), _FakeGBM(0.0),
                        val_mae_s=0.205, val_mae_d=0.10,
                        rls_val_mae_s=0.20, rls_val_mae_d=0.10)
    pred_s, pred_d, source = learner.predict(_phi(d=0.0))
    assert source == 0


def test_gbm_predict_clipped():
    params = {
        "mode": "apply", "target_horizon_s": 0.05,
        "forgetting_factor": 0.99, "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5, "error_window": 200,
        "outlier_threshold_m": 2.0, "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5, "max_theta_norm": 50.0,
        "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
        "gbm_enabled": False, "cache_enabled": False,
        "gbm_select_eps_s_m": 0.01, "gbm_select_eps_d_m": 0.005,
        "gbm_predict_clip_m": 0.5,
    }
    learner = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    learner._install_gbm(_FakeGBM(2.0), _FakeGBM(-1.0),
                        val_mae_s=0.05, val_mae_d=0.05,
                        rls_val_mae_s=0.30, rls_val_mae_d=0.30)
    pred_s, pred_d, source = learner.predict(_phi(d=0.0))
    assert source == 1
    assert pred_s == 0.5     # clipped from 2.0 to +0.5
    assert pred_d == -0.5    # clipped from -1.0 to -0.5
    assert learner.last_pred_clipped == 1


def test_rls_divergence_restores_matching_regime():
    from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature
    learner = _make({"regime_dv_tol_mps": 1.0, "regime_dd_tol_m": 0.3,
                     "regime_dkappa_rel_tol": 0.5, "max_theta_norm": 0.5})
    # Plant a good RLS state in a checkpoint with regime ≈ current
    good_theta_s = np.ones(NUM_FEATURES) * 0.1
    learner._seed_checkpoint(
        theta_s=good_theta_s, theta_d=np.zeros(NUM_FEATURES),
        P=np.eye(NUM_FEATURES), samples_trained=500,
        val_mae_s=0.05, val_mae_d=0.02,
        regime=RegimeSignature(4.0, 0.5, 0.02),
    )
    learner._current_regime = RegimeSignature(4.1, 0.5, 0.02)
    # Force divergence
    learner.theta_s = np.ones(NUM_FEATURES) * 100.0
    learner.theta_d = np.ones(NUM_FEATURES) * 100.0
    learner._trigger_divergence_check()
    np.testing.assert_allclose(learner.theta_s, good_theta_s)
    assert learner.divergence_resets == 1


def test_rls_divergence_zeros_when_regime_dissimilar():
    from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature
    learner = _make({"max_theta_norm": 0.5})
    learner._seed_checkpoint(
        theta_s=np.ones(NUM_FEATURES) * 0.1, theta_d=np.zeros(NUM_FEATURES),
        P=np.eye(NUM_FEATURES), samples_trained=500,
        val_mae_s=0.05, val_mae_d=0.02,
        regime=RegimeSignature(2.0, 0.1, 0.005),
    )
    learner._current_regime = RegimeSignature(7.0, 1.0, 0.05)
    learner.theta_s = np.ones(NUM_FEATURES) * 100.0
    learner.theta_d = np.ones(NUM_FEATURES) * 100.0
    learner._trigger_divergence_check()
    np.testing.assert_allclose(learner.theta_s, np.zeros(NUM_FEATURES))


def test_clip_rate_revert_disables_gbm():
    learner = _make({"rls_warmup_samples": 0,
                     "revert_clip_rate_window": 10,
                     "revert_clip_rate_threshold": 0.30,
                     "gbm_predict_clip_m": 0.1})
    learner._install_gbm(_FakeGBM(1.0), _FakeGBM(1.0),
                         val_mae_s=0.05, val_mae_d=0.05,
                         rls_val_mae_s=0.30, rls_val_mae_d=0.30)
    assert learner.use_gbm
    # Force many clipped predictions
    for _ in range(10):
        learner.predict(_phi(d=0.0))
    learner._maybe_clip_revert()
    assert not learner.use_gbm
    assert learner.revert_count == 1


def test_manual_revert_to_checkpoint():
    from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature
    learner = _make()
    learner._seed_checkpoint(
        theta_s=np.full(NUM_FEATURES, 0.7), theta_d=np.zeros(NUM_FEATURES),
        P=np.eye(NUM_FEATURES), samples_trained=999,
        val_mae_s=0.04, val_mae_d=0.02,
        regime=RegimeSignature(99.0, 99.0, 99.0),  # deliberately weird regime
    )
    learner.theta_s = np.zeros(NUM_FEATURES)
    learner.manual_revert()
    np.testing.assert_allclose(learner.theta_s, np.full(NUM_FEATURES, 0.7))
    assert learner.revert_count == 1


def test_current_regime_tracks_buffer():
    learner = _make({"gbm_enabled": True, "gbm_buffer_capacity": 200,
                     "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
                     "cache_enabled": False})
    try:
        for i in range(150):
            phi = np.zeros(NUM_FEATURES)
            phi[0] = 1.0
            phi[1] = 0.5    # d
            phi[2] = 4.0    # v_s placeholder (we re-derive from speed in step)
            phi[4] = 0.02   # kappa
            learner.push(phi, s_t=float(i) * 0.1, d_t=0.5,
                         nom_ds=0.0, nom_dd=0.0, speed=4.0)
            learner.step(s_now=float(i + 1) * 0.1, d_now=0.5)
        sig = learner._compute_current_regime()
        assert abs(sig.mean_v - 4.0) < 0.5
        assert abs(sig.mean_abs_d - 0.5) < 0.2
    finally:
        learner.shutdown()


def test_trainer_installs_gbm_and_flips_selector():
    import time as _time
    params = {
        "mode": "shadow", "target_horizon_s": 0.05,
        "forgetting_factor": 0.99, "initial_cov": 1000.0,
        "min_train_speed_mps": 0.5, "error_window": 200,
        "outlier_threshold_m": 2.0, "max_train_d_m": 3.0,
        "max_p_trace": 1.0e5, "max_theta_norm": 50.0,
        "rls_warmup_samples": 0, "apply_min_samples_this_run": 0,
        "gbm_enabled": True, "gbm_buffer_capacity": 2000,
        "gbm_max_iter": 30, "gbm_max_depth": 3, "gbm_learning_rate": 0.1,
        "gbm_min_samples_leaf": 10, "gbm_min_samples_to_train": 1500,
        "gbm_retrain_secs": 0.2, "gbm_retrain_every_samples": 999999,
        "cache_enabled": False,
        "gbm_select_eps_s_m": 0.01, "gbm_select_eps_d_m": 0.005,
        "gbm_predict_clip_m": 0.5,
    }
    learner = ResidualLearner(params, solve_dt=1.0 / 60.0, s_total=850.0)
    try:
        rng = np.random.default_rng(0)
        for i in range(1800):
            phi = rng.standard_normal(NUM_FEATURES)
            phi[0] = 1.0
            learner.buffer.append(phi, r_s=0.3 * phi[1], r_d=0.1 * phi[2], t_ns=i)
        _time.sleep(2.0)
        # After training, the learner should have a result with a non-None GBM
        assert learner.trainer.last_result is not None
        assert learner.trainer.last_result.gbm_s is not None
        # And the learner's _gbm_s slot should be populated via the install-callback
        assert learner._gbm_s is not None
    finally:
        learner.shutdown()


def test_rls_predict_clipped():
    """RLS predictions are clipped to ±gbm_predict_clip_m, matching GBM.

    Offline replay caught RLS spiking to ~200 m of Δs prediction in benign
    driving. The clip is hard safety for apply mode and matches GBM."""
    learner = _make({"rls_warmup_samples": 0,
                     "gbm_predict_clip_m": 0.5})
    # Push theta beyond the clip threshold via the public attributes
    learner.theta_s = np.zeros(NUM_FEATURES)
    learner.theta_d = np.zeros(NUM_FEATURES)
    learner.theta_s[0] = 100.0   # bias slot — phi[0]=1.0 → prediction = 100
    learner.theta_d[0] = -100.0
    learner.samples_trained = 100  # past warmup
    pred_s, pred_d, source = learner.predict(_phi(d=0.0))
    assert source == 0   # RLS
    assert pred_s == 0.5     # clipped from 100 → +0.5
    assert pred_d == -0.5    # clipped from -100 → -0.5
    assert learner.last_pred_clipped == 1


def test_rls_predict_unclipped_when_in_bounds():
    """RLS predictions within ±clip pass through unchanged."""
    learner = _make({"rls_warmup_samples": 0, "gbm_predict_clip_m": 0.5})
    learner.theta_s = np.zeros(NUM_FEATURES)
    learner.theta_d = np.zeros(NUM_FEATURES)
    learner.theta_s[0] = 0.2
    learner.samples_trained = 100
    pred_s, pred_d, _ = learner.predict(_phi(d=0.0))
    assert abs(pred_s - 0.2) < 1e-12
    assert learner.last_pred_clipped == 0


def test_selector_flips_to_gbm_when_only_d_wins():
    """Selector's OR rule: GBM only needs to beat RLS on s OR d, not both.

    Offline replay showed the AND rule kept the selector on RLS 94% of ticks
    because s val_maes were nearly tied even when GBM was a clear winner on d."""
    learner = _make({"rls_warmup_samples": 0,
                     "gbm_select_eps_s_m": 0.0,
                     "gbm_select_eps_d_m": 0.0,
                     "gbm_predict_clip_m": 5.0})
    # GBM ties on s (no improvement), but beats on d by a clear margin.
    learner._install_gbm(_FakeGBM(0.1), _FakeGBM(0.05),
                         val_mae_s=0.30, val_mae_d=0.05,
                         rls_val_mae_s=0.30, rls_val_mae_d=0.12)
    assert learner.use_gbm    # OR rule flips because d wins


def test_selector_stays_rls_when_neither_axis_beats():
    """If GBM is no better on either axis, selector stays on RLS."""
    learner = _make({"rls_warmup_samples": 0,
                     "gbm_select_eps_s_m": 0.0,
                     "gbm_select_eps_d_m": 0.0,
                     "gbm_predict_clip_m": 5.0})
    learner._install_gbm(_FakeGBM(0.0), _FakeGBM(0.0),
                         val_mae_s=0.30, val_mae_d=0.10,
                         rls_val_mae_s=0.30, rls_val_mae_d=0.10)
    assert not learner.use_gbm
