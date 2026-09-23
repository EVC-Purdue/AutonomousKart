"""Tests for `mpc.residual.model_size`: the xs..xl ladder behind one param."""
import numpy as np
import pytest

from autonomous_kart.nodes.pathfinder.planners.mpc_residual import (
    NUM_FEATURES, ResidualLearner,
)
from autonomous_kart.nodes.pathfinder.planners.residual import models


def _params(**over):
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
        "rls_warmup_samples": 2,
        "cache_enabled": False,
    }
    p.update(over)
    return p


def _phi(d=1.0, v_s=2.0):
    v = np.zeros(NUM_FEATURES)
    v[0] = 1.0
    v[1] = d
    v[2] = v_s
    return v


# --- the size param ---------------------------------------------------------
def test_yaml_carries_the_default():
    """The default lives in the YAML, not in a Python constant."""
    import yaml
    doc = yaml.safe_load(open(
        "src/autonomous_kart/autonomous_kart/params/pathfinder.yaml"))
    block = doc["pathfinder_node"]["ros__parameters"]
    assert block["mpc.residual.model_size"] == "m"
    assert "mpc.residual.gbm_enabled" not in block


def test_size_is_case_insensitive_and_trimmed():
    assert ResidualLearner(_params(model_size=" XL "), solve_dt=1.0).model_size == "xl"


@pytest.mark.parametrize("bad", ["huge", "", None, "medium"])
def test_unusable_size_is_rejected(bad):
    with pytest.raises(ValueError):
        ResidualLearner(_params(model_size=bad), solve_dt=1.0)


def test_gbm_enabled_is_gone_and_ignored():
    learner = ResidualLearner(_params(model_size="s", gbm_enabled=True),
                              solve_dt=1.0 / 60.0)
    assert learner.uses_batch_model is False
    assert learner.trainer is None
    assert not hasattr(learner, "gbm_enabled")


# --- xs masks every feature but the bias -----------------------------------
def test_xs_learns_a_constant_and_ignores_features():
    learner = ResidualLearner(_params(model_size="xs"), solve_dt=1.0 / 60.0)
    # A target that is a fixed offset plus a term the features could explain.
    for i in range(200):
        phi = _phi(d=float(i % 7), v_s=float(i % 3))
        learner.push(phi, s_t=0.0, d_t=0.0, nom_ds=0.0, nom_dd=0.0, speed=5.0)
        learner.step(s_now=0.25, d_now=0.10)
    # Everything but the bias weight must stay exactly zero.
    assert np.allclose(learner.theta_s[1:], 0.0)
    assert np.allclose(learner.theta_d[1:], 0.0)
    # And the bias must have found the offset.
    assert learner.theta_s[0] == pytest.approx(0.25, abs=0.02)
    assert learner.theta_d[0] == pytest.approx(0.10, abs=0.02)
    # The prediction cannot depend on the features it was never given.
    a = learner.predict(_phi(d=0.0, v_s=0.0))[:2]
    b = learner.predict(_phi(d=50.0, v_s=-30.0))[:2]
    assert a == b


def test_s_does_use_the_features():
    learner = ResidualLearner(_params(model_size="s"), solve_dt=1.0 / 60.0)
    rng = np.random.default_rng(0)
    # `step` pops the sample pushed `horizon_steps` ticks ago, so the target
    # has to be built from the d that far back or the pair is mismatched.
    lag = learner.horizon_steps
    hist = []
    for _ in range(300):
        d = float(rng.uniform(-1.0, 1.0))
        hist.append(d)
        learner.push(_phi(d=d), s_t=0.0, d_t=0.0, nom_ds=0.0, nom_dd=0.0,
                     speed=5.0)
        learner.step(s_now=0.3 * hist[-1 - lag] if len(hist) > lag else 0.0,
                     d_now=0.0)
    assert abs(learner.theta_s[1]) > 0.05
    a = learner.predict(_phi(d=0.0))[:2]
    b = learner.predict(_phi(d=1.0))[:2]
    assert a != b


# --- the estimator the trainer is handed -----------------------------------
def test_estimator_returns_the_named_model():
    assert type(models.estimator("m", {})()).__name__ == "HistGradientBoostingRegressor"
    for size, hidden in (("l", (64, 64)), ("xl", (128, 128))):
        pipe = models.estimator(size, {})()
        mlp = pipe.steps[-1][1]
        assert type(mlp).__name__ == "MLPRegressor"
        assert mlp.hidden_layer_sizes == hidden


def test_trainer_fits_whatever_factory_it_is_given():
    from autonomous_kart.nodes.pathfinder.planners.residual.buffer import TrainBuffer
    from autonomous_kart.nodes.pathfinder.planners.residual.trainer import GBMTrainer
    from sklearn.linear_model import Ridge

    rng = np.random.default_rng(1)
    buf = TrainBuffer(capacity=1500, feature_dim=NUM_FEATURES)
    for i in range(1500):
        phi = rng.standard_normal(NUM_FEATURES)
        buf.append(phi, r_s=float(0.4 * phi[1]), r_d=float(0.2 * phi[2]),
                   t_ns=i)
    trainer = GBMTrainer(
        buffer=buf, rls_predict_batch=lambda X: (np.zeros(len(X)), np.zeros(len(X))),
        max_iter=10, max_depth=2, learning_rate=0.1, min_samples_leaf=20,
        min_samples_to_train=1000, retrain_secs=10.0, retrain_every_samples=1000,
        estimator_factory=lambda: Ridge(alpha=1.0),
    )
    result = trainer.train_once()
    assert result is not None
    assert isinstance(result.gbm_s, Ridge)
    assert result.val_mae_s < result.rls_val_mae_s
