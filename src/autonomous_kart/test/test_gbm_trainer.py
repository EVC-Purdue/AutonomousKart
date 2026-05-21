"""Tests for GBMTrainer: synchronous train_once + threaded run loop."""
import time

import numpy as np

from autonomous_kart.nodes.pathfinder.planners.residual.buffer import TrainBuffer
from autonomous_kart.nodes.pathfinder.planners.residual.trainer import (
    GBMTrainer, TrainResult,
)


def _make_buffer(n=2000, dim=14, seed=0):
    rng = np.random.default_rng(seed)
    buf = TrainBuffer(capacity=n, feature_dim=dim)
    for i in range(n):
        phi = rng.standard_normal(dim)
        # target_s is a nonlinear function so the GBM has something to learn
        r_s = 0.3 * phi[1] - 0.2 * phi[2] * phi[3] + 0.1 * rng.standard_normal()
        r_d = 0.1 * phi[4] ** 2 + 0.05 * rng.standard_normal()
        buf.append(phi, r_s=float(r_s), r_d=float(r_d), t_ns=i)
    return buf


def _rls_predict(X):
    """Stub RLS predictor that returns zeros (so GBM clearly beats it)."""
    return np.zeros(len(X)), np.zeros(len(X))


def test_train_once_happy_path():
    buf = _make_buffer(n=2000)
    trainer = GBMTrainer(
        buffer=buf,
        rls_predict_batch=_rls_predict,
        max_iter=60, max_depth=3, learning_rate=0.1, min_samples_leaf=20,
        min_samples_to_train=1200, retrain_secs=10.0, retrain_every_samples=1200,
    )
    result = trainer.train_once()
    assert isinstance(result, TrainResult)
    assert result.gbm_s is not None
    assert result.gbm_d is not None
    assert np.isfinite(result.val_mae_s)
    assert np.isfinite(result.rls_val_mae_s)
    # GBM should beat the zero baseline
    assert result.val_mae_s < result.rls_val_mae_s


def test_train_once_returns_none_with_too_few_samples():
    buf = _make_buffer(n=100)
    trainer = GBMTrainer(
        buffer=buf, rls_predict_batch=_rls_predict,
        max_iter=60, max_depth=3, learning_rate=0.1, min_samples_leaf=20,
        min_samples_to_train=1200, retrain_secs=10.0, retrain_every_samples=1200,
    )
    assert trainer.train_once() is None


def test_train_once_catches_nan_samples():
    buf = _make_buffer(n=2000)
    # Poison the buffer with NaNs
    X, ys, yd, _ = buf.snapshot()
    X[0, 0] = float("nan")
    # Re-insert by clearing/reappending wouldn't be clean; instead test that
    # GBMTrainer drops non-finite rows internally.
    buf2 = TrainBuffer(capacity=2000, feature_dim=14)
    for i in range(len(X)):
        buf2.append(X[i], r_s=float(ys[i]), r_d=float(yd[i]), t_ns=i)
    trainer = GBMTrainer(
        buffer=buf2, rls_predict_batch=_rls_predict,
        max_iter=60, max_depth=3, learning_rate=0.1, min_samples_leaf=20,
        min_samples_to_train=1200, retrain_secs=10.0, retrain_every_samples=1200,
    )
    result = trainer.train_once()
    assert result is not None  # NaN row was filtered, train succeeded
    assert np.isfinite(result.val_mae_s)


def test_thread_lifecycle():
    buf = _make_buffer(n=2000)
    trainer = GBMTrainer(
        buffer=buf, rls_predict_batch=_rls_predict,
        max_iter=20, max_depth=3, learning_rate=0.1, min_samples_leaf=20,
        min_samples_to_train=1200, retrain_secs=0.1, retrain_every_samples=999999,
    )
    trainer.start()
    time.sleep(0.5)   # allow at least one train pass
    assert trainer.train_seq >= 1
    trainer.stop(timeout=2.0)
    assert not trainer.is_alive()
