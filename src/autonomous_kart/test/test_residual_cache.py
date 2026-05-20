"""Tests for the residual disk cache (manifest + RLS state; GBM tested separately)."""
import json
from pathlib import Path

import numpy as np
import pytest

from autonomous_kart.nodes.pathfinder.planners.residual.cache import (
    ResidualCache, SCHEMA_VERSION,
)


def _rls_state():
    return dict(
        theta_s=np.arange(14, dtype=np.float64),
        theta_d=-np.arange(14, dtype=np.float64),
        P=np.eye(14) * 7.0,
        samples_trained=123,
    )


def test_round_trip_rls(tmp_path: Path):
    cache = ResidualCache(str(tmp_path), feature_dim=14)
    state = _rls_state()
    cache.write_rls(**state, best_val_mae_s=0.15, best_val_mae_d=0.04, train_seq=42)
    loaded = cache.load()
    assert loaded is not None
    assert loaded.feature_dim == 14
    np.testing.assert_allclose(loaded.theta_s, state["theta_s"])
    np.testing.assert_allclose(loaded.theta_d, state["theta_d"])
    np.testing.assert_allclose(loaded.P, state["P"])
    assert loaded.samples_trained == 123
    assert loaded.best_val_mae_s == 0.15


def test_load_returns_none_when_no_manifest(tmp_path: Path):
    cache = ResidualCache(str(tmp_path / "doesnotexist"), feature_dim=14)
    assert cache.load() is None


def test_load_rejects_wrong_schema(tmp_path: Path):
    cache = ResidualCache(str(tmp_path), feature_dim=14)
    cache.write_rls(**_rls_state(), best_val_mae_s=0.1, best_val_mae_d=0.04, train_seq=1)
    # Corrupt the manifest version
    manifest_path = tmp_path / "manifest.json"
    m = json.loads(manifest_path.read_text())
    m["schema_version"] = SCHEMA_VERSION + 999
    manifest_path.write_text(json.dumps(m))
    assert cache.load() is None


def test_load_rejects_wrong_feature_dim(tmp_path: Path):
    cache = ResidualCache(str(tmp_path), feature_dim=14)
    cache.write_rls(**_rls_state(), best_val_mae_s=0.1, best_val_mae_d=0.04, train_seq=1)
    cache_wrong = ResidualCache(str(tmp_path), feature_dim=15)
    assert cache_wrong.load() is None


def test_atomic_write_no_partial_state(tmp_path: Path):
    cache = ResidualCache(str(tmp_path), feature_dim=14)
    state = _rls_state()
    cache.write_rls(**state, best_val_mae_s=0.1, best_val_mae_d=0.04, train_seq=1)
    # No .tmp files left behind
    leftovers = list(tmp_path.glob("*.tmp"))
    assert leftovers == []


def test_gbm_round_trip(tmp_path: Path):
    from sklearn.ensemble import HistGradientBoostingRegressor
    cache = ResidualCache(str(tmp_path), feature_dim=14)
    rng = np.random.default_rng(0)
    X = rng.standard_normal((400, 14))
    y_s = 0.3 * X[:, 1] + 0.1 * rng.standard_normal(400)
    y_d = 0.1 * X[:, 2] + 0.05 * rng.standard_normal(400)
    gbm_s = HistGradientBoostingRegressor(max_iter=20, max_depth=3).fit(X, y_s)
    gbm_d = HistGradientBoostingRegressor(max_iter=20, max_depth=3).fit(X, y_d)
    cache.write_rls(theta_s=np.zeros(14), theta_d=np.zeros(14), P=np.eye(14),
                    samples_trained=0, best_val_mae_s=0.05, best_val_mae_d=0.02, train_seq=1)
    cache.write_gbm(gbm_s, gbm_d)
    loaded = cache.load()
    loaded_s, loaded_d = cache.load_gbm()
    assert loaded is not None and loaded_s is not None and loaded_d is not None
    np.testing.assert_allclose(loaded_s.predict(X[:5]), gbm_s.predict(X[:5]))


def test_load_gbm_returns_none_when_missing(tmp_path: Path):
    cache = ResidualCache(str(tmp_path), feature_dim=14)
    s, d = cache.load_gbm()
    assert s is None and d is None
