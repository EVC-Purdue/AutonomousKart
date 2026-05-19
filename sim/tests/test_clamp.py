import numpy as np

from sim.clamp import Clamp, fit_clamp


def test_alpha_one_at_training_mean():
    rng = np.random.default_rng(0)
    feats = rng.normal(0, 1, (5000, 11))
    clamp = fit_clamp(feats, d_warn_pct=95, d_max_pct=99.5)
    assert clamp.alpha(np.zeros(11)) == 1.0


def test_alpha_zero_far_off():
    rng = np.random.default_rng(0)
    feats = rng.normal(0, 1, (5000, 11))
    clamp = fit_clamp(feats, d_warn_pct=95, d_max_pct=99.5)
    far = np.full(11, 100.0)
    assert clamp.alpha(far) == 0.0


def test_alpha_between_at_midpoint():
    rng = np.random.default_rng(0)
    feats = rng.normal(0, 1, (5000, 11))
    clamp = fit_clamp(feats, d_warn_pct=95, d_max_pct=99.5)
    halfway = (clamp.d_warn + clamp.d_max) / 2
    assert clamp._alpha_from_d(clamp.d_warn) == 1.0
    assert clamp._alpha_from_d(clamp.d_max) == 0.0
    mid_alpha = clamp._alpha_from_d(halfway)
    assert 0.45 < mid_alpha < 0.55
