import os

import numpy as np
import pytest

from sim.load_bags import (
    AlignedBag,
    central_diff,
    make_history,
    zoh_align,
)


def test_zoh_align_basic():
    sample_t = np.array([0.0, 0.1, 0.3])
    sample_v = np.array([10.0, 20.0, 30.0])
    grid_t = np.array([0.0, 0.05, 0.1, 0.2, 0.3, 0.5])
    out = zoh_align(sample_t, sample_v, grid_t)
    np.testing.assert_array_equal(out, [10.0, 10.0, 20.0, 20.0, 30.0, 30.0])


def test_zoh_align_unsorted_input():
    sample_t = np.array([0.3, 0.0, 0.1])
    sample_v = np.array([30.0, 10.0, 20.0])
    grid_t = np.array([0.05, 0.15])
    out = zoh_align(sample_t, sample_v, grid_t)
    np.testing.assert_array_equal(out, [10.0, 20.0])


def test_zoh_align_empty_raises():
    with pytest.raises(ValueError):
        zoh_align(np.array([]), np.array([]), np.array([0.0]))


def test_central_diff_constant_slope():
    dt = 0.01
    x = np.arange(10) * 2.0
    out = central_diff(x, dt)
    np.testing.assert_allclose(out, 200.0)


def test_make_history_pads_with_first():
    v = np.array([1.0, 2.0, 3.0, 4.0])
    h = make_history(v, depth=3)
    assert h.shape == (4, 3)
    np.testing.assert_array_equal(h[3], [3.0, 2.0, 1.0])
    np.testing.assert_array_equal(h[2], [2.0, 1.0, 1.0])
    np.testing.assert_array_equal(h[0], [1.0, 1.0, 1.0])


REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HOLDOUT_NPZ = os.path.join(REPO, "docs", "superpowers", "data_sim", "cache", "231452.npz")


@pytest.mark.skipif(not os.path.isfile(HOLDOUT_NPZ),
                    reason="run sim/load_bags.py first")
def test_holdout_roundtrip():
    bag = AlignedBag.from_npz(HOLDOUT_NPZ)
    n = bag.t.size
    assert n > 10000
    assert bag.cmd_throttle.shape == (n,)
    assert bag.cmd_steer_hist.shape[1] == 4
    assert bag.cmd_throttle_hist.shape[1] == 3
    assert bag.autonomous.dtype == bool
    assert bag.autonomous.mean() > 0.5  # holdout is mostly autonomous
    assert np.isfinite(bag.v).all()
    assert np.isfinite(bag.psi_dot).all()
