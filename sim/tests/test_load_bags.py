import os

import numpy as np
import pytest

from sim.load_bags import (
    AlignedBag,
    _mpc_status_fields,
    _track_angle_fields,
    _zoh_align_optional,
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


# RL-residual extras (docs/rl_residual_plan.md) — optional topics that are
# empty in every bag recorded before the camera pipeline existed.


class _FakeMsg:
    def __init__(self, data):
        self.data = data


def test_zoh_align_optional_empty_stream_is_nan():
    grid = np.array([0.0, 0.1, 0.2])
    out = _zoh_align_optional([], grid, lambda m: m.data[0])
    assert out.shape == grid.shape
    assert np.isnan(out).all()


def test_zoh_align_optional_multi_value_empty_stream():
    grid = np.array([0.0, 0.1])
    a, b = _zoh_align_optional([], grid, lambda m: [m.data[0], m.data[1]], n_values=2)
    assert np.isnan(a).all() and np.isnan(b).all()


def test_zoh_align_optional_populated_stream():
    stream = [(0.0, _FakeMsg([10.0])), (0.2, _FakeMsg([20.0]))]
    grid = np.array([0.0, 0.1, 0.2, 0.3])
    out = _zoh_align_optional(stream, grid, lambda m: m.data[0])
    np.testing.assert_array_equal(out, [10.0, 10.0, 20.0, 20.0])


def test_track_angle_fields_right_left_order():
    # opencv_pathfinder_node publishes [right_angle, left_angle]
    right, left = _track_angle_fields(_FakeMsg([5.0, 12.0]))
    assert (right, left) == (5.0, 12.0)


def test_track_angle_fields_handles_short_payload():
    right, left = _track_angle_fields(_FakeMsg([]))
    assert np.isnan(right) and np.isnan(left)


def test_mpc_status_fields_extracts_expected_indices():
    payload = [0.0] * 88
    payload[4], payload[5], payload[10], payload[11] = 1.5, -0.2, 99.0, 3.0
    d, psi_track, cost, margin = _mpc_status_fields(_FakeMsg(payload))
    assert (d, psi_track, cost, margin) == (1.5, -0.2, 99.0, 3.0)


def test_aligned_bag_from_npz_nan_fills_fields_missing_from_old_cache(tmp_path):
    """A cache written before the RL-residual fields existed shouldn't break."""
    n = 5
    old_style = {
        "t": np.arange(n, dtype=np.float64),
        "cmd_throttle": np.zeros(n), "cmd_steer": np.zeros(n),
        "cmd_throttle_hist": np.zeros((n, 3)), "cmd_steer_hist": np.zeros((n, 4)),
        "v": np.zeros(n), "psi_dot": np.zeros(n), "accel_x": np.zeros(n),
        "dv_dt": np.zeros(n),
        "state_mode": np.array(["AUTONOMOUS"] * n, dtype=object),
        "odom_x": np.zeros(n), "odom_y": np.zeros(n), "odom_yaw": np.zeros(n),
        "autonomous": np.ones(n, dtype=bool),
    }
    path = tmp_path / "old_cache.npz"
    np.savez_compressed(path, **old_style)

    bag = AlignedBag.from_npz(str(path))
    assert bag.t.shape == (n,)
    assert bag.track_angle_right.shape == (n,)
    assert np.isnan(bag.track_angle_right).all()
    assert np.isnan(bag.mpc_cost).all()


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
