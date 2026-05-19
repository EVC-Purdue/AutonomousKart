"""Tests for the TrainBuffer ring."""
import threading

import numpy as np
import pytest

from autonomous_kart.nodes.pathfinder.planners.residual.buffer import TrainBuffer


def test_buffer_grows_then_wraps():
    buf = TrainBuffer(capacity=4, feature_dim=3)
    for i in range(3):
        phi = np.array([float(i), float(i + 1), float(i + 2)])
        buf.append(phi, r_s=float(i) * 0.1, r_d=float(i) * 0.2, t_ns=i)
    assert buf.size == 3
    X, ys, yd, t = buf.snapshot()
    assert X.shape == (3, 3)
    np.testing.assert_allclose(X[1], [1, 2, 3])
    np.testing.assert_allclose(ys, [0.0, 0.1, 0.2])
    np.testing.assert_allclose(t, [0, 1, 2])

    # Fill past capacity — oldest entries drop.
    for i in range(3, 8):
        buf.append(np.array([float(i)] * 3), r_s=0.0, r_d=0.0, t_ns=i)
    assert buf.size == 4
    X, ys, yd, t = buf.snapshot()
    # Most recent 4 entries (i=4..7), in chronological order.
    np.testing.assert_allclose(t, [4, 5, 6, 7])


def test_snapshot_returns_independent_copy():
    buf = TrainBuffer(capacity=2, feature_dim=2)
    buf.append(np.array([1.0, 2.0]), r_s=0.5, r_d=0.5, t_ns=0)
    X, ys, yd, t = buf.snapshot()
    X[0, 0] = 99.0
    X2, *_ = buf.snapshot()
    assert X2[0, 0] == 1.0  # buffer wasn't mutated


def test_concurrent_writes_no_torn_rows():
    buf = TrainBuffer(capacity=200, feature_dim=4)

    def writer(start, count):
        for i in range(start, start + count):
            phi = np.array([float(i)] * 4)
            buf.append(phi, r_s=float(i), r_d=-float(i), t_ns=i)

    threads = [threading.Thread(target=writer, args=(k * 50, 50)) for k in range(4)]
    for t in threads: t.start()
    for t in threads: t.join()

    X, ys, yd, _ = buf.snapshot()
    # Each row's 4 features should be all-equal; r_s should equal +feature; r_d should equal -feature
    for row, s, d in zip(X, ys, yd):
        v = row[0]
        assert all(row == v), f"torn row: {row}"
        assert s == v, f"r_s mismatch with row value: {s} vs {v}"
        assert d == -v, f"r_d mismatch with row value: {d} vs {v}"


def test_initial_state():
    buf = TrainBuffer(capacity=10, feature_dim=14)
    assert buf.size == 0
    assert buf.capacity == 10
    X, ys, yd, t = buf.snapshot()
    assert X.shape == (0, 14)
    assert ys.shape == (0,)
