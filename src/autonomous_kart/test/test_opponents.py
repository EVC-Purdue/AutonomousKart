"""Unit tests for opponent prediction, track bounds, pass tactics and the MPC hooks."""
import math
import os
import sys
import types

import numpy as np
import pytest

from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import TrackFrame
from autonomous_kart.nodes.pathfinder.opponents import (
    FOLLOW, FREE, PASS, Opponent, PassPlanner, TrackBounds, predict,
)

_DATA = os.path.join(os.path.dirname(__file__), "..", "..", "..", "data")
_LINE6 = os.path.join(_DATA, "racing_line", "line6.csv")
_KML = os.path.join(_DATA, "track", "purdue_grand_prix.kml")
LAT0, LON0 = 40.4380475, -86.9442826


def _circle(radius=50.0, n=400):
    th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return [(radius * t, radius * math.sin(t), radius - radius * math.cos(t), 0.0, 1.0 / radius) for t in th]


@pytest.fixture(scope="module")
def line6():
    if not (os.path.exists(_LINE6) and os.path.exists(_KML)):
        pytest.skip("track data not available")
    return [tuple(r) for r in np.genfromtxt(_LINE6, delimiter=",", skip_header=1)]


# Track bounds


def test_kml_bounds_match_the_real_track(line6):
    fr = TrackFrame(line6)
    b = TrackBounds.from_kml(_KML, fr, LAT0, LON0)
    width = b.left + b.right
    assert 4.0 < width.min() and np.median(width) == pytest.approx(5.0, abs=0.5)
    assert b.left.min() > 0.3 and b.right.min() > 0.3  # line stays on track
    left, right = b.room([0.0, 100.0])
    assert left.shape == (2,) and np.all(left > 0) and np.all(right > 0)


def test_symmetric_bounds():
    fr = TrackFrame(_circle())
    left, right = TrackBounds.symmetric(fr, 2.0).room(np.array([1.0, 150.0]))
    assert np.allclose(left, 2.0) and np.allclose(right, 2.0)


# Prediction


def test_predict_constant_speed_along_line_with_latency_compensation():
    fr = TrackFrame(_circle())
    t = np.array([0.5, 1.0, 2.0])
    op = Opponent(1, 10.0, 0.0, 5.0, 0.0, stamp_s=99.8)  # 0.2 s old
    x, y, h = predict(fr, [op], now_s=100.0, t_steps=t)
    ex, ey, epsi, _ = fr.pose_at(10.0 + 5.0 * (0.2 + t))
    assert np.allclose(x[0], ex) and np.allclose(y[0], ey) and np.allclose(h[0], epsi)


def test_predict_decays_lateral_velocity_and_clamps_to_track():
    fr = TrackFrame(_circle())
    op = Opponent(1, 10.0, 0.0, 0.0, 2.0, stamp_s=0.0)
    t = np.array([0.1, 1.0, 10.0])
    x, y, _ = predict(fr, [op], 0.0, t, d_dot_tau_s=0.5)
    d, _ = fr.to_frenet(np.column_stack([x[0], y[0]]), np.repeat(np.eye(2)[None], 3, 0))
    assert d[-1, 1] == pytest.approx(2.0 * 0.5, abs=0.05)  # tau * d_dot, not 20 m
    x, y, _ = predict(fr, [op], 0.0, t, bounds=TrackBounds.symmetric(fr, 1.0), kart_half_width=0.5)
    d, _ = fr.to_frenet(np.column_stack([x[0], y[0]]), np.repeat(np.eye(2)[None], 3, 0))
    assert d[-1, 1] == pytest.approx(0.5, abs=0.05)


def test_predict_empty():
    x, y, h = predict(TrackFrame(_circle()), [], 0.0, np.arange(5))
    assert x.shape == (0, 5)


# Tactics


def _pp(half_width=2.5):
    fr = TrackFrame(_circle())
    return PassPlanner(), fr, TrackBounds.symmetric(fr, half_width)


def test_free_road():
    pp, fr, b = _pp()
    assert pp.update(0.0, 0.0, 6.0, [], fr, b) == 0.0 and pp.state == FREE


def test_passes_slower_kart_on_side_with_least_deviation():
    pp, fr, b = _pp()
    slow = Opponent(1, 10.0, 0.3, 3.0, 0.0, 0.0)  # slightly left of the line
    d_ref = pp.update(0.0, 0.0, 6.0, [slow], fr, b)
    assert pp.state == PASS and pp.side == -1  # right side needs less deviation
    assert d_ref == pytest.approx(0.3 - 1.5)


def test_ignores_faster_kart_ahead():
    pp, fr, b = _pp()
    assert pp.update(0.0, 0.0, 6.0, [Opponent(1, 10.0, 0.0, 7.0, 0.0, 0.0)], fr, b) == 0.0
    assert pp.state == FREE


def test_follows_when_no_room():
    pp, fr, b = _pp(half_width=1.2)  # 2.4 m track: no side has room
    d_ref = pp.update(0.0, 0.0, 6.0, [Opponent(1, 10.0, 0.0, 3.0, 0.0, 0.0)], fr, b)
    assert d_ref == 0.0 and pp.state == FOLLOW


def test_follows_two_karts_side_by_side():
    pp, fr, b = _pp(half_width=2.5)
    opps = [Opponent(1, 10.0, 1.1, 3.0, 0.0, 0.0), Opponent(2, 10.0, -1.1, 3.0, 0.0, 0.0)]
    assert pp.update(0.0, 0.0, 6.0, opps, fr, b) == 0.0 and pp.state == FOLLOW


def test_pass_completes_then_returns_to_line():
    pp, fr, b = _pp()
    tgt = Opponent(1, 10.0, 0.0, 3.0, 0.0, 0.0)
    d1 = pp.update(0.0, 0.0, 6.0, [tgt], fr, b)
    assert pp.state == PASS
    # Alongside: keep the side even though the target is no longer "ahead"
    assert pp.update(10.0, d1, 6.0, [tgt], fr, b) == d1
    # A kart length + clearance past: done
    assert pp.update(10.0 + 1.8 + 1.6, d1, 6.0, [tgt], fr, b) == 0.0 and pp.state == FREE


# MPC hooks


def _install_ros_stubs():
    if "std_msgs" not in sys.modules:
        std_msgs = types.ModuleType("std_msgs")
        msg = types.ModuleType("std_msgs.msg")

        class Float32MultiArray:
            def __init__(self, data=None):
                self.data = list(data or [])

        msg.Float32MultiArray = Float32MultiArray
        std_msgs.msg = msg
        sys.modules["std_msgs"], sys.modules["std_msgs.msg"] = std_msgs, msg


def _mpc(line, **extra):
    _install_ros_stubs()
    from autonomous_kart.nodes.pathfinder.planners.base import KartConstants
    from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner
    kart = KartConstants(12.0, 1.05, 60.0, 33.7, 5.3, -3.0, 5.3)
    params = {"horizon_steps": 40, "num_samples": 128, "residual.cache_enabled": False,
              "residual.model_size": "s", "opp.track_kml": _KML, **extra}
    return MPCPlanner(params, kart, line)


def _inputs(line, t, opponents=()):
    from autonomous_kart.nodes.pathfinder.planners.base import PlannerInputs
    x, y = line[20][1], line[20][2]
    yaw = math.atan2(line[21][2] - y, line[21][1] - x)
    return PlannerInputs(pose_xy=(x, y), yaw_rad=yaw, speed_mps=6.0, track_angles=None,
                         now_ns=int(t * 1e9), opponents=opponents)


def test_mpc_without_opponents_is_unchanged(line6):
    a, b = _mpc(line6), _mpc(line6)
    for k in range(5):
        t = k / 60.0
        assert a.plan(_inputs(line6, t)) == b.plan(_inputs(line6, t, opponents=()))
    assert not a.opp_engaged and a._opp_pred is None


def test_mpc_avoids_stopped_kart_ahead(line6):
    mpc = _mpc(line6)
    s_ego = line6[20][0]
    stopped = (Opponent(1, s_ego + 6.0, 0.0, 0.0, 0.0, 0.0),)
    free_speed, free_steer = mpc.plan(_inputs(line6, 0.0))
    for k in range(1, 30):
        speed, steer = mpc.plan(_inputs(line6, k / 60.0, stopped))
    assert mpc.opp_engaged
    # Either way out is fine at 6 m/s and 6 m: swerve around it or slow down
    assert mpc.pass_planner.state == "pass" or speed < free_speed - 0.5
    assert abs(steer - free_steer) > 5.0 or speed < free_speed - 0.5


def test_mpc_brakes_when_karts_block_the_track(line6):
    mpc = _mpc(line6)
    s_ego = line6[20][0]
    wall = (Opponent(1, s_ego + 8.0, 1.2, 0.0, 0.0, 0.0), Opponent(2, s_ego + 8.0, -1.2, 0.0, 0.0, 0.0))
    free_speed, _ = mpc.plan(_inputs(line6, 0.0))
    for k in range(1, 30):
        speed, _ = mpc.plan(_inputs(line6, k / 60.0, wall))
    assert mpc.pass_planner.state == "follow"
    assert mpc.opp_engaged and speed < free_speed - 0.5
