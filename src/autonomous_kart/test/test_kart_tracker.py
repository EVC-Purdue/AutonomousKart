"""Unit tests for the Frenet-frame multi-kart tracker."""
import math
import os

import numpy as np
import pytest

from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import KartTracker, TrackFrame

DT = 1.0 / 30.0
SIG = 0.3
R = np.diag([SIG ** 2, SIG ** 2])
NONE_XY, NONE_COV = np.zeros((0, 2)), np.zeros((0, 2, 2))

_LINE6 = os.path.join(os.path.dirname(__file__), "..", "..", "..", "data", "racing_line", "line6.csv")


def _circle(radius=12.0, n=200):
    th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    # psi column deliberately garbage: TrackFrame must derive heading from xy
    return [(radius * t, radius * math.sin(t), radius - radius * math.cos(t), 0.0, 1.0 / radius) for t in th]


@pytest.fixture(scope="module")
def line6():
    if not os.path.exists(_LINE6):
        pytest.skip("racing line data not available")
    return [tuple(r) for r in np.genfromtxt(_LINE6, delimiter=",", skip_header=1)]


def _follow(tracker, frame, v, d, steps, rng, t0=0.0, drop=lambda k: False, in_fov=None):
    """One kart at constant s_dot=v, offset d. Returns (last output, vel errors, ids)."""
    errs, ids, out = [], set(), []
    for k in range(steps):
        t = t0 + k * DT
        s = (v * t) % frame.s_total
        x, y, vx, vy = frame.to_world(s, d, v, 0.0)
        z = NONE_XY if drop(k) else (np.array([[x, y]]) + rng.normal(0.0, SIG, 2))
        cov = np.repeat(R[None], len(z), axis=0)
        out = tracker.step(t, z, cov, ego_s=s, ego_s_dot=v, in_fov=in_fov)
        if k > 60 and out:
            errs.append(math.hypot(out[0].vx - vx, out[0].vy - vy))
            ids |= {o.id for o in out}
    return out, np.sqrt(np.mean(np.square(errs))) if errs else math.nan, ids


# TrackFrame


def test_frame_closes_loop_and_round_trips(line6):
    fr = TrackFrame(line6)
    assert fr.s_total == pytest.approx(846.0, abs=0.5)
    for s, d in [(10.0, 0.0), (200.0, 1.5), (500.0, -1.0), (845.0, 0.5)]:
        x, y, _, _ = fr.to_world(s, d)
        sd, _ = fr.to_frenet(np.array([[x, y]]), np.eye(2)[None], hint_s=s)
        assert float(fr.wrap_ds(sd[0, 0] - s)) == pytest.approx(0.0, abs=0.1)
        assert sd[0, 1] == pytest.approx(d, abs=0.1)


def test_frame_heading_ignores_csv_psi_column(line6):
    # line6's psi column is ~pi/2 off the direction of travel
    fr = TrackFrame(line6)
    _, _, vx, vy = fr.to_world(100.0, 0.0, 5.0, 0.0)
    x0, y0, _, _ = fr.to_world(100.0, 0.0)
    x1, y1, _, _ = fr.to_world(101.0, 0.0)
    assert math.atan2(vy, vx) == pytest.approx(math.atan2(y1 - y0, x1 - x0), abs=0.05)


def test_frenet_covariance_rotates_with_line():
    fr = TrackFrame(_circle())
    s = 0.25 * fr.s_total  # theta = pi/2: point (12, 12), tangent +y
    x, y, _, _ = fr.to_world(s, 0.0)
    cov = np.diag([1.0, 0.01])[None]  # uncertain in world x only
    _, cf = fr.to_frenet(np.array([[x, y]]), cov, hint_s=s)
    # Tangent is along +y, so the world-x uncertainty is lateral (d)
    assert cf[0, 1, 1] == pytest.approx(1.0, rel=0.05)
    assert cf[0, 0, 0] == pytest.approx(0.01, rel=0.2)


def test_frame_wrap():
    fr = TrackFrame(_circle())
    L = fr.s_total
    assert float(fr.wrap_ds(L - 1.0)) == pytest.approx(-1.0)
    assert float(fr.wrap_ds(-(L - 2.0))) == pytest.approx(2.0)


# Tracking accuracy


def test_corner_velocity_accuracy_on_tight_circle():
    # Constant velocity in x/y scored 1.2 m/s RMS here at best; the Frenet
    # model knows the kart is following the curve.
    fr = TrackFrame(_circle(12.0))
    _, rms, ids = _follow(KartTracker(fr), fr, 7.0, 0.0, 300, np.random.default_rng(0))
    assert ids == {1}
    assert rms < 0.6


def test_full_lap_of_real_line_keeps_one_id_across_seam(line6):
    fr = TrackFrame(line6)
    # 846 m at 7 m/s ~ 121 s; run a lap plus the seam crossing
    out, rms, ids = _follow(KartTracker(fr), fr, 7.0, 1.0, int(125.0 / DT), np.random.default_rng(1))
    assert ids == {1}
    assert rms < 0.6
    assert out[0].d == pytest.approx(1.0, abs=0.4)
    assert out[0].s_dot == pytest.approx(7.0, abs=0.6)


def test_two_karts_side_by_side_keep_ids(line6):
    fr = TrackFrame(line6)
    tr = KartTracker(fr)
    rng = np.random.default_rng(2)
    ids = set()
    for k in range(150):
        t = k * DT
        pts = []
        for s0, d, v in [(50.0, -1.2, 7.0), (52.0, 1.2, 6.5)]:
            x, y, _, _ = fr.to_world(s0 + v * t, d)
            pts.append([x, y])
        z = np.array(pts) + rng.normal(0.0, SIG, (2, 2))
        out = tr.step(t, z, np.repeat(R[None], 2, axis=0), ego_s=40.0 + 7.0 * t, ego_s_dot=7.0)
        if k > 10:
            assert len(out) == 2
            ids |= {o.id for o in out}
            left = max(out, key=lambda o: o.d)
            if k > 60:
                assert left.s_dot == pytest.approx(6.5, abs=0.8)
    assert ids == {1, 2}


# Lifecycle


def test_single_false_positive_never_confirmed():
    fr = TrackFrame(_circle())
    tr = KartTracker(fr)
    assert tr.step(0.0, np.array([[0.0, 0.0]]), R[None]) == []
    for k in range(1, 20):
        assert tr.step(k * DT, NONE_XY, NONE_COV) == []


def test_confirmed_track_dropped_after_coasting_in_view():
    fr = TrackFrame(_circle(30.0))
    tr = KartTracker(fr, {"max_coast_s": 0.4})
    _follow(tr, fr, 5.0, 0.0, 30, np.random.default_rng(3))
    t0 = 29 * DT
    assert len(tr.step(t0 + 0.2, NONE_XY, NONE_COV)) == 1
    assert tr.step(t0 + 0.5, NONE_XY, NONE_COV) == []


def test_out_of_fov_track_coasts_longer():
    fr = TrackFrame(_circle(30.0))
    tr = KartTracker(fr, {"max_coast_s": 0.4, "max_coast_out_of_fov_s": 2.0})
    _follow(tr, fr, 5.0, 0.0, 30, np.random.default_rng(4))
    t0 = 29 * DT
    hidden = lambda x, y: False  # noqa: E731
    out = tr.step(t0 + 1.0, NONE_XY, NONE_COV, in_fov=hidden)
    assert len(out) == 1 and out[0].coast_s == pytest.approx(1.0)
    assert tr.step(t0 + 2.5, NONE_XY, NONE_COV, in_fov=hidden) == []


def test_reacquires_same_id_after_short_dropout(line6):
    fr = TrackFrame(line6)
    out, _, ids = _follow(KartTracker(fr), fr, 6.0, 0.0, 90, np.random.default_rng(5),
                          drop=lambda k: 40 <= k < 48)
    assert ids == {1} and [o.id for o in out] == [1]


def test_duplicate_box_does_not_spawn_second_track():
    fr = TrackFrame(_circle(30.0))
    tr = KartTracker(fr, {"confirm_hits": 1})
    tr.step(0.0, np.array([[0.0, 0.0]]), R[None])
    out = tr.step(DT, np.array([[0.0, 0.0], [0.2, 0.1]]), np.repeat(R[None], 2, axis=0))
    assert len(out) == 1


def test_ego_speed_seeds_new_tracks():
    fr = TrackFrame(_circle(30.0))
    tr = KartTracker(fr, {"confirm_hits": 1})
    out = tr.step(0.0, np.array([[0.0, 0.0]]), R[None], ego_s=0.0, ego_s_dot=6.0)
    assert out[0].s_dot == 6.0
    assert out[0].speed_mps == pytest.approx(6.0, rel=1e-3)
