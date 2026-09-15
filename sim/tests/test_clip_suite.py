import numpy as np
import pytest

from sim.clip_suite import (CLIP_S, HALF_WIDTH_M, OFF_M, ON_M, Clip,
                            distance_to_line, find_clips, score_clips)


def _straight_line(n=400, spacing=1.0):
    """A closed line: out along +x and back, so it runs beside itself."""
    x = np.concatenate([np.arange(n) * spacing, np.arange(n)[::-1] * spacing])
    y = np.concatenate([np.zeros(n), np.full(n, 40.0)])
    return x, y


def test_distance_is_the_true_minimum_over_the_whole_line():
    """The line runs back beside itself, so a nearest-point search must take
    the smaller of the two branches. Measured on line6: the nearest point
    alternates between index 15 and index 226 on successive ticks, and a
    search that tracks only one branch reports a kart 30 m from a line it is
    sitting on."""
    lx, ly = _straight_line()
    # a point a metre above the outbound leg, 39 m below the return leg
    d = distance_to_line([10.0], [1.0], lx, ly)
    assert d[0] == pytest.approx(1.0, abs=1e-6)
    # and one nearer the return leg
    d = distance_to_line([10.0], [38.0], lx, ly)
    assert d[0] == pytest.approx(2.0, abs=1e-6)


def _run(peak, n=480, hz=60.0, v=5.0):
    """A run that bulges `peak` metres off a straight line and comes back.

    Long enough that find_clips has room for a 2 s window with the CLIP_S
    margin it keeps at each end.
    """
    t = np.arange(n) / hz
    x = v * t
    y = peak * np.sin(np.pi * np.clip((t - 2.2) / 2.0, 0.0, 1.0))
    return t, x, y, np.full(n, v)


def test_a_clear_departure_is_labelled_off():
    lx, ly = _straight_line()
    t, x, y, v = _run(peak=4.5)
    clips = find_clips(t, x, y, v, lx, ly, stride_s=0.5)
    assert clips, "a 4.5 m excursion has to be found"
    assert any(c.label == "off" for c in clips)
    assert all(c.peak_m > OFF_M for c in clips if c.label == "off")


def test_a_corner_cut_is_excluded_rather_than_guessed():
    """2.0 m is the planner's corridor parameter, not the track. A kart 2.2 m
    off the line may be cutting a corner, so the band between ON_M and OFF_M
    is dropped -- labelling it either way was what put ordinary driving into
    the departure set."""
    lx, ly = _straight_line()
    t, x, y, v = _run(peak=2.2)
    clips = find_clips(t, x, y, v, lx, ly, stride_s=0.5)
    assert clips, "the run still has clean windows either side of the bulge"
    assert all(c.label != "off" for c in clips), "2.2 m is not a departure"
    # every window that actually contains the bulge is dropped, so nothing
    # labelled sits in the ambiguous band
    assert all(c.peak_m < ON_M for c in clips if c.label == "on")
    assert max(c.peak_m for c in clips) < ON_M


def test_on_track_running_is_labelled_on():
    lx, ly = _straight_line()
    t, x, y, v = _run(peak=0.8)
    clips = find_clips(t, x, y, v, lx, ly, stride_s=0.5)
    assert clips and all(c.label == "on" for c in clips)
    assert all(c.peak_m < ON_M for c in clips)


def test_a_gps_glitch_is_not_a_departure():
    """Three of the 33131 September fixes land hundreds of kilometres away."""
    lx, ly = _straight_line()
    t, x, y, v = _run(peak=0.5)
    y = y.copy()
    y[240] = 889537.0
    clips = find_clips(t, x, y, v, lx, ly, stride_s=0.5)
    assert all(c.label != "off" for c in clips)


def test_slow_and_already_off_clips_are_skipped():
    lx, ly = _straight_line()
    t, x, y, v = _run(peak=4.5)
    assert not find_clips(t, x, y, np.full(len(t), 1.0), lx, ly), "too slow to count"
    # starting well off the line tests nothing about the plant
    assert not find_clips(t, x, y + 6.0, v, lx, ly)


def test_score_counts_both_directions():
    res = [("off", 3.1), ("off", 0.4), ("on", 0.9), ("on", 4.0), ("on", 1.1)]
    s = score_clips(res)
    assert (s["off_reproduced"], s["off_total"]) == (1, 2)
    assert (s["on_reproduced"], s["on_total"]) == (2, 3)
    assert s["agreement"] == pytest.approx(3 / 5)
    assert math_isnan(score_clips([])["agreement"])


def math_isnan(v):
    return v != v


def test_seed_window_reads_the_reference_not_the_estimator():
    """The fix that turned 1 of 5 departures into 3 of 3.

    The plant is trained against the fused RTK reference, so a clip has to
    hand it reference states. Seeding from /mpc/status EKF telemetry charges
    the plant for the estimator's own error, which on these runs is 0.12-0.81 m
    -- most of a departure -- and the 2 s position error fell from 2.56 m
    median to 0.78 m when the seed changed.
    """
    from types import SimpleNamespace

    from sim.clip_suite import seed_window

    n = 20
    ref = SimpleNamespace(v=np.arange(n) * 0.1 + 4.0,
                          omega=np.arange(n) * 0.01,
                          x=np.zeros(n), y=np.zeros(n), psi=np.zeros(n))
    wheel = np.arange(n) * 0.5
    throttle = np.full(n, 6.0)

    rows = seed_window(ref, wheel, throttle, k0=10, history=5)
    assert len(rows) == 5
    # oldest first, ending on the start tick
    assert rows[-1] == pytest.approx((ref.v[10], ref.omega[10], wheel[10], 6.0))
    assert rows[0] == pytest.approx((ref.v[6], ref.omega[6], wheel[6], 6.0))
    # the yaw rate is carried, not zeroed: a kart handed over mid-corner is turning
    assert rows[-1][1] > 0.0
