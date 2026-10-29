"""The correction reaches the rollout, by moving the line, only when applying."""
import math

import numpy as np
import pytest

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants
from autonomous_kart.nodes.pathfinder.planners.mpc import LineCorrection, MPCPlanner

# Fast rack on purpose: at 33.7 deg/s the first command is slew-pinned.
KART = KartConstants(v_max_mps=10.0, wheelbase_m=1.0, steer_max_deg=15.0,
                     steer_rate_max_degps=400.0, a_max_mps2=5.3,
                     a_min_mps2=-3.0, a_lat_max_mps2=5.3)
HORIZONS = (0.25, 0.5, 1.0, 1.5, 2.0)


def _line(n=400, step=1.0):
    """Straight along +x, so the line normal is +y and d reads off directly."""
    return [(i * step, i * step, 0.0, 0.0, 0.0, 6.0) for i in range(n)]


def _planner(mode, **over):
    params = {
        "num_samples": 64, "horizon_steps": 20, "dt_s": 0.05,
        "target_speed_mps": 6.0, "actuator_gain": 1.0, "use_steer_map": False,
        "proj_window_back": 5, "proj_window_fwd": 60,
        "residual.mode": mode, "residual.model_size": "s",
        "residual.cache_enabled": False, "residual.target_horizon_s": 0.5,
        "residual.taps_s": list(HORIZONS),
        "residual.correction_speed_floor_mps": 0.5,
        "residual.apply_min_samples_this_run": 0,
        "residual.rls_warmup_samples": 0,
    }
    params.update(over)
    return MPCPlanner(params, KART, _line(), logger=None, node=None)


def _project(win, px, py):
    """(s, d) of a point against a line slice, as _solve computes it."""
    lx, ly, lp, ls = win
    j = int(np.argmin((lx - px) ** 2 + (ly - py) ** 2))
    ex, ey = px - lx[j], py - ly[j]
    c, s_ = math.cos(lp[j]), math.sin(lp[j])
    return ls[j] + ex * c + ey * s_, -ex * s_ + ey * c


def _scored(p, j_now):
    _, _, lx, ly, psi, ls, _ = p._scored_line(j_now)
    return lx, ly, psi, ls


def _correction(**over):
    c = LineCorrection(HORIZONS, over.pop("speed_floor", 0.5))
    return c


def test_idle_correction_passes_the_line_through():
    c = _correction()
    x, y, s = np.arange(5.0), np.zeros(5), np.arange(5.0)
    out = c.apply(x, y, np.zeros(5), s)
    assert out[0] is x and out[1] is y and out[2] is s


def test_speed_floor_bounds_how_far_ahead_a_point_is_read_as():
    c = LineCorrection((1.0,), speed_floor_mps=4.0)
    c.set([0.0], [0.5], s_now=0.0, speed=1.0)
    # At the 4 m/s floor, 4 m ahead reads as one second even at 1 m/s.
    _, y, _ = c.apply(np.array([4.0]), np.array([0.0]), np.array([0.0]),
                      np.array([4.0]))
    assert y[0] == pytest.approx(-0.5)


def test_gate_shut_leaves_the_line_untouched():
    p = _planner("shadow")
    p._predict_correction(0.0, 0.0, 0.0, 6.0, 10, 10.0, 0.0, 6.0, 0.0)
    assert not p._correction.active
    lo, hi, lx, ly, _, ls, _ = p._scored_line(10)
    assert np.array_equal(lx, p.l_x[lo:hi]) and np.array_equal(ls, p.l_s[lo:hi])


def test_corrected_line_reproduces_the_predicted_lateral_shift():
    p = _planner("apply")
    p._correction.set([0.0] * 5, [0.05, 0.10, 0.20, 0.30, 0.40],
                      s_now=10.0, speed=6.0)
    win = _scored(p, 10)
    # 6 m ahead at 6 m/s is the 1.0 s horizon, so the line carries that offset.
    assert _project(win, 16.0, 0.0)[1] == pytest.approx(0.20, abs=1e-9)
    assert _project(win, 13.0, 0.0)[1] == pytest.approx(0.10, abs=1e-9)


def test_lateral_correction_does_not_leak_into_arc_length():
    p = _planner("apply")
    plain_s, _ = _project(_scored(p, 10), 16.0, 0.0)
    p._correction.set([0.0] * 5, [0.05, 0.10, 0.20, 0.30, 0.40],
                      s_now=10.0, speed=6.0)
    corrected_s, _ = _project(_scored(p, 10), 16.0, 0.0)
    assert corrected_s == pytest.approx(plain_s, abs=1e-9)


def test_along_track_correction_shifts_arc_length():
    p = _planner("apply")
    plain_s, _ = _project(_scored(p, 10), 16.0, 0.0)
    p._correction.set([0.1, 0.2, 0.4, 0.6, 0.8], [0.0] * 5,
                      s_now=10.0, speed=6.0)
    corrected_s, _ = _project(_scored(p, 10), 16.0, 0.0)
    assert corrected_s - plain_s == pytest.approx(0.4, abs=1e-9)


def test_correction_moves_the_published_command():
    base = _planner("off")
    base._rng = np.random.default_rng(0)
    base._solve(5.0, 0.5, 0.0, 6.0, 5, 6.0, 10.0)
    plain = base._cmd_out

    p = _planner("apply")
    p.residual.predict = lambda features: ([0.0] * 5, [0.3] * 5)
    p._predict_correction(5.0, 0.5, 0.0, 6.0, 5, 5.0, 0.5, 6.0, 0.0)
    p._rng = np.random.default_rng(0)
    p._solve(5.0, 0.5, 0.0, 6.0, 5, 6.0, 10.0)
    # Claiming the kart will drift further left makes the solver steer right.
    assert p._cmd_out < plain


def test_every_horizon_gets_its_own_nominal():
    p = _planner("apply")
    nominals = p._nominal_offsets(0.0, 0.0, 0.0, 6.0, math.radians(5.0), 0.0,
                                  0, 0.0, 0.0)
    assert len(nominals) == len(HORIZONS)
    # A held steering command curves away, so the lateral nominal grows.
    lateral = [abs(d) for _, d in nominals]
    assert lateral == sorted(lateral) and lateral[-1] > lateral[0]


def test_warm_cmd_respects_both_clamps():
    p = _planner("off")
    # Rate limit binds: 20 deg of slew per step from -10 deg caps the ask.
    p.delta_prev = math.radians(-10.0)
    p.u_mean[0, 0] = math.radians(14.0)
    assert p._warm_cmd() == pytest.approx(math.radians(10.0))
    # Travel limit binds.
    p.delta_prev = math.radians(14.0)
    p.u_mean[0, 0] = math.radians(30.0)
    assert p._warm_cmd() == pytest.approx(p.steer_max)


def test_gates_scale_with_the_horizon():
    p = _planner("shadow")
    half = p.residual.learners[p.residual.horizons.index(0.5)]
    two = p.residual.learners[p.residual.horizons.index(2.0)]
    assert two.outlier_threshold == pytest.approx(4.0 * half.outlier_threshold)
    assert two._predict_clip_m == pytest.approx(4.0 * half._predict_clip_m)
