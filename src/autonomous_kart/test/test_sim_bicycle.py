"""Unit tests for the kinematic bicycle used by localization in sim mode."""
import math

import pytest

from autonomous_kart.nodes.localization.sim_bicycle import BicycleModel


def _mk(**kw):
    defaults = dict(wheelbase_m=1.05, v_max_mps=15.0, steer_max_deg=25.0)
    defaults.update(kw)
    return BicycleModel(**defaults)


def test_starts_at_origin_at_rest():
    m = _mk()
    assert (m.x, m.y, m.yaw, m.speed) == (0.0, 0.0, 0.0, 0.0)


def test_reset_places_pose_and_zeros_speed():
    m = _mk()
    m.speed = 4.0
    m.reset(10.0, -5.0, math.pi / 2)
    assert m.x == 10.0 and m.y == -5.0
    assert math.isclose(m.yaw, math.pi / 2)
    assert m.speed == 0.0


def test_zero_motor_does_not_move():
    m = _mk()
    for _ in range(100):
        m.step(motor_pct=0.0, steer_deg=0.0, dt=0.02)
    assert m.x == pytest.approx(0.0, abs=1e-9)
    assert m.y == pytest.approx(0.0, abs=1e-9)
    assert m.speed == pytest.approx(0.0, abs=1e-9)


def test_full_throttle_straight_accelerates_and_approaches_vmax():
    m = _mk(v_max_mps=15.0)
    for _ in range(500):
        m.step(100.0, 0.0, 0.02)
    assert m.speed == pytest.approx(15.0, rel=1e-3)
    assert m.x > 10.0
    assert m.y == pytest.approx(0.0, abs=1e-9)
    assert m.yaw == pytest.approx(0.0, abs=1e-9)


def test_motor_is_clamped_to_nonnegative_and_v_max():
    m = _mk(v_max_mps=10.0)
    # Negative / >100 should behave as 0 / 100
    for _ in range(500):
        m.step(-20.0, 0.0, 0.02)
    assert m.speed == pytest.approx(0.0, abs=1e-9)

    m2 = _mk(v_max_mps=10.0)
    for _ in range(500):
        m2.step(1e6, 0.0, 0.02)
    assert m2.speed == pytest.approx(10.0, rel=1e-3)


def test_steering_clamped_to_physical_limit():
    m = _mk(steer_max_deg=25.0)
    m.speed = 5.0
    # Command 90 deg but physical max is 25 deg → yaw rate should match 25 deg
    m_ref = _mk(steer_max_deg=25.0)
    m_ref.speed = 5.0
    for _ in range(10):
        m.step(100.0, 90.0, 0.02)
        m_ref.step(100.0, 25.0, 0.02)
    assert m.yaw == pytest.approx(m_ref.yaw, rel=1e-6)


def test_turns_left_with_positive_steer():
    m = _mk()
    m.speed = 5.0
    for _ in range(100):
        m.step(100.0, 15.0, 0.02)
    assert m.yaw > 0.0
    assert m.y > 0.0  # heading left, world y grew


def test_turns_right_with_negative_steer():
    m = _mk()
    m.speed = 5.0
    for _ in range(100):
        m.step(100.0, -15.0, 0.02)
    assert m.yaw < 0.0
    assert m.y < 0.0


def test_yaw_stays_in_minus_pi_to_pi():
    m = _mk()
    m.speed = 10.0
    for _ in range(2000):
        m.step(100.0, 20.0, 0.02)
        assert -math.pi - 1e-9 <= m.yaw <= math.pi + 1e-9


def test_zero_wheelbase_does_not_divide_by_zero():
    m = _mk(wheelbase_m=0.0)
    m.speed = 5.0
    # Must not raise; yaw should remain unchanged because the kinematic
    # yaw-rate term is guarded.
    m.step(100.0, 10.0, 0.02)
    assert m.yaw == 0.0


def test_brake_decelerates_to_zero():
    m = _mk()
    m.speed = 10.0
    for _ in range(500):
        m.step(0.0, 0.0, 0.02)
    assert m.speed == pytest.approx(0.0, abs=1e-3)
