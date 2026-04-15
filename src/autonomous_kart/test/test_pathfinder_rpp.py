"""
Unit tests for the Nav2-style regulated pure-pursuit controller
in `autonomous_kart.nodes.pathfinder.pathfinder`.

Tests exercise the behaviours that actually matter on the kart:
 - straight-ahead target → zero steering
 - lateral offset → turn toward it, signed correctly
 - sharp corner → curvature regulation slows throttle
 - target behind vehicle → crawl forward, don't command negative speed
 - approach slowdown near end of path
 - degenerate configs return (0, 0) instead of blowing up
 - clamping of speed_pct / desired_speed_pct inputs
"""
import math

import pytest

from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder

BASE = dict(
    wheelbase_m=1.05,
    v_max_mps=15.0,
    steer_max_deg=25.0,
    desired_speed_pct=1.0,
    use_velocity_scaled_lookahead=False,  # deterministic L in tests
    min_lookahead_m=3.0,
    max_lookahead_m=8.0,
    use_curvature_regulation=False,
    min_radius_m=12.0,
    min_reg_speed_pct=0.20,
    approach_dist_m=0.0,  # disable approach ramp unless test opts in
    min_approach_speed_pct=0.05,
)


def call(**overrides):
    kwargs = dict(BASE)
    kwargs.update(overrides)
    pose = kwargs.pop("pose", ((0.0, 0.0), 0.0))
    target = kwargs.pop("target", (5.0, 0.0))
    speed_pct = kwargs.pop("speed_pct", 0.5)
    return pathfinder(pose[0], target, pose[1], speed_pct, **kwargs)


def test_straight_ahead_zero_steering_full_throttle():
    throttle, steer = call(target=(5.0, 0.0))
    assert steer == pytest.approx(0.0, abs=1e-6)
    assert throttle == pytest.approx(100.0, rel=1e-6)


def test_target_left_commands_left_steering():
    _, steer = call(target=(5.0, 1.5))
    assert steer > 0.0  # left is positive in this project's convention


def test_target_right_commands_right_steering():
    _, steer = call(target=(5.0, -1.5))
    assert steer < 0.0


def test_steering_is_clamped_to_physical_limit():
    # Nearly sideways target should saturate the steering rack.
    _, steer = call(target=(0.1, 2.0))
    assert abs(steer) == pytest.approx(25.0, abs=1e-6)


def test_larger_lateral_offset_steers_harder_than_smaller():
    _, s_small = call(target=(5.0, 0.3))
    _, s_big = call(target=(5.0, 1.5))
    assert abs(s_big) > abs(s_small)


def test_curvature_regulation_slows_throttle_on_sharp_bend():
    # Tight target → high curvature → regulated velocity scaling engages.
    kw_off = dict(use_curvature_regulation=False)
    kw_on = dict(use_curvature_regulation=True, min_radius_m=12.0)
    target = (3.0, 2.0)  # sharp
    t_off, _ = call(target=target, **kw_off)
    t_on, _ = call(target=target, **kw_on)
    assert t_on < t_off
    # Regulated speed must still respect the minimum floor.
    assert t_on >= BASE["min_reg_speed_pct"] * 100.0 - 1e-6


def test_curvature_regulation_does_not_slow_gentle_bend():
    # Large radius → regulation should be a no-op.
    t_off, _ = call(target=(10.0, 0.1), use_curvature_regulation=False)
    t_on, _ = call(target=(10.0, 0.1), use_curvature_regulation=True, min_radius_m=12.0)
    assert t_on == pytest.approx(t_off, rel=1e-6)


def test_target_behind_vehicle_returns_crawl_speed():
    throttle, steer = call(
        pose=((0.0, 0.0), 0.0),
        target=(-2.0, 0.5),
        desired_speed_pct=1.0,
    )
    # Code path: x_v <= 0  →  return min(10%, desired) * 100, 0
    assert throttle == pytest.approx(10.0, abs=1e-6)
    assert steer == 0


def test_approach_distance_ramps_speed_down_near_end():
    # Enabling approach ramp with L < approach_dist should reduce throttle.
    t_no_ramp, _ = call(target=(0.5, 0.0), approach_dist_m=0.0)
    t_ramp, _ = call(target=(0.5, 0.0), approach_dist_m=2.0, min_approach_speed_pct=0.05)
    assert t_ramp < t_no_ramp
    assert t_ramp >= 0.05 * 100.0 - 1e-6


def test_zero_v_max_returns_zeros():
    throttle, steer = call(v_max_mps=0.0)
    assert (throttle, steer) == (0.0, 0.0)


def test_zero_wheelbase_returns_zeros():
    throttle, steer = call(wheelbase_m=0.0)
    assert (throttle, steer) == (0.0, 0.0)


def test_zero_steer_limit_returns_zeros():
    throttle, steer = call(steer_max_deg=0.0)
    assert (throttle, steer) == (0.0, 0.0)


def test_speed_pct_out_of_range_is_clamped_not_rejected():
    # Negative and >1 inputs should still produce a sane output.
    throttle_low, _ = call(speed_pct=-0.5)
    throttle_high, _ = call(speed_pct=5.0)
    assert 0.0 <= throttle_low <= 100.0
    assert 0.0 <= throttle_high <= 100.0


def test_yaw_rotation_is_respected():
    # Target directly "north" of a vehicle facing north is straight ahead.
    _, steer = call(
        pose=((0.0, 0.0), math.pi / 2),
        target=(0.0, 5.0),
    )
    assert steer == pytest.approx(0.0, abs=1e-6)
