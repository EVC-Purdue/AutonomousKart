"""
Unit tests for SafetyChecker, the chokepoint between the active planner and
cmd_drive that reacts to the CV pathfinder's track_angles. Pure Python, no
rclpy dependency (mirrors how the planner pure-helper tests are structured).
"""
import math

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, PlannerInputs
from autonomous_kart.nodes.pathfinder.safety_checker import SafetyChecker

KART = KartConstants(
    v_max_mps=12.0,
    wheelbase_m=1.05,
    steer_max_deg=25.0,
    steer_rate_max_degps=180.0,
    a_max_mps2=5.0,
    a_min_mps2=-3.0,
    a_lat_max_mps2=5.0,
)

PARAMS = {
    "enabled": True,
    "mode": "blend",
    "warn_angle_deg": 30.0,
    "critical_angle_deg": 50.0,
    "max_correction_deg": 15.0,
    "min_speed_at_critical_mps": 2.0,
    "camera_only_speed_mps": 3.0,
    "camera_steer_gain": 1.0,
}

PROPOSED = (8.0, 4.0)  # (speed_mps, steering_deg) as if MPC proposed this


def _inputs(track_angles):
    return PlannerInputs(
        pose_xy=(0.0, 0.0), yaw_rad=0.0, speed_mps=8.0,
        track_angles=track_angles, now_ns=0,
    )


def _checker(**overrides):
    params = dict(PARAMS, **overrides)
    return SafetyChecker(params, KART)


# Fail-open


def test_passthrough_when_no_track_angles_ever_received():
    sc = _checker()
    assert sc.check(PROPOSED, _inputs(None)) == PROPOSED


def test_passthrough_when_right_angle_is_nan():
    sc = _checker()
    assert sc.check(PROPOSED, _inputs((math.nan, 10.0))) == PROPOSED


def test_passthrough_when_left_angle_is_nan():
    sc = _checker()
    assert sc.check(PROPOSED, _inputs((10.0, math.nan))) == PROPOSED


def test_passthrough_when_disabled_regardless_of_angles():
    sc = _checker(enabled=False, mode="hard_override")
    assert sc.check(PROPOSED, _inputs((80.0, 5.0))) == PROPOSED


# blend mode


def test_blend_passthrough_below_warn_threshold():
    sc = _checker(mode="blend")
    assert sc.check(PROPOSED, _inputs((10.0, 5.0))) == PROPOSED


def test_blend_pinned_to_full_correction_at_critical():
    sc = _checker(mode="blend")
    # right (60) >= left (5) -> encroaching right -> correct left (+)
    speed, steer = sc.check(PROPOSED, _inputs((60.0, 5.0)))
    assert steer == 15.0  # +max_correction_deg
    assert speed == 2.0  # min_speed_at_critical_mps


def test_blend_pinned_correction_is_negative_when_left_encroaches():
    sc = _checker(mode="blend")
    speed, steer = sc.check(PROPOSED, _inputs((5.0, 60.0)))
    assert steer == -15.0


def test_blend_interpolates_at_midpoint():
    sc = _checker(mode="blend")
    # worst=40 -> t = (40-30)/(50-30) = 0.5
    speed, steer = sc.check(PROPOSED, _inputs((40.0, 5.0)))
    expected_steer = 0.5 * PROPOSED[1] + 0.5 * 15.0
    expected_speed = 0.5 * PROPOSED[0] + 0.5 * 2.0
    assert steer == expected_steer
    assert speed == expected_speed


# hard_override mode


def test_hard_override_passthrough_below_critical():
    sc = _checker(mode="hard_override")
    assert sc.check(PROPOSED, _inputs((49.9, 5.0))) == PROPOSED


def test_hard_override_forces_fixed_correction_at_critical():
    sc = _checker(mode="hard_override")
    speed, steer = sc.check(PROPOSED, _inputs((50.0, 5.0)))
    assert (speed, steer) == (2.0, 15.0)


# camera_only mode


def test_camera_only_ignores_proposed_and_uses_fixed_speed():
    sc = _checker(mode="camera_only")
    speed, _ = sc.check(PROPOSED, _inputs((10.0, 4.0)))
    assert speed == 3.0  # camera_only_speed_mps, not PROPOSED's 8.0


def test_camera_only_steers_left_when_right_encroaches():
    sc = _checker(mode="camera_only")
    _, steer = sc.check(PROPOSED, _inputs((20.0, 5.0)))
    assert steer > 0.0


def test_camera_only_steers_right_when_left_encroaches():
    sc = _checker(mode="camera_only")
    _, steer = sc.check(PROPOSED, _inputs((5.0, 20.0)))
    assert steer < 0.0


def test_camera_only_falls_back_to_proposed_when_angles_missing():
    sc = _checker(mode="camera_only")
    assert sc.check(PROPOSED, _inputs(None)) == PROPOSED


# Final clamp, regardless of mode


def test_correction_never_exceeds_kart_limits():
    sc = _checker(mode="hard_override", max_correction_deg=999.0,
                   min_speed_at_critical_mps=999.0)
    speed, steer = sc.check(PROPOSED, _inputs((60.0, 5.0)))
    assert steer == KART.steer_max_deg
    assert speed == KART.v_max_mps


# set_mode


def test_set_mode_rejects_unknown_value():
    sc = _checker(mode="blend")
    assert sc.set_mode("bogus") is False
    assert sc.mode == "blend"


def test_set_mode_switches_behavior_live():
    sc = _checker(mode="blend")
    assert sc.set_mode("camera_only") is True
    speed, _ = sc.check(PROPOSED, _inputs((10.0, 4.0)))
    assert speed == 3.0
