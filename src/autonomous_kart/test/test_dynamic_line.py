"""
Unit tests for DynamicLineManager and RejoinStrategy.

These cover the strategy-selection FSM and the geometry of the
cubic-Bezier rejoin path without touching ROS.
"""
import math

import pytest

from autonomous_kart.nodes.pathfinder.dynamic_line import (
    DynamicLineManager,
    KartState,
    LineStrategy,
)
from autonomous_kart.nodes.pathfinder.strategies.rejoin import RejoinStrategy


# s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
def _straight_line(n=40, step=1.0, vx=10.0):
    return [(i * step, i * step, 0.0, 0.0, 0.0, vx, 0.0) for i in range(n)]


class _AlwaysOn(LineStrategy):
    priority = 10

    def __init__(self):
        self.activated = 0
        self.deactivated = 0

    def should_activate(self, state, line):
        return True

    def should_deactivate(self, state, line):
        return True  # one-shot

    def generate(self, state, line):
        return list(line[:5]), 4

    def on_activate(self, state):
        self.activated += 1

    def on_deactivate(self, state):
        self.deactivated += 1


class _NeverOn(LineStrategy):
    priority = 5

    def should_activate(self, state, line):
        return False

    def generate(self, state, line):  # pragma: no cover - never called
        return None


def test_manager_starts_inactive():
    m = DynamicLineManager(_straight_line())
    assert not m.is_active
    line, idx = m.get_line_and_idx()
    assert idx == -1
    assert line == m.racing_line


def test_register_sorts_by_priority():
    m = DynamicLineManager(_straight_line())
    low = _AlwaysOn()
    low.priority = 100
    high = _NeverOn()
    high.priority = 5
    m.register(low)
    m.register(high)
    assert m._strategies[0] is high  # lower number == higher priority first


def test_compute_cte_on_straight_line():
    m = DynamicLineManager(_straight_line())
    cte = m.compute_cte((3.0, 2.0), m.racing_line, 3)
    assert cte == pytest.approx(2.0, abs=1e-9)


def test_compute_cte_empty_line_is_zero():
    m = DynamicLineManager([])
    assert m.compute_cte((1.0, 1.0), [], 0) == 0.0


def test_activation_generates_dynamic_line_and_calls_hooks():
    m = DynamicLineManager(_straight_line())
    strat = _AlwaysOn()
    # Neutralize deactivation so we can observe an active cycle.
    strat.should_deactivate = lambda s, l: False
    m.register(strat)

    m.update(KartState(xy=(0.0, 0.0)))
    assert m.is_active
    assert strat.activated == 1
    line, idx = m.get_line_and_idx()
    assert len(line) == 5
    assert idx == 0


def test_deactivation_restores_racing_line_and_snaps_idx():
    m = DynamicLineManager(_straight_line())
    strat = _AlwaysOn()  # activates then immediately wants out
    m.register(strat)

    state = KartState(xy=(0.0, 0.0), closest_idx=0)
    m.update(state)  # activate
    m.update(state)  # deactivate on next tick
    assert not m.is_active
    # The manager should snap state.closest_idx to merge_idx (4) on handoff.
    assert state.closest_idx == 4


def test_end_of_dynamic_line_forces_deactivation():
    m = DynamicLineManager(_straight_line())

    class Holds(LineStrategy):
        priority = 10

        def should_activate(self, state, line):
            return True

        def should_deactivate(self, state, line):
            return False  # never wants to quit on its own

        def generate(self, state, line):
            return list(line[:6]), 5

    m.register(Holds())
    state = KartState(xy=(0.0, 0.0))
    m.update(state)
    assert m.is_active

    # Push progress to near the end of a 6-point dynamic line.
    m.dynamic_closest_idx = 2  # len - 5 == 1, so 2 >= 1 triggers deactivation
    m.update(state)
    assert not m.is_active


# ---- RejoinStrategy -----------------------------------------------------


def test_rejoin_activates_above_threshold_and_not_below():
    strat = RejoinStrategy(cte_activate=3.0, cte_deactivate=1.0)
    line = _straight_line()
    assert strat.should_activate(KartState(cross_track_error=4.0), line)
    assert not strat.should_activate(KartState(cross_track_error=2.0), line)
    assert strat.should_deactivate(KartState(cross_track_error=0.5), line)
    assert not strat.should_deactivate(KartState(cross_track_error=2.0), line)


def test_rejoin_generates_waypoints_that_start_at_kart_and_end_on_line():
    strat = RejoinStrategy(cte_activate=3.0, merge_lookahead_m=10.0, min_turning_radius=3.0)
    line = _straight_line(n=80)
    state = KartState(
        xy=(5.0, 4.0),  # 4m off the racing line
        yaw=0.0,
        closest_idx=5,
        cross_track_error=4.0,
    )
    result = strat.generate(state, line)
    assert result is not None
    pts, merge_idx = result
    assert len(pts) >= 10
    # First waypoint should be at the kart's current pose.
    first = pts[0]
    assert math.hypot(first[1] - state.xy[0], first[2] - state.xy[1]) < 1e-6
    # Last waypoint should coincide with the chosen merge point on the racing line.
    last = pts[-1]
    merge_xy = (line[merge_idx][1], line[merge_idx][2])
    assert math.hypot(last[1] - merge_xy[0], last[2] - merge_xy[1]) < 1e-6


def test_rejoin_waypoint_speed_is_monotonically_ramping():
    strat = RejoinStrategy(cte_activate=3.0, rejoin_speed_pct=0.4)
    line = _straight_line(n=80)
    state = KartState(xy=(5.0, 4.0), yaw=0.0, closest_idx=5, cross_track_error=4.0)
    pts, _ = strat.generate(state, line)
    speeds = [p[5] for p in pts]
    assert speeds[0] < speeds[-1]
    # Non-strict monotonicity (quadratic ramp).
    for a, b in zip(speeds, speeds[1:]):
        assert b + 1e-9 >= a


def test_rejoin_returns_none_for_tiny_line():
    strat = RejoinStrategy()
    assert strat.generate(KartState(xy=(0.0, 0.0)), [(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)]) is None
