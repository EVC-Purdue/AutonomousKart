"""
Regression tests for the three closest-idx fixes in pathfinder_node:

1. Stale racing-line closest_idx while a dynamic line is active
2. Windowed forward search wrapping modulo on short non-closed lines
3. Initial closest_idx sync when the kart spawns far from index 0

The two search helpers are pure staticmethods, so we test them directly
without rclpy. The "stale-CTE while dynamic line active" and "initial
full-track sync" behaviors are tested at the node level.
"""
import math

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.pathfinder.pathfinder_node import PathfinderNode  # noqa: E402
from autonomous_kart.nodes.pathfinder.dynamic_line import (  # noqa: E402
    DynamicLineManager,
    KartState,
)


# Row shape: (s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2)
def _straight_line(n=40, step=1.0, vx=10.0):
    return [(i * step, i * step, 0.0, 0.0, 0.0, vx, 0.0) for i in range(n)]


# Pure-helper tests (no ROS)


def test_nearest_idx_forward_no_wrap_on_short_open_line():
    """Issue 2: a short, non-closed line (e.g. a Bezier rejoin) must not
    wrap modulo back to the start when the search window exceeds the
    remaining length of the line.

    Scenario: a Bezier-like 51-pt line where the kart is mid-line (far from
    idx 0 on the line but close to it in map space — idx 0 is the start
    of the Bezier, i.e. where the kart was when rejoin began). The forward
    scan from dyn_idx must stay ahead of dyn_idx rather than wrap back
    and snap to idx 0 behind the kart.
    """
    # Straight line along +x from x=0 to x=50, but with idx 0 relocated so
    # it sits directly under where the kart currently is. This models the
    # Bezier start-point being near the kart's current location.
    kart_xy = (25.3, 0.0)
    line = [(0.0, kart_xy[0], kart_xy[1], 0.0, 0.0, 10.0, 0.0)] + [
        (float(i), float(i), 0.0, 0.0, 0.0, 10.0, 0.0) for i in range(1, 51)
    ]

    # With allow_wrap=False, the search starting from dyn_idx=25 must stay
    # in [25..50] and not wrap back to idx 0 even though idx 0 is right
    # underneath the kart.
    idx_no_wrap = PathfinderNode._nearest_idx_forward(
        line, kart_xy, start_idx=25, window=80, allow_wrap=False
    )
    assert 25 <= idx_no_wrap <= 50, (
        f"no-wrap search snapped to {idx_no_wrap}, should stay forward of 25"
    )
    # Specifically, the nearest forward-only point to (25.3, 0) on the
    # 1m-spaced straight is idx 25 (at x=25) or idx 26 (at x=26).
    assert idx_no_wrap in (25, 26)

    # Sanity: with allow_wrap=True the modular search visits idx 0 (the
    # lure) and picks it — reproducing the original bug.
    idx_wrap = PathfinderNode._nearest_idx_forward(
        line, kart_xy, start_idx=25, window=80, allow_wrap=True
    )
    assert idx_wrap == 0


def test_nearest_idx_forward_wrap_ok_on_closed_line():
    """Racing line (closed, thousands of points) still benefits from wrap
    when closest_idx is near the end and the actual nearest is past the
    seam."""
    # Square-ish closed loop made of 100 points returning to origin.
    n = 100
    pts = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        pts.append((i * 1.0, math.cos(theta), math.sin(theta), 0.0, 0.0, 10.0, 0.0))
    # Kart sits near (1, 0) which is closest to idx 0 (and idx n-1 is also
    # near it because it's a closed loop).
    xy = (1.0, 0.0)
    # Start at idx n-5, allow_wrap=True → must wrap to idx 0.
    idx = PathfinderNode._nearest_idx_forward(
        pts, xy, start_idx=n - 5, window=20, allow_wrap=True
    )
    # Accept idx in {n-4..n-1, 0, 1} — either side of the seam.
    assert idx in set(range(n - 4, n)) | {0, 1}


def test_nearest_idx_forward_clamps_start_idx():
    line = _straight_line(n=10)
    # Out-of-range start_idx should be treated as 0, not crash.
    idx = PathfinderNode._nearest_idx_forward(
        line, (5.0, 0.0), start_idx=999, window=80, allow_wrap=False
    )
    assert idx == 5


def test_nearest_idx_forward_empty_line():
    assert (
        PathfinderNode._nearest_idx_forward([], (0.0, 0.0), 0, 80, True) == 0
    )


def test_full_nearest_idx_picks_absolute_best():
    """Issue 3: full search must find the true nearest even when kart is
    far from the last-known idx."""
    line = _straight_line(n=100)
    # Kart at (73, 0) → true nearest is idx 73.
    xy = (73.0, 0.0)
    assert PathfinderNode._full_nearest_idx(line, xy) == 73


def test_full_nearest_idx_empty_line():
    assert PathfinderNode._full_nearest_idx([], (1.0, 2.0)) == 0


# pick_lookahead_point is now pure


def test_pick_lookahead_point_is_pure(ros_ctx, tiny_racing_line):
    """After the refactor pick_lookahead_point takes (line, idx, L) and
    does not read/mutate self.racing_line or self.closest_idx."""
    with ros_ctx(
        {
            "simulation_mode": True,
            "system_frequency": 60,
            "system_state": "IDLE",
            "max_speed": 50,
            "acceleration": 0.2,
            "max_steering": 10,
            "steering_accel": 0.05,
            "line_path": tiny_racing_line,
            "wheelbase_m": 1.05,
            "v_max_mps": 15.0,
            "steer_max_deg": 25.0,
            "use_velocity_scaled_lookahead": True,
            "lookahead_time_s": 0.6,
            "min_lookahead_m": 3.0,
            "max_lookahead_m": 8.0,
            "use_curvature_regulation": True,
            "min_radius_m": 12.0,
            "min_reg_speed_pct": 0.20,
            "approach_dist_m": 1.0,
            "min_approach_speed_pct": 0.05,
        }
    ):
        node = PathfinderNode()
        try:
            node.current_xy = (0.0, 0.0)
            # Feed a short synthetic line instead of the racing line.
            short = _straight_line(n=5, step=1.0, vx=10.0)
            pre_racing = node.racing_line
            pre_idx = node.closest_idx
            tgt, _spd = node.pick_lookahead_point(short, 0, lookahead_m=2.0)
            # Should target the first point in `short` where s >= s_target,
            # which is idx 2 at s=2.0.
            assert tgt[0] == pytest.approx(2.0)
            # pick_lookahead_point must not have rebound self.racing_line
            assert node.racing_line is pre_racing
            assert node.closest_idx == pre_idx
        finally:
            node.destroy_node()



def _params(line_path, system_state="AUTONOMOUS"):
    return {
        "simulation_mode": True,
        "system_frequency": 60,
        "system_state": system_state,
        "max_speed": 50,
        "acceleration": 0.2,
        "max_steering": 10,
        "steering_accel": 0.05,
        "line_path": line_path,
        "wheelbase_m": 1.05,
        "v_max_mps": 15.0,
        "steer_max_deg": 25.0,
        "use_velocity_scaled_lookahead": True,
        "lookahead_time_s": 0.6,
        "min_lookahead_m": 3.0,
        "max_lookahead_m": 8.0,
        "use_curvature_regulation": True,
        "min_radius_m": 12.0,
        "min_reg_speed_pct": 0.20,
        "approach_dist_m": 1.0,
        "min_approach_speed_pct": 0.05,
    }


_LONG_LINE_CSV_HEADER = "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n"


def _write_long_straight_line(path, n=200, step=1.0, vx=10.0):
    lines = [_LONG_LINE_CSV_HEADER]
    for i in range(n):
        lines.append(f"{i * step},{i * step},0.0,0.0,0.0,{vx},0.0\n")
    path.write_text("".join(lines))
    return str(path)


@pytest.fixture
def long_racing_line(tmp_path):
    return _write_long_straight_line(tmp_path / "long.csv", n=200)


def _make_odom(x=0.0, y=0.0):
    from nav_msgs.msg import Odometry

    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_racing_line_closest_idx_advances_while_dynamic_line_active(
    ros_ctx, long_racing_line
):
    """Issue 1: while a dynamic strategy owns pick_lookahead_point, the
    racing-line closest_idx must still advance with the kart so CTE is not
    computed against a frozen waypoint.
    """
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_params(long_racing_line, "AUTONOMOUS")):
        node = PathfinderNode()
        try:
            # Install an always-active strategy that returns a long-enough
            # dynamic line so the manager does not auto-deactivate at the
            # end-of-line check.
            from autonomous_kart.nodes.pathfinder.dynamic_line import LineStrategy

            class _Sticky(LineStrategy):
                priority = 1

                def should_activate(self, s, l):
                    return True

                def should_deactivate(self, s, l):
                    return False

                def generate(self, s, l):
                    # Synthetic dynamic line: 60 points along +x from kart,
                    # covering 60 m → longer than the end-of-line threshold
                    # (len - 5) check.
                    cx, cy = s.xy
                    pts = [
                        (float(i), cx + float(i), cy, 0.0, 0.0, 5.0)
                        for i in range(60)
                    ]
                    # Merge index on racing line: 60 m further along +x.
                    target_idx = min(
                        int(cx + 60.0), len(l) - 1
                    )  # racing line is spaced 1 m
                    return pts, target_idx

            # Replace registered strategies with ours.
            node.line_manager._strategies = []
            node.line_manager.register(_Sticky())

            # Start kart at origin; first autonomous tick will do the full
            # initial sync and set closest_idx = 0.
            node.current_xy = (0.0, 0.0)
            node.current_yaw = 0.0
            node.current_speed_mps = 0.0
            node.pose_ready = True

            msg = Float32MultiArray(data=[0.0, 0.0])
            # Tick 1: initial sync + activate dynamic line.
            node.autonomous_control_loop(msg)
            assert node.line_manager.is_active
            idx_at_start = node.closest_idx
            assert idx_at_start == 0

            # Simulate the kart driving along the dynamic line: move it 50 m
            # forward without deactivating the strategy. After each pose
            # update, the racing-line closest_idx MUST advance.
            for step_x in (10.0, 20.0, 30.0, 40.0, 50.0):
                node.current_xy = (step_x, 0.0)
                node.autonomous_control_loop(msg)
                # Dynamic line is still active.
                assert node.line_manager.is_active
                # Racing-line closest_idx (on the long 1 m-spaced straight
                # line) should have advanced to ~step_x.
                assert abs(node.closest_idx - int(step_x)) <= 2, (
                    f"racing-line closest_idx stuck at {node.closest_idx} "
                    f"while kart moved to x={step_x}"
                )

            # CTE (via compute_cte) must now be small: the advanced
            # closest_idx points at a waypoint right under the kart.
            cte = node.line_manager.compute_cte(
                node.current_xy, node.racing_line, node.closest_idx
            )
            assert cte < 1.0, f"CTE inflated ({cte:.2f}m) — closest_idx stale"
        finally:
            node.destroy_node()


def test_initial_sync_picks_true_nearest_when_kart_spawns_far_from_idx0(
    ros_ctx, long_racing_line
):
    """Issue 3: if the kart's first pose lands far from racing_line[0],
    the first autonomous tick must do an O(n) search rather than trust
    the default closest_idx=0."""
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_params(long_racing_line, "AUTONOMOUS")):
        node = PathfinderNode()
        try:
            node.current_xy = (150.0, 0.0)  # well beyond the 80-window from 0
            node.current_yaw = 0.0
            node.current_speed_mps = 0.0
            node.pose_ready = True

            assert node.closest_idx == 0
            assert not node.initial_sync_done

            node.autonomous_control_loop(Float32MultiArray(data=[0.0, 0.0]))

            assert node.initial_sync_done
            # True nearest on a 1 m-spaced straight is idx 150.
            assert abs(node.closest_idx - 150) <= 1, (
                f"initial sync failed to find true nearest; got "
                f"{node.closest_idx}"
            )
        finally:
            node.destroy_node()


def test_resync_on_large_localization_jump(ros_ctx, long_racing_line):
    """Bonus: a huge jump (localization glitch, operator re-poses the kart)
    triggers another full search so the windowed scan doesn't stay stuck."""
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_params(long_racing_line, "AUTONOMOUS")):
        node = PathfinderNode()
        try:
            node.current_xy = (0.0, 0.0)
            node.current_yaw = 0.0
            node.current_speed_mps = 0.0
            node.pose_ready = True

            # Establish initial sync at idx 0.
            node.autonomous_control_loop(Float32MultiArray(data=[0.0, 0.0]))
            assert node.closest_idx == 0

            # Teleport to x=180 — outside the bounded search window (80).
            node.current_xy = (180.0, 0.0)
            node.autonomous_control_loop(Float32MultiArray(data=[0.0, 0.0]))
            # Resync branch should have fired (distance 180 > 20 m).
            assert abs(node.closest_idx - 180) <= 1
        finally:
            node.destroy_node()
