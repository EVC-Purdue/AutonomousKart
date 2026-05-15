"""
Regression tests for the three closest-idx behaviors in the pure pursuit planner:

1. Stale racing-line closest_idx while a dynamic line is active
2. Windowed forward search wrapping modulo on short non-closed lines
3. Initial closest_idx sync when the kart spawns far from index 0

The two search helpers are pure staticmethods on PurePursuitPlanner, so we test
them directly without rclpy. The "stale-CTE while dynamic line active" and
"initial full-track sync" behaviors are tested at the node level by calling
the timer-driven autonomous tick directly.
"""
import math

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.pathfinder.pathfinder_node import PathfinderNode  # noqa: E402
from autonomous_kart.nodes.pathfinder.planners.pure_pursuit import PurePursuitPlanner  # noqa: E402
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
    """
    kart_xy = (25.3, 0.0)
    line = [(0.0, kart_xy[0], kart_xy[1], 0.0, 0.0, 10.0, 0.0)] + [
        (float(i), float(i), 0.0, 0.0, 0.0, 10.0, 0.0) for i in range(1, 51)
    ]

    idx_no_wrap = PurePursuitPlanner._nearest_idx_forward(
        line, kart_xy, start_idx=25, window=80, allow_wrap=False
    )
    assert 25 <= idx_no_wrap <= 50, (
        f"no-wrap search snapped to {idx_no_wrap}, should stay forward of 25"
    )
    assert idx_no_wrap in (25, 26)

    idx_wrap = PurePursuitPlanner._nearest_idx_forward(
        line, kart_xy, start_idx=25, window=80, allow_wrap=True
    )
    assert idx_wrap == 0


def test_nearest_idx_forward_wrap_ok_on_closed_line():
    n = 100
    pts = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        pts.append((i * 1.0, math.cos(theta), math.sin(theta), 0.0, 0.0, 10.0, 0.0))
    xy = (1.0, 0.0)
    idx = PurePursuitPlanner._nearest_idx_forward(
        pts, xy, start_idx=n - 5, window=20, allow_wrap=True
    )
    assert idx in set(range(n - 4, n)) | {0, 1}


def test_nearest_idx_forward_clamps_start_idx():
    line = _straight_line(n=10)
    idx = PurePursuitPlanner._nearest_idx_forward(
        line, (5.0, 0.0), start_idx=999, window=80, allow_wrap=False
    )
    assert idx == 5


def test_nearest_idx_forward_empty_line():
    assert (
        PurePursuitPlanner._nearest_idx_forward([], (0.0, 0.0), 0, 80, True) == 0
    )


def test_full_nearest_idx_picks_absolute_best():
    line = _straight_line(n=100)
    xy = (73.0, 0.0)
    assert PurePursuitPlanner._full_nearest_idx(line, xy) == 73


def test_full_nearest_idx_empty_line():
    assert PurePursuitPlanner._full_nearest_idx([], (1.0, 2.0)) == 0


# pick_lookahead_point is pure on the planner


def test_pick_lookahead_point_is_pure(ros_ctx, tiny_racing_line):
    """The planner's _pick_lookahead_point takes (line, idx, L, current_xy)
    and does not read/mutate the planner's racing_line or closest_idx."""
    with ros_ctx(
        {
            "simulation_mode": True,
            "system_frequency": 60,
            "system_state": "IDLE",
            "line_path": tiny_racing_line,
            "wheelbase_m": 1.05,
            "v_max_mps": 15.0,
            "steer_max_deg": 25.0,
            "steer_rate_max_degps": 180.0,
            "a_max_mps2": 2.0,
            "a_min_mps2": -3.0,
            "a_lat_max_mps2": 4.0,
            "planner": "pure_pursuit",
            "pure_pursuit.use_velocity_scaled_lookahead": True,
            "pure_pursuit.lookahead_time_s": 0.6,
            "pure_pursuit.min_lookahead_m": 3.0,
            "pure_pursuit.max_lookahead_m": 8.0,
            "pure_pursuit.use_curvature_regulation": True,
            "pure_pursuit.min_radius_m": 12.0,
            "pure_pursuit.min_reg_speed_mps": 3.0,
            "pure_pursuit.approach_dist_m": 1.0,
            "pure_pursuit.min_approach_speed_mps": 0.75,
            "pure_pursuit.search_window": 80,
            "pure_pursuit.initial_sync_done": False,
            "pure_pursuit.max_resync_dist": 80.0,
            "pure_pursuit.max_closed_dist": 2.0,
        }
    ):
        node = PathfinderNode()
        try:
            node.current_xy = (0.0, 0.0)
            short = _straight_line(n=5, step=1.0, vx=10.0)
            pre_racing = node.racing_line
            pre_idx = node.planner.closest_idx
            tgt, _spd = node.planner._pick_lookahead_point(
                short, 0, lookahead_m=2.0, current_xy=node.current_xy
            )
            assert tgt[0] == pytest.approx(2.0)
            assert node.racing_line is pre_racing
            assert node.planner.closest_idx == pre_idx
        finally:
            node.destroy_node()



def _params(line_path, system_state="AUTONOMOUS"):
    return {
        "simulation_mode": True,
        "system_frequency": 60,
        "system_state": system_state,
        "line_path": line_path,
        "wheelbase_m": 1.05,
        "v_max_mps": 15.0,
        "steer_max_deg": 25.0,
        "steer_rate_max_degps": 180.0,
        "a_max_mps2": 2.0,
        "a_min_mps2": -3.0,
        "a_lat_max_mps2": 4.0,
        "planner": "pure_pursuit",
        "pure_pursuit.use_velocity_scaled_lookahead": True,
        "pure_pursuit.lookahead_time_s": 0.6,
        "pure_pursuit.min_lookahead_m": 3.0,
        "pure_pursuit.max_lookahead_m": 8.0,
        "pure_pursuit.use_curvature_regulation": True,
        "pure_pursuit.min_radius_m": 12.0,
        "pure_pursuit.min_reg_speed_mps": 3.0,
        "pure_pursuit.approach_dist_m": 1.0,
        "pure_pursuit.min_approach_speed_mps": 0.75,
        "pure_pursuit.search_window": 80,
        "pure_pursuit.initial_sync_done": False,
        "pure_pursuit.max_resync_dist": 20.0,
        "pure_pursuit.max_closed_dist": 2.0,
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
    """Issue 1: while a dynamic strategy owns the lookahead pick, the
    racing-line closest_idx must still advance with the kart so CTE is not
    computed against a frozen waypoint.
    """
    with ros_ctx(_params(long_racing_line, "AUTONOMOUS")):
        node = PathfinderNode()
        try:
            from autonomous_kart.nodes.pathfinder.dynamic_line import LineStrategy

            class _Sticky(LineStrategy):
                priority = 1

                def should_activate(self, s, l):
                    return True

                def should_deactivate(self, s, l):
                    return False

                def generate(self, s, l):
                    cx, cy = s.xy
                    pts = [
                        (float(i), cx + float(i), cy, 0.0, 0.0, 5.0)
                        for i in range(60)
                    ]
                    target_idx = min(int(cx + 60.0), len(l) - 1)
                    return pts, target_idx

            node.planner.line_manager._strategies = []
            node.planner.line_manager.register(_Sticky())

            node.current_xy = (0.0, 0.0)
            node.current_yaw = 0.0
            node.current_speed_mps = 0.0
            node.pose_ready = True

            # Tick 1: initial sync + activate dynamic line.
            node._autonomous_tick()
            assert node.planner.line_manager.is_active
            idx_at_start = node.planner.closest_idx
            assert idx_at_start == 0

            for step_x in (10.0, 20.0, 30.0, 40.0, 50.0):
                node.current_xy = (step_x, 0.0)
                node._autonomous_tick()
                assert node.planner.line_manager.is_active
                assert abs(node.planner.closest_idx - int(step_x)) <= 2, (
                    f"racing-line closest_idx stuck at {node.planner.closest_idx} "
                    f"while kart moved to x={step_x}"
                )

            cte = node.planner.line_manager.compute_cte(
                node.current_xy, node.racing_line, node.planner.closest_idx
            )
            assert cte < 1.0, f"CTE inflated ({cte:.2f}m) closest_idx stale"
        finally:
            node.destroy_node()


def test_initial_sync_picks_true_nearest_when_kart_spawns_far_from_idx0(
    ros_ctx, long_racing_line
):
    """Issue 3: if the kart's first pose lands far from racing_line[0],
    the first autonomous tick must do an O(n) search rather than trust
    the default closest_idx=0."""
    with ros_ctx(_params(long_racing_line, "AUTONOMOUS")):
        node = PathfinderNode()
        try:
            node.current_xy = (150.0, 0.0)
            node.current_yaw = 0.0
            node.current_speed_mps = 0.0
            node.pose_ready = True

            assert node.planner.closest_idx == 0
            assert not node.planner.initial_sync_done

            node._autonomous_tick()

            assert node.planner.initial_sync_done
            assert abs(node.planner.closest_idx - 150) <= 1, (
                f"initial sync failed to find true nearest; got "
                f"{node.planner.closest_idx}"
            )
        finally:
            node.destroy_node()


def test_resync_on_large_localization_jump(ros_ctx, long_racing_line):
    """A huge jump triggers another full search so the windowed scan
    doesn't stay stuck."""
    with ros_ctx(_params(long_racing_line, "AUTONOMOUS")):
        node = PathfinderNode()
        try:
            node.current_xy = (0.0, 0.0)
            node.current_yaw = 0.0
            node.current_speed_mps = 0.0
            node.pose_ready = True

            node._autonomous_tick()
            assert node.planner.closest_idx == 0

            node.current_xy = (180.0, 0.0)
            node._autonomous_tick()
            assert abs(node.planner.closest_idx - 180) <= 1
        finally:
            node.destroy_node()
