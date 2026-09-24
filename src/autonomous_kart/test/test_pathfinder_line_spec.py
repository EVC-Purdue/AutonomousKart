"""
PathfinderNode's `pathfinder/line_spec` handling.

A spec names a shape plus the three speed knobs. Adopting it reloads the
shape's base CSV, reshapes the vx column, and rebuilds the planners on it.
MPC caps v_ref at its own target_speed independently of the line, so the
spec's v_max has to reach the rebuilt planner too.
"""
import json

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.pathfinder.pathfinder_node import PathfinderNode  # noqa: E402


def _params(line_path, line_dir):
    return {
        "simulation_mode": True,
        "system_frequency": 60,
        "system_state": "IDLE",
        "line_path": line_path,
        "line_dir": line_dir,
        "wheelbase_m": 1.05,
        "v_max_mps": 12.0,
        "steer_max_deg": 25.0,
        "steer_rate_max_degps": 180.0,
        "a_max_mps2": 2.0,
        "a_min_mps2": -3.0,
        "a_lat_max_mps2": 4.0,
        "planner": "pure_pursuit",
        "mpc.target_speed_mps": 10.0,
        # ResidualLearner refuses to construct without one.
        "mpc.residual.model_size": "xs",
    }


@pytest.fixture
def shape_dir(tmp_path):
    """Two shapes: `line1` (flat 10, 5 pts) and `center` (flat 8, 3 pts)."""
    d = tmp_path / "lines"
    d.mkdir()
    (d / "line1.csv").write_text(
        "".join(f"{i}.0,{i}.0,0.0,0.0,0.0,10.0,0.0\n" for i in range(5))
    )
    (d / "center.csv").write_text(
        "".join(f"{i}.0,{i}.0,1.0,0.0,0.0,8.0,0.0\n" for i in range(3))
    )
    return str(d)


def _spec_msg(**kwargs):
    from std_msgs.msg import String

    return String(data=json.dumps(kwargs))


def test_spec_loads_the_shape_and_scales_vx(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            node._on_line_spec(_spec_msg(shape="line1", v_max=6.0))

            assert len(node.racing_line) == 5
            assert [r[5] for r in node.racing_line] == pytest.approx([6.0] * 5)
        finally:
            node.destroy_node()


def test_spec_multiplier_applies_before_the_clamp(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            # 10 * 0.5 = 5.0, which is under the 8.0 ceiling.
            node._on_line_spec(_spec_msg(shape="line1", v_max=8.0, v_mult=0.5))

            assert [r[5] for r in node.racing_line] == pytest.approx([5.0] * 5)
        finally:
            node.destroy_node()


def test_spec_v_min_raises_the_floor(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            node._on_line_spec(
                _spec_msg(shape="line1", v_min=7.0, v_max=9.0, v_mult=0.1)
            )

            assert [r[5] for r in node.racing_line] == pytest.approx([7.0] * 5)
        finally:
            node.destroy_node()


def test_spec_switches_geometry(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            node._on_line_spec(_spec_msg(shape="center"))

            assert len(node.racing_line) == 3
            assert node.racing_line[0][2] == pytest.approx(1.0)  # center's y offset
        finally:
            node.destroy_node()


def test_spec_v_max_reaches_the_rebuilt_mpc(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            assert node.planners["mpc"].target_speed == pytest.approx(10.0)

            node._on_line_spec(_spec_msg(shape="line1", v_max=7.0))

            assert node.planners["mpc"].target_speed == pytest.approx(7.0)
        finally:
            node.destroy_node()


def test_spec_survives_a_later_planner_rebuild(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            node._on_line_spec(_spec_msg(shape="line1", v_max=7.0))
            node.planners = node._build_planners()

            assert node.planners["mpc"].target_speed == pytest.approx(7.0)
        finally:
            node.destroy_node()


def test_raw_path_swap_clears_the_spec(ros_ctx, tiny_racing_line, shape_dir):
    from std_msgs.msg import String

    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            node._on_line_spec(_spec_msg(shape="line1", v_max=7.0))
            assert node.speed_spec is not None

            node._on_line_swap(String(data=tiny_racing_line))

            assert node.speed_spec is None
            assert node.planners["mpc"].target_speed == pytest.approx(10.0)
        finally:
            node.destroy_node()


def test_unknown_shape_leaves_the_line_alone(ros_ctx, tiny_racing_line, shape_dir):
    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            before = list(node.racing_line)

            node._on_line_spec(_spec_msg(shape="nope"))

            assert node.racing_line == before
            assert node.speed_spec is None
        finally:
            node.destroy_node()


def test_invalid_spec_leaves_the_line_alone(ros_ctx, tiny_racing_line, shape_dir):
    from std_msgs.msg import String

    with ros_ctx(_params(tiny_racing_line, shape_dir)):
        node = PathfinderNode()
        try:
            before = list(node.racing_line)

            node._on_line_spec(_spec_msg(shape="line1", v_min=9.0, v_max=4.0))
            node._on_line_spec(String(data="not json"))

            assert node.racing_line == before
        finally:
            node.destroy_node()


def test_line_spec_topic_is_wired(ros_ctx, tiny_racing_line, shape_dir, spin_helper):
    from std_msgs.msg import String

    with ros_ctx(_params(tiny_racing_line, shape_dir)) as rclpy:
        node = PathfinderNode()
        driver = rclpy.create_node("spec_driver")
        pub = driver.create_publisher(String, "pathfinder/line_spec", 1)

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery
            msg = String(data=json.dumps({"shape": "center"}))
            adopted = spin_helper(
                exe,
                lambda: (pub.publish(msg), len(node.racing_line) == 3)[1],
                timeout=3.0,
            )

            assert adopted, "line_spec publish never reached the node"
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()
