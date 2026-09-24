"""
MasterNode's racing-line shape API.

`get_line_shapes` reports what is on disk plus the defaults a caller needs to
fill a form, and `set_line_speed` validates a spec and publishes it. Both keep
`self.path` pointing at the line actually in use, so /map, /lines and
/racing_line stop serving the boot line after a swap.
"""
import json

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.master.master_node import MasterNode  # noqa: E402


@pytest.fixture
def shape_dir(tmp_path):
    d = tmp_path / "lines"
    d.mkdir()
    (d / "line1.csv").write_text(
        "".join(f"{i}.0,{i}.0,0.0,0.0,0.0,10.0,0.0\n" for i in range(5))
    )
    (d / "line2.csv").write_text(
        "".join(f"{i}.0,{i}.0,0.0,0.0,0.0,6.0,0.0\n" for i in range(5))
    )
    (d / "center.csv").write_text(
        "".join(f"{i}.0,{i}.0,1.0,0.0,0.0,8.0,0.0\n" for i in range(3))
    )
    return str(d)


def _params(shape_dir, **extra):
    p = {
        "system_state": "IDLE",
        "system_frequency": 60,
        "line_dir": shape_dir,
        "v_max_mps": 12.0,
        "mpc.target_speed_mps": 10.0,
    }
    p.update(extra)
    return p


def test_get_line_shapes_lists_what_is_on_disk(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            out = node.get_line_shapes()

            assert [s["shape"] for s in out["shapes"]] == ["center", "line1", "line2"]
        finally:
            node.destroy_node()


def test_get_line_shapes_reports_each_shapes_file(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            shapes = {s["shape"]: s for s in node.get_line_shapes()["shapes"]}

            assert shapes["line1"]["file"].endswith("line1.csv")
            assert shapes["line1"]["vx_max"] == pytest.approx(10.0)
        finally:
            node.destroy_node()


def test_get_line_shapes_defaults_v_max_to_the_mpc_target_speed(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            defaults = node.get_line_shapes()["defaults"]

            assert defaults["v_max"] == pytest.approx(10.0)
            assert defaults["v_min"] == pytest.approx(0.0)
            assert defaults["v_mult"] == pytest.approx(1.0)
            assert defaults["v_max_limit"] == pytest.approx(12.0)
        finally:
            node.destroy_node()


def test_get_line_shapes_has_no_active_spec_before_any_swap(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            assert node.get_line_shapes()["active"] is None
        finally:
            node.destroy_node()


def test_set_line_speed_publishes_the_normalized_spec(ros_ctx, shape_dir, spin_helper):
    from std_msgs.msg import String

    with ros_ctx(_params(shape_dir)) as rclpy:
        node = MasterNode()
        driver = rclpy.create_node("spec_listener")
        seen = []
        driver.create_subscription(
            String, "pathfinder/line_spec", lambda m: seen.append(m.data), 1
        )

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery
            ok, _ = node.set_line_speed({"shape": "line1", "v_max": 6.0})
            assert ok
            assert spin_helper(exe, lambda: bool(seen), timeout=3.0)

            assert json.loads(seen[-1]) == {
                "shape": "line1", "v_min": 0.0, "v_max": 6.0, "v_mult": 1.0,
            }
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()


def test_set_line_speed_points_self_path_at_the_shapes_file(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            node.set_line_speed({"shape": "line1", "v_max": 6.0})

            assert node.path.endswith("line1.csv")
        finally:
            node.destroy_node()


def test_set_line_speed_records_the_active_spec(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            node.set_line_speed({"shape": "line1", "v_mult": 0.5})

            assert node.get_line_shapes()["active"]["v_mult"] == pytest.approx(0.5)
        finally:
            node.destroy_node()


def test_set_line_speed_rejects_an_unknown_shape(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            ok, reason = node.set_line_speed({"shape": "nope"})

            assert not ok
            assert "nope" in reason
        finally:
            node.destroy_node()


def test_set_line_speed_rejects_v_min_above_v_max(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            ok, reason = node.set_line_speed(
                {"shape": "line1", "v_min": 9.0, "v_max": 4.0}
            )

            assert not ok
            assert "v_min" in reason
        finally:
            node.destroy_node()


def test_set_line_speed_rejects_v_max_above_the_kart_limit(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            ok, reason = node.set_line_speed({"shape": "line1", "v_max": 99.0})

            assert not ok
            assert "12" in reason
        finally:
            node.destroy_node()


def test_set_line_updates_self_path(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            target = shape_dir + "/center.csv"

            ok, _ = node.set_line(target)

            assert ok
            assert node.path == target
        finally:
            node.destroy_node()


def test_set_line_rejects_a_missing_file(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            before = node.path

            ok, reason = node.set_line(shape_dir + "/absent.csv")

            assert not ok
            assert "absent.csv" in reason
            assert node.path == before
        finally:
            node.destroy_node()


def test_set_line_clears_the_active_spec(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            node.set_line_speed({"shape": "line1", "v_max": 6.0})

            node.set_line(shape_dir + "/center.csv")

            assert node.get_line_shapes()["active"] is None
        finally:
            node.destroy_node()
