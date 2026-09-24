"""MasterNode's racing-line shape API: one CSV per shape, named by its stem."""
import json

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.master.master_node import MasterNode  # noqa: E402


@pytest.fixture
def shape_dir(tmp_path):
    d = tmp_path / "lines"
    d.mkdir()
    for name, vx in (("line1.csv", 10.0), ("line2.csv", 6.0), ("center.csv", 8.0)):
        (d / name).write_text(
            "".join(f"{i}.0,{i}.0,0.0,0.0,0.0,{vx},0.0\n" for i in range(5))
        )
    (d / "notes.txt").write_text("not a line\n")
    return str(d)


def _params(shape_dir):
    return {
        "system_state": "IDLE",
        "system_frequency": 60,
        "line_dir": shape_dir,
    }


def test_get_line_shapes_lists_the_csv_stems(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            assert node.get_line_shapes()["shapes"] == ["center", "line1", "line2"]
        finally:
            node.destroy_node()


def test_get_line_shapes_ignores_non_csv_files(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            assert "notes" not in node.get_line_shapes()["shapes"]
        finally:
            node.destroy_node()


def test_get_line_shapes_on_a_missing_dir_is_empty(ros_ctx, tmp_path):
    with ros_ctx(_params(str(tmp_path / "nope"))):
        node = MasterNode()
        try:
            assert node.get_line_shapes()["shapes"] == []
        finally:
            node.destroy_node()


def test_get_line_shapes_has_no_active_spec_before_any_swap(ros_ctx, shape_dir):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            assert node.get_line_shapes()["active"] is None
        finally:
            node.destroy_node()


def test_set_line_speed_publishes_only_the_keys_it_was_given(
    ros_ctx, shape_dir, spin_helper
):
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
            ok, _ = node.set_line_speed({"shape": "line1", "v_mult": 1.5})
            assert ok
            assert spin_helper(exe, lambda: bool(seen), timeout=3.0)

            # No v_max: pathfinder_node supplies it from the mpc params.
            assert json.loads(seen[-1]) == {"shape": "line1", "v_mult": 1.5}
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

            assert node.get_line_shapes()["active"] == {
                "shape": "line1", "v_mult": 0.5,
            }
        finally:
            node.destroy_node()


@pytest.mark.parametrize("payload,needle", [
    ({}, "shape"),
    ({"shape": "nope"}, "nope"),
    ({"shape": "line1", "v_max": "fast"}, "v_max"),
    ({"shape": "line1", "v_mult": 0}, "v_mult"),
    ({"shape": "line1", "v_min": 9.0, "v_max": 4.0}, "v_min"),
])
def test_set_line_speed_rejects(ros_ctx, shape_dir, payload, needle):
    with ros_ctx(_params(shape_dir)):
        node = MasterNode()
        try:
            ok, reason = node.set_line_speed(payload)

            assert not ok
            assert needle in reason
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
