"""ROS wiring for opponents: kart_detector_node startup and pathfinder_node's /opponents input."""
import time

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.opencv_pathfinder.kart_detector_node import KartDetectorNode  # noqa: E402
from autonomous_kart.nodes.pathfinder.opponents import Opponent, encode_opponents  # noqa: E402
from autonomous_kart.nodes.pathfinder.pathfinder_node import PathfinderNode  # noqa: E402

from test_pathfinder_node import _params  # noqa: E402


def test_detector_node_without_model_idles_instead_of_crashing(ros_ctx, tiny_racing_line):
    with ros_ctx({"model_path": "/nonexistent/kart.onnx", "line_path": tiny_racing_line,
                  "simulation_mode": True}):
        node = KartDetectorNode()
        try:
            assert node.perception is None
            assert node.opp_pub is not None  # still up, just publishing nothing
        finally:
            node.destroy_node()


def test_pathfinder_takes_fresh_opponents_and_drops_stale_ones(ros_ctx, tiny_racing_line, spin_helper):
    from std_msgs.msg import Float64MultiArray

    params = dict(_params(tiny_racing_line, "IDLE"), opponents_timeout_s=0.3)
    with ros_ctx(params) as rclpy:
        node = PathfinderNode()
        driver = rclpy.create_node("opp_driver")
        pub = driver.create_publisher(Float64MultiArray, "opponents", 5)
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery
            opp = Opponent(4, 5.0, 0.5, 3.0, 0.0, 1.0, 0.4, 0.2)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not node._opponents:
                pub.publish(Float64MultiArray(data=encode_opponents([opp])))
                exe.spin_once(timeout_sec=0.05)
            now = node.get_clock().now().nanoseconds
            assert node._fresh_opponents(now) == (opp,)
            assert node._fresh_opponents(now + int(1e9)) == ()  # stale -> planner sees none
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()
