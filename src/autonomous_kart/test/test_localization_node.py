"""
Integration test for LocalizationNode in sim mode.

Drives the node with constant cmd_vel/cmd_turn and checks that:
 - /odom is published
 - the kart moves forward when throttled
 - the initial pose is auto-spawned from the racing-line CSV
"""
import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.localization.localization_node import LocalizationNode  # noqa: E402


def _params(line_path):
    return {
        "simulation_mode": True,
        "system_frequency": 60,
        "wheelbase_m": 1.05,
        "v_max_mps": 15.0,
        "steer_max_deg": 25.0,
        "line_path": line_path,
    }


def test_localization_auto_spawns_from_racing_line(ros_ctx, tiny_racing_line):
    with ros_ctx(_params(tiny_racing_line)) as rclpy:
        node = LocalizationNode()
        try:
            assert node.model.x == 0.0
            assert node.model.y == 0.0
        finally:
            node.destroy_node()


def test_localization_publishes_odom_and_moves_kart(
    ros_ctx, tiny_racing_line, spin_helper
):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32

    with ros_ctx(_params(tiny_racing_line)) as rclpy:
        node = LocalizationNode()
        driver = rclpy.create_node("loc_driver")
        vel_pub = driver.create_publisher(Float32, "cmd_vel", 10)
        turn_pub = driver.create_publisher(Float32, "cmd_turn", 10)
        received = []
        driver.create_subscription(Odometry, "odom", lambda m: received.append(m), 10)

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery
            # Hold full throttle for ~1s of wall-clock time; the node's
            # own timer advances the bicycle model.
            import time
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                vel_pub.publish(Float32(data=100.0))
                turn_pub.publish(Float32(data=0.0))
                exe.spin_once(timeout_sec=0.05)

            assert len(received) >= 5, "expected several /odom messages"
            first, last = received[0], received[-1]
            assert (
                last.pose.pose.position.x > first.pose.pose.position.x
            ), "kart should have moved forward under throttle"
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()
