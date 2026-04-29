"""
Integration tests for PathfinderNode.

Construction exercises CSV loading + DynamicLineManager wiring. Beyond
that we verify:
 - AUTONOMOUS mode: fake odom + fake track_angles → publishes on
   cmd_vel and cmd_turn.
 - IDLE mode: same inputs → silent.
 - update_state accepts valid values and rejects garbage.
"""
import time

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.pathfinder.pathfinder_node import PathfinderNode  # noqa: E402


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
        "search_window": 80,
        "initial_sync_done": False,
        "max_resync_dist": 80.0,
        "max_closed_dist": 2.0,
    }


def _make_odom(x=0.0, y=0.0):
    from nav_msgs.msg import Odometry

    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_pathfinder_loads_racing_line(ros_ctx, tiny_racing_line):
    with ros_ctx(_params(tiny_racing_line)) as rclpy:
        node = PathfinderNode()
        try:
            assert len(node.racing_line) == 10
            assert node.racing_line[0][0] == 0.0
        finally:
            node.destroy_node()


def test_pathfinder_issues_commands_in_autonomous_mode(
    ros_ctx, tiny_racing_line, spin_helper
):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32, Float32MultiArray

    with ros_ctx(_params(tiny_racing_line, "AUTONOMOUS")) as rclpy:
        node = PathfinderNode()
        driver = rclpy.create_node("pf_driver")
        odom_pub = driver.create_publisher(Odometry, "odom", 10)
        angles_pub = driver.create_publisher(Float32MultiArray, "track_angles", 10)

        vel, turn = [], []
        driver.create_subscription(Float32, "cmd_vel", lambda m: vel.append(m.data), 10)
        driver.create_subscription(Float32, "cmd_turn", lambda m: turn.append(m.data), 10)

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery
            # Republish odom on every tick until the node flips pose_ready.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not node.pose_ready:
                odom_pub.publish(_make_odom(0.0, 0.0))
                exe.spin_once(timeout_sec=0.05)
            assert node.pose_ready

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and (not vel or not turn):
                angles_pub.publish(Float32MultiArray(data=[45.0, 45.0]))
                exe.spin_once(timeout_sec=0.05)

            assert len(vel) >= 1, "pathfinder never published cmd_vel"
            assert len(turn) >= 1, "pathfinder never published cmd_turn"
            # Tiny racing line is straight along +x, kart is at origin, yaw 0
            # → steering should be small.
            assert abs(turn[-1]) < 5.0
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()


def test_pathfinder_silent_in_idle_state(ros_ctx, tiny_racing_line, spin_helper):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32, Float32MultiArray

    with ros_ctx(_params(tiny_racing_line, "IDLE")) as rclpy:
        node = PathfinderNode()
        driver = rclpy.create_node("pf_driver")
        odom_pub = driver.create_publisher(Odometry, "odom", 10)
        angles_pub = driver.create_publisher(Float32MultiArray, "track_angles", 10)

        vel, turn = [], []
        driver.create_subscription(Float32, "cmd_vel", lambda m: vel.append(m.data), 10)
        driver.create_subscription(Float32, "cmd_turn", lambda m: turn.append(m.data), 10)

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not node.pose_ready:
                odom_pub.publish(_make_odom())
                exe.spin_once(timeout_sec=0.05)
            for _ in range(10):
                angles_pub.publish(Float32MultiArray(data=[45.0, 45.0]))
                exe.spin_once(timeout_sec=0.05)
            assert vel == []
            assert turn == []
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()


def test_pathfinder_state_update_accepts_valid_and_rejects_bad(
    ros_ctx, tiny_racing_line
):
    from std_msgs.msg import String

    with ros_ctx(_params(tiny_racing_line, "IDLE")) as rclpy:
        node = PathfinderNode()
        try:
            node.update_state(String(data="AUTONOMOUS"))
            assert node.state == "AUTONOMOUS"
            node.update_state(String(data="GARBAGE"))
            assert node.state == "AUTONOMOUS"
        finally:
            node.destroy_node()
