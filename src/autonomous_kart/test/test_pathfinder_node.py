"""
Integration tests for PathfinderNode.

Construction exercises CSV loading + planner wiring. Beyond that we verify:
 - AUTONOMOUS mode: fake odom + fake track_angles → publishes on
   cmd_drive ([throttle, steering]).
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
        "pure_pursuit.min_reg_speed_pct": 0.20,
        "pure_pursuit.approach_dist_m": 1.0,
        "pure_pursuit.min_approach_speed_pct": 0.05,
        "pure_pursuit.search_window": 80,
        "pure_pursuit.initial_sync_done": False,
        "pure_pursuit.max_resync_dist": 80.0,
        "pure_pursuit.max_closed_dist": 2.0,
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
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_params(tiny_racing_line, "AUTONOMOUS")) as rclpy:
        node = PathfinderNode()
        driver = rclpy.create_node("pf_driver")
        odom_pub = driver.create_publisher(Odometry, "odom", 10)
        angles_pub = driver.create_publisher(Float32MultiArray, "track_angles", 10)

        drive = []
        driver.create_subscription(
            Float32MultiArray, "cmd_drive", lambda m: drive.append(list(m.data)), 10
        )

        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery
            # Republish odom until the node flips pose_ready.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not node.pose_ready:
                odom_pub.publish(_make_odom(0.0, 0.0))
                exe.spin_once(timeout_sec=0.05)
            assert node.pose_ready

            # Timer-driven now: publishing track_angles just refreshes the
            # cached input; cmd_drive is emitted from the autonomous tick.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not drive:
                angles_pub.publish(Float32MultiArray(data=[45.0, 45.0]))
                exe.spin_once(timeout_sec=0.05)

            assert len(drive) >= 1, "pathfinder never published cmd_drive"
            # Tiny racing line is straight along +x, kart is at origin, yaw 0
            # → steering should be small.
            assert abs(drive[-1][1]) < 5.0
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()


def test_pathfinder_silent_in_idle_state(ros_ctx, tiny_racing_line, spin_helper):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_params(tiny_racing_line, "IDLE")) as rclpy:
        node = PathfinderNode()
        driver = rclpy.create_node("pf_driver")
        odom_pub = driver.create_publisher(Odometry, "odom", 10)
        angles_pub = driver.create_publisher(Float32MultiArray, "track_angles", 10)

        drive = []
        driver.create_subscription(
            Float32MultiArray, "cmd_drive", lambda m: drive.append(list(m.data)), 10
        )

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
            assert drive == []
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
