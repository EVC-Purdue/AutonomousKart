"""
Integration test for LocalizationNode in sim mode.

Drives the node with constant cmd_drive ([throttle, steering]) and checks that:
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
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_params(tiny_racing_line)) as rclpy:
        node = LocalizationNode()
        driver = rclpy.create_node("loc_driver")
        drive_pub = driver.create_publisher(Float32MultiArray, "cmd_drive", 10)
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
                drive_pub.publish(Float32MultiArray(data=[100.0, 0.0]))
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


def _real_params():
    return {
        "simulation_mode": False,
        "system_frequency": 60,
        "wheelbase_m": 1.05,
        "v_max_mps": 15.0,
        "steer_max_deg": 25.0,
    }


def _gps_odom(x, y, yaw, speed, yaw_var=0.003, speed_var=0.0025, xy_var=0.01):
    """Build the GPS Odometry shape gps_node now publishes (with VTG)."""
    import math
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Quaternion

    m = Odometry()
    m.pose.pose.position.x = float(x)
    m.pose.pose.position.y = float(y)
    m.pose.pose.orientation = Quaternion(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
    )
    cov = [0.0] * 36
    cov[0] = xy_var
    cov[7] = xy_var
    cov[14] = xy_var
    cov[21] = 1e6
    cov[28] = 1e6
    cov[35] = yaw_var
    m.pose.covariance = cov
    m.twist.twist.linear.x = float(speed)
    twist_cov = [0.0] * 36
    twist_cov[0] = speed_var
    twist_cov[7] = 1e6
    twist_cov[14] = 1e6
    twist_cov[21] = 1e6
    twist_cov[28] = 1e6
    twist_cov[35] = 1e6
    m.twist.covariance = twist_cov
    return m


def test_localization_real_mode_seeds_from_single_vtg_fix(ros_ctx):
    """With VTG-style heading present, EKF initializes on a single GPS fix."""
    import math

    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            assert not node.ekf.initialized
            node.gps_callback(_gps_odom(x=10.0, y=20.0, yaw=math.pi / 4, speed=3.0))
            assert node.ekf.initialized, "single fix w/ VTG should seed EKF"
            px, py, yaw, v = node.ekf.x
            assert abs(px - 10.0) < 1e-6
            assert abs(py - 20.0) < 1e-6
            assert abs(yaw - math.pi / 4) < 1e-6, f"yaw={yaw}"
            assert abs(v - 3.0) < 1e-6
        finally:
            node.destroy_node()


def test_localization_real_mode_skips_seed_when_yaw_unknown(ros_ctx):
    """If GPS yaw cov is 1e6 (no VTG yet), do NOT init the EKF."""
    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            node.gps_callback(
                _gps_odom(x=1.0, y=2.0, yaw=0.0, speed=0.0,
                          yaw_var=1e6, speed_var=1e6)
            )
            assert not node.ekf.initialized
        finally:
            node.destroy_node()


def test_localization_real_mode_folds_vtg_updates(ros_ctx):
    """After init, subsequent fixes update xy + yaw + speed."""
    import math

    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            node.gps_callback(_gps_odom(x=0.0, y=0.0, yaw=0.0, speed=2.0))
            assert node.ekf.initialized
            # Step forward: report moving north now at 4 m/s.
            for _ in range(5):
                node.gps_callback(
                    _gps_odom(x=0.0, y=1.0, yaw=math.pi / 2, speed=4.0)
                )
            _, _, yaw, v = node.ekf.x
            # EKF pulls toward the new measurements (not all the way in 5 steps).
            assert math.pi / 4 < yaw < math.pi / 2 + 0.1, f"yaw={yaw}"
            assert v > 2.5, f"v={v}"

            # Speed-only fix (yaw dropped out): still folds in.
            node.gps_callback(
                _gps_odom(x=0.0, y=1.0, yaw=0.0, speed=7.0,
                          yaw_var=1e6, speed_var=0.01)
            )
            _, _, _, v_after = node.ekf.x
            assert v_after > v, f"speed-only update should pull v up: {v}->{v_after}"
        finally:
            node.destroy_node()
