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
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=0.0, stamp_sec=0))
            node.gps_callback(_gps_odom(x=10.0, y=20.0, yaw=math.pi / 4, speed=3.0))
            assert node.ekf.initialized, "single fix w/ VTG should seed EKF"
            px, py, yaw, v = node.ekf.x
            assert abs(px - 10.0) < 1e-6
            assert abs(py - 20.0) < 1e-6
            assert abs(yaw - math.pi / 4) < 1e-6, f"yaw={yaw}"
            assert abs(v - 3.0) < 1e-6
        finally:
            node.destroy_node()


def test_localization_real_mode_seeds_xy_only_when_yaw_unknown(ros_ctx):
    """If GPS yaw cov is 1e6 (no VTG yet), still seed xy with defaults for
    yaw/speed at large variance so subsequent updates can pull them in."""
    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=0.0, stamp_sec=0))
            node.gps_callback(
                _gps_odom(x=1.0, y=2.0, yaw=0.0, speed=0.0,
                          yaw_var=1e6, speed_var=1e6)
            )
            assert node.ekf.initialized
            px, py, yaw0, v0 = node.ekf.x
            assert abs(px - 1.0) < 1e-6
            assert abs(py - 2.0) < 1e-6
            assert yaw0 == 0.0
            assert v0 == 0.0
            # Yaw/speed variances should be wide since they weren't measured.
            assert node.ekf.P[2, 2] > 1.0
            assert node.ekf.P[3, 3] >= 1.0
        finally:
            node.destroy_node()


def test_localization_real_mode_folds_vtg_updates(ros_ctx):
    """After init, subsequent fixes update xy + yaw + speed."""
    import math

    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=0.0, stamp_sec=0))
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


def test_localization_real_mode_wheel_speed_pulls_v_and_shrinks_pv(ros_ctx):
    """An /e_comms/kart_speed_m_per_s message after init folds into v."""
    from std_msgs.msg import Float32

    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            # Seed with v=0; then feed a wheel-speed of 5 m/s.
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=0.0, stamp_sec=0))
            node.gps_callback(_gps_odom(x=0.0, y=0.0, yaw=0.0, speed=0.0))
            assert node.ekf.initialized
            pv_before = node.ekf.P[3, 3]
            v_before = node.ekf.x[3]

            node._wheel_speed_cb(Float32(data=5.0))

            assert node.ekf.x[3] > v_before, "v should move toward 5 m/s"
            assert node.ekf.P[3, 3] < pv_before, "P[v,v] should shrink"
        finally:
            node.destroy_node()


def test_localization_real_mode_wheel_speed_ignored_before_init(ros_ctx):
    """Wheel-speed messages before GPS init are no-ops (no crash, no state)."""
    from std_msgs.msg import Float32

    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            assert not node.ekf.initialized
            node._wheel_speed_cb(Float32(data=5.0))  # must not raise
            assert not node.ekf.initialized
            assert node.ekf.x[3] == 0.0
        finally:
            node.destroy_node()


def _imu_msg(omega_z, accel_x, stamp_sec, stamp_nanosec=0,
             omega_var=0.001, accel_var=0.01):
    from sensor_msgs.msg import Imu

    m = Imu()
    m.header.stamp.sec = int(stamp_sec)
    m.header.stamp.nanosec = int(stamp_nanosec)
    m.angular_velocity.z = float(omega_z)
    m.linear_acceleration.x = float(accel_x)
    cov_g = [0.0] * 9
    cov_g[8] = float(omega_var)
    m.angular_velocity_covariance = cov_g
    cov_a = [0.0] * 9
    cov_a[0] = float(accel_var)
    m.linear_acceleration_covariance = cov_a
    return m


def test_localization_real_mode_imu_first_msg_only_sets_stamp(ros_ctx):
    """First /imu msg seeds _last_imu_stamp; no predict, no /odom."""
    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            # No GPS yet, no init. First IMU just sets the stamp.
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=2.0, stamp_sec=1))
            assert node._imu_seen is True
            assert node._last_imu_stamp_s == pytest.approx(1.0)
            # No predict happened -> v still 0 (filter not initialized either).
            assert node.ekf.x[3] == 0.0
            assert not node.ekf.initialized
        finally:
            node.destroy_node()


def test_localization_real_mode_imu_drives_predict(ros_ctx, spin_helper):
    """Second /imu msg integrates accel_x over the stamp delta."""
    from nav_msgs.msg import Odometry

    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        driver = rclpy.create_node("odom_sink")
        received = []
        driver.create_subscription(
            Odometry, "odom", lambda m: received.append(m), 10
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.2)  # discovery

            # Seed IMU first so the GPS init gate opens.
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=0.0, stamp_sec=1))
            node.gps_callback(_gps_odom(x=0.0, y=0.0, yaw=0.0, speed=0.0))
            assert node.ekf.initialized
            # Second IMU 0.1 s later with a_x = 1.0 -> v gains ~0.1 m/s.
            node._imu_cb(
                _imu_msg(omega_z=0.0, accel_x=1.0,
                         stamp_sec=1, stamp_nanosec=100_000_000)
            )
            spin_helper(exe, lambda: len(received) >= 1, timeout=0.5)

            assert node.ekf.x[3] > 0.05, f"v should grow under a_x=1 for 0.1s: {node.ekf.x[3]}"
            assert len(received) >= 1, "predict should publish /odom"
        finally:
            exe.remove_node(driver)
            exe.remove_node(node)
            driver.destroy_node()
            node.destroy_node()


def test_localization_real_mode_init_gate_waits_for_imu(ros_ctx):
    """A GPS fix arriving before any /imu does NOT initialize the EKF."""
    with ros_ctx(_real_params()) as rclpy:
        node = LocalizationNode()
        try:
            node.gps_callback(_gps_odom(x=1.0, y=2.0, yaw=0.0, speed=0.0))
            assert not node.ekf.initialized, "init should wait for IMU"
            # Seed IMU stamp, then re-send the GPS fix.
            node._imu_cb(_imu_msg(omega_z=0.0, accel_x=0.0, stamp_sec=1))
            node.gps_callback(_gps_odom(x=1.0, y=2.0, yaw=0.0, speed=0.0))
            assert node.ekf.initialized
        finally:
            node.destroy_node()
