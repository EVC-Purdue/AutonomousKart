"""
Integration tests for MasterNode.

Covers:
 - STATES enum invariants
 - state transition pub/sub round-trip
 - manual_control publishes on `manual_commands`
 - odom_callback + cmd callbacks populate the snapshot exposed via get_odom
 - get_logs flushes the queue
 - bad initial state assertion
"""
import json
import math

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.master.master_node import MasterNode, STATES  # noqa: E402


def _default_params(state="IDLE"):
    return {"system_state": state, "system_frequency": 60}


def test_states_enum_has_all_expected_values():
    assert {s.value for s in STATES} == {"IDLE", "MANUAL", "AUTONOMOUS", "STOPPED"}


def test_master_starts_in_parameter_state(ros_ctx):
    with ros_ctx(_default_params("IDLE")) as rclpy:
        node = MasterNode()
        try:
            assert node.state == "IDLE"
        finally:
            node.destroy_node()


def test_master_rejects_bad_initial_state(ros_ctx):
    with ros_ctx(_default_params("GARBAGE")) as rclpy:
        with pytest.raises(AssertionError):
            MasterNode()


def test_update_state_publishes_and_updates(ros_ctx, spin_helper):
    with ros_ctx(_default_params("IDLE")) as rclpy:
        node = MasterNode()
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        try:
            node.update_state("AUTONOMOUS")
            assert spin_helper(exe, lambda: node.state == "AUTONOMOUS", timeout=2.0)
        finally:
            exe.remove_node(node)
            node.destroy_node()


def test_update_state_ignores_unknown_state(ros_ctx):
    with ros_ctx(_default_params("IDLE")) as rclpy:
        node = MasterNode()
        try:
            node.update_state("BOGUS")
            assert node.state == "IDLE"
        finally:
            node.destroy_node()


def test_manual_control_publishes_on_manual_commands(ros_ctx, spin_helper):
    from std_msgs.msg import Float32MultiArray

    with ros_ctx(_default_params("IDLE")) as rclpy:
        node = MasterNode()
        listener = rclpy.create_node("manual_listener")
        received = []
        listener.create_subscription(
            Float32MultiArray,
            "manual_commands",
            lambda m: received.append(list(m.data)),
            10,
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        try:
            # Give the discovery a tick before publishing.
            spin_helper(exe, lambda: False, timeout=0.3)
            node.manual_control(42.0, -7.5)
            assert spin_helper(exe, lambda: len(received) >= 1, timeout=2.0)
            assert received[0] == [42.0, -7.5]
        finally:
            exe.remove_node(listener)
            exe.remove_node(node)
            listener.destroy_node()
            node.destroy_node()


def test_odom_snapshot_and_cmd_callbacks(ros_ctx, spin_helper):
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32

    with ros_ctx(_default_params("AUTONOMOUS")) as rclpy:
        node = MasterNode()
        pub_node = rclpy.create_node("fake_pubs")
        odom_pub = pub_node.create_publisher(Odometry, "odom", 10)
        vel_pub = pub_node.create_publisher(Float32, "cmd_vel", 10)
        turn_pub = pub_node.create_publisher(Float32, "cmd_turn", 10)
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(pub_node)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery

            msg = Odometry()
            msg.pose.pose.position.x = 1.0
            msg.pose.pose.position.y = 2.0
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = 3.5
            odom_pub.publish(msg)
            vel_pub.publish(Float32(data=11.0))
            turn_pub.publish(Float32(data=-0.25))

            assert spin_helper(
                exe,
                lambda: node.odom_data["x"] == 1.0
                and node.cmd_data["motor"] == 11.0
                and node.cmd_data["steer"] == -0.25,
                timeout=2.0,
            )
            snap = node.get_odom()
            assert snap["x"] == 1.0 and snap["y"] == 2.0
            assert math.isclose(snap["yaw"], 0.0, abs_tol=1e-9)
            assert snap["speed"] == 3.5
            assert snap["motor"] == 11.0 and snap["steer"] == -0.25
            assert snap["state"] == "AUTONOMOUS"
        finally:
            exe.remove_node(pub_node)
            exe.remove_node(node)
            pub_node.destroy_node()
            node.destroy_node()


# ----------------------------------------------------- yaw calibration drive


def _yaw_cal_params(state="IDLE", **overrides):
    p = _default_params(state)
    p.update({
        "yaw_cal_speed": 10.0,
        "yaw_cal_duration_s": 0.3,
        "yaw_cal_accel_floor": 0.1,
        "yaw_cal_min_samples": 2,
    })
    p.update(overrides)
    return p


def test_start_yaw_calibration_rejects_when_imu_not_calibrated(ros_ctx):
    with ros_ctx(_yaw_cal_params("IDLE")) as rclpy:
        node = MasterNode()
        try:
            ok, reason = node.start_yaw_calibration()
            assert not ok
            assert "IMU" in reason
        finally:
            node.destroy_node()


def test_start_yaw_calibration_rejects_when_state_is_autonomous(ros_ctx):
    with ros_ctx(_yaw_cal_params("AUTONOMOUS")) as rclpy:
        node = MasterNode()
        try:
            node.imu_status_data = {"state": "CALIBRATED"}
            ok, reason = node.start_yaw_calibration()
            assert not ok
            assert "state must be IDLE or STOPPED" in reason
        finally:
            node.destroy_node()


def test_start_yaw_calibration_rejects_when_already_running(ros_ctx):
    with ros_ctx(_yaw_cal_params("IDLE", yaw_cal_duration_s=0.5)) as rclpy:
        node = MasterNode()
        try:
            node.imu_status_data = {"state": "CALIBRATED"}
            with node._lock:
                node.imu_data["ax"] = 1.0
                node.imu_data["ay"] = 0.0
                node.odom_data["yaw"] = 0.0

            ok, _ = node.start_yaw_calibration()
            assert ok

            ok2, reason = node.start_yaw_calibration()
            assert not ok2
            assert "already running" in reason

            node._yaw_cal_thread.join(timeout=2.0)
        finally:
            node.destroy_node()


def test_yaw_calibration_publishes_offset_and_ends_in_stopped(ros_ctx, spin_helper):
    """End-to-end: drive worker collects samples and publishes computed offset."""
    import threading
    import time as _time
    from std_msgs.msg import Float32

    with ros_ctx(_yaw_cal_params("IDLE", yaw_cal_duration_s=0.5, yaw_cal_min_samples=2)) as rclpy:
        node = MasterNode()
        listener = rclpy.create_node("yaw_listener")
        received = []
        listener.create_subscription(
            Float32, "imu/yaw_offset", lambda m: received.append(m.data), 1
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)  # discovery

            node.imu_status_data = {"state": "CALIBRATED"}
            with node._lock:
                node.imu_data["ax"] = 1.0   # forward accel along body-X in leveled frame
                node.imu_data["ay"] = 0.0
                node.odom_data["yaw"] = 0.4

            # Bump yaw partway through so the "GPS yaw unchanged" guard passes.
            def bumper():
                _time.sleep(0.2)
                with node._lock:
                    node.odom_data["yaw"] = 0.6
            threading.Thread(target=bumper, daemon=True).start()

            ok, _ = node.start_yaw_calibration()
            assert ok

            assert spin_helper(exe, lambda: len(received) >= 1, timeout=3.0)
            node._yaw_cal_thread.join(timeout=2.0)

            assert node.state == "STOPPED"
            # theta_imu = atan2(0, 1) = 0; psi_gps ≈ 0.5; offset ≈ 0.5
            assert received[0] == pytest.approx(0.5, abs=0.15)
        finally:
            exe.remove_node(listener)
            exe.remove_node(node)
            listener.destroy_node()
            node.destroy_node()


def test_yaw_calibration_aborts_when_below_accel_floor(ros_ctx):
    """All samples have |a_h| below the floor → abort with no publish, R untouched."""
    from std_msgs.msg import Float32

    with ros_ctx(_yaw_cal_params(
        "IDLE",
        yaw_cal_duration_s=0.2,
        yaw_cal_accel_floor=10.0,
        yaw_cal_min_samples=5,
    )) as rclpy:
        node = MasterNode()
        listener = rclpy.create_node("yaw_listener")
        received = []
        listener.create_subscription(
            Float32, "imu/yaw_offset", lambda m: received.append(m.data), 1
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(listener)
        try:
            node.imu_status_data = {"state": "CALIBRATED"}
            with node._lock:
                node.imu_data["ax"] = 0.05   # well below 10.0 floor
                node.imu_data["ay"] = 0.0
                node.odom_data["yaw"] = 0.0

            ok, _ = node.start_yaw_calibration()
            assert ok
            node._yaw_cal_thread.join(timeout=2.0)

            # Drain any pending publishes (there shouldn't be any).
            import time as _t
            t0 = _t.monotonic()
            while _t.monotonic() - t0 < 0.5 and not received:
                exe.spin_once(timeout_sec=0.05)

            assert received == []
            assert node.state == "STOPPED"
        finally:
            exe.remove_node(listener)
            exe.remove_node(node)
            listener.destroy_node()
            node.destroy_node()


def test_get_logs_flushes_after_read(ros_ctx):
    from std_msgs.msg import String

    with ros_ctx(_default_params("IDLE")) as rclpy:
        node = MasterNode()
        try:
            node.logs_callback(String(data=json.dumps({"hello": "world"})))
            assert node.get_logs() == [{"hello": "world"}]
            assert node.get_logs() == []
        finally:
            node.destroy_node()
