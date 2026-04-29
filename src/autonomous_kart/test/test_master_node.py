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
