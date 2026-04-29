"""
Integration tests for the motor and steering driver nodes in sim mode.

These are thin sub/pub wrappers so the contract is:
  - receive on cmd_vel/cmd_turn → re-publish on motor_speed/turn_angle
  - respect steering range limits (reject out-of-range, don't crash)
"""
import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.motor.motor_node import MotorNode  # noqa: E402
from autonomous_kart.nodes.steering.steering_node import SteeringNode  # noqa: E402


def test_motor_republishes_cmd_vel_on_motor_speed(ros_ctx, spin_helper):
    from std_msgs.msg import Float32

    with ros_ctx({"simulation_mode": True}) as rclpy:
        motor = MotorNode()
        driver = rclpy.create_node("driver")
        pub = driver.create_publisher(Float32, "cmd_vel", 10)
        received = []
        driver.create_subscription(
            Float32, "motor_speed", lambda m: received.append(m.data), 10
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(motor)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)
            pub.publish(Float32(data=7.5))
            assert spin_helper(exe, lambda: len(received) >= 1, timeout=2.0)
            assert motor.cmd_count >= 1
        finally:
            exe.remove_node(driver)
            exe.remove_node(motor)
            driver.destroy_node()
            motor.destroy_node()


def test_steering_passes_valid_command_through(ros_ctx, spin_helper):
    from std_msgs.msg import Float32

    params = {"simulation_mode": True, "max_steering": 100, "min_steering": -100}
    with ros_ctx(params) as rclpy:
        steering = SteeringNode()
        driver = rclpy.create_node("driver")
        pub = driver.create_publisher(Float32, "cmd_turn", 10)
        received = []
        driver.create_subscription(
            Float32, "turn_angle", lambda m: received.append(m.data), 10
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(steering)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)
            pub.publish(Float32(data=25.0))
            assert spin_helper(exe, lambda: len(received) >= 1, timeout=2.0)
            assert received[-1] == 25.0
        finally:
            exe.remove_node(driver)
            exe.remove_node(steering)
            driver.destroy_node()
            steering.destroy_node()


def test_steering_rejects_out_of_range_command(ros_ctx, spin_helper):
    from std_msgs.msg import Float32

    params = {"simulation_mode": True, "max_steering": 10, "min_steering": -10}
    with ros_ctx(params) as rclpy:
        steering = SteeringNode()
        driver = rclpy.create_node("driver")
        pub = driver.create_publisher(Float32, "cmd_turn", 10)
        received = []
        driver.create_subscription(
            Float32, "turn_angle", lambda m: received.append(m.data), 10
        )
        exe = rclpy.executors.SingleThreadedExecutor()
        exe.add_node(steering)
        exe.add_node(driver)
        try:
            spin_helper(exe, lambda: False, timeout=0.3)
            pub.publish(Float32(data=99.0))
            spin_helper(exe, lambda: False, timeout=0.5)
            assert received == []
        finally:
            exe.remove_node(driver)
            exe.remove_node(steering)
            driver.destroy_node()
            steering.destroy_node()
