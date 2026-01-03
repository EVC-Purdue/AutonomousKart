import traceback
from typing import Tuple, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String
from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder

from autonomous_kart.nodes.master.master_node import STATES

class PathfinderNode(Node):
    def __init__(self):
        super().__init__(
            "PathfinderNode",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.logger = self.get_logger()
        self.angles = None

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        self.system_frequency = self.get_parameter("system_frequency").value
        self.sim_mode = self.get_parameter("simulation_mode").value

        # Manual params
        self.steering = 0.0
        self.speed = 0.0

        self.expected_steering = 0.0
        self.expected_speed = 0.0

        self.max_speed = self.get_parameter("max_speed").value
        self.acceleration = self.get_parameter("acceleration").value
        self.max_steering = self.get_parameter("max_steering").value
        self.steering_accel = self.get_parameter("steering_accel").value

        self.metrics_publisher = self.create_publisher(
            Float32MultiArray, "pathfinder_params", 5
        )

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        self.line_path = self.get_parameter("line_path").value
        self.racing_line: List[Tuple[float, float, float]] = []
        with open(self.line_path, 'r') as f:
            p1 = [(float(x) for x in f.readline().split(','))]
            self.racing_line.append(p1)

        self.state = self.get_parameter("system_state").value
        self.state_subscriber = self.create_subscription(String, "system_state", self.update_state, 10)

        # Subscriber to opencv pathfinder for angles
        self.opencv_pathfinder_subscriber = self.create_subscription(
            Float32MultiArray, "track_angles", self.autonomous_control_loop, 5
        )

        self.manual_subscriber = self.create_subscription(
            Float32MultiArray,
            "manual_commands",
            self.manual_loop,
            5
        )

        # Publisher to motor
        self.motor_publisher = self.create_publisher(Float32, "cmd_vel", 5)

        # # Publisher to steering
        self.steering_publisher = self.create_publisher(Float32, "cmd_turn", 5)

        self.logger.info("Initialize Pathfinder Node")

    def update_state(self, msg: String):
        if msg.data not in [s.value for s in STATES]:
            self.logger.error(f"State {msg.data} not recognized")
            return

        self.state = msg.data

    def autonomous_control_loop(self, msg: Float32MultiArray):
        """
        Calculate commands for steering and motor from opencv_pathfinder efficiently
        Part of hot loop so must be efficient
        :param msg: Float32MultiArray of [left angle from center to base of track from image, right angle ...]
        :return: Publishes commands to motor & steering
        """
        self.cmd_count += 1
        if self.state == STATES.AUTONOMOUS.value:
            self.angles = (msg.data[0], msg.data[1])

            motor_speed, steering_angle = pathfinder(msg.data)

            self.steering_publisher.publish(Float32(data=steering_angle))
            self.motor_publisher.publish(Float32(data=motor_speed))

    def manual_loop(self, msg: Float32MultiArray):
        self.cmd_count += 1
        if self.state == STATES.MANUAL:
            if len(msg.data) < 2:
                self.logger.error(f"Message length of manual loop is {len(msg.data)}: {msg.data}")
            motor_speed, steering = msg.data[0], msg.data[1]
            self.set_manual_speeds(motor_speed, steering)

    def set_manual_speeds(self, speed: float = 0.0, steering: float = 0.0):
        target_speed = min(self.max_speed, max(0.0, speed))
        target_steering = min(self.max_steering, max(-1 * self.max_steering, steering))

        tolerance = 0.1
        speed_ok = abs(target_speed - self.speed) > tolerance
        steering_ok = abs(target_steering - self.steering) > tolerance

        while speed_ok or steering_ok:
            self.manual_speed_helper(target_speed, target_steering)

            speed_ok = abs(target_speed - self.speed) > tolerance
            steering_ok = abs(target_steering - self.steering) > tolerance

    def manual_speed_helper(self, speed: float = 0.0, steering: float = 0.0):
        """Updates speed & steering params from given values, speed & steering are absolute"""
        self.cmd_count += 1

        if speed < 0:
            speed = 0.0

        if speed > self.max_speed:
            speed = self.max_speed
        if abs(steering) > self.max_steering:
            steering = self.max_steering if steering > 0 else -1 * self.max_steering

        self.expected_speed = speed
        self.expected_steering = steering

        d_speed = self.expected_speed - self.speed
        d_steer = self.expected_steering - self.steering

        # Update speed by difference or acceleration, whichever abs value is smaller
        if d_speed != 0:
            self.speed += (
                d_speed
                if abs(d_speed) < self.acceleration
                else (d_speed / abs(d_speed)) * self.acceleration
            )
        if d_steer != 0:
            self.steering += (
                d_steer
                if abs(d_steer) < self.steering_accel
                else (d_steer / abs(d_steer)) * self.steering_accel
            )
        self.publish_commands()

    def publish_commands(self):
        """Publishes commands from params to steering & motor"""
        self.steering_publisher.publish(Float32(data=self.steering))
        self.motor_publisher.publish(Float32(data=self.speed))

        time = self.get_clock().now().nanoseconds

        self.metrics_publisher.publish(
            Float32MultiArray(
                data=[
                    self.speed,
                    self.expected_speed,
                    self.acceleration,
                    self.max_speed,
                    self.steering,
                    self.expected_steering,
                    self.steering_accel,
                    self.max_steering,
                    time - self.last_log_time,
                ]
            )
        )
        self.last_log_time = time

    def log_command_rate(self):
        """Log average commands per second every 5 seconds"""
        current_time = self.get_clock().now()
        elapsed = (current_time.nanoseconds - self.last_log_time) / 1e9

        if elapsed > 0:
            avg_rate = self.cmd_count / elapsed
            self.get_logger().info(
                f"Average command rate: {avg_rate:.2f} commands/sec "
                f"(Total: {self.cmd_count} in {elapsed:.1f}s)"
            )

        # Reset counters
        self.cmd_count = 0
        self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    node = PathfinderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
