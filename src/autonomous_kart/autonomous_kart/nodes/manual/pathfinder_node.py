from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


class PathfinderNode(Node):
    def __init__(self):
        super().__init__("PathfinderNode")
        self.logger = self.get_logger()
        self.angles = None

        self.steering = 0.0
        self.speed = 0.0

        self.expected_steering = 0.0
        self.expected_speed = 0.0

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        self.declare_parameter("system_frequency", 60)
        self.system_frequency = self.get_parameter("system_frequency").value

        self.declare_parameter("simulation_mode", False)
        self.sim_mode = self.get_parameter("simulation_mode").value

        self.declare_parameter("max_speed", 50)
        self.max_speed = self.get_parameter("max_speed").value

        self.declare_parameter("acceleration", 2)
        self.acceleration = self.get_parameter("acceleration").value

        self.declare_parameter("max_steering", 75)
        self.max_steering = self.get_parameter("max_steering").value

        self.declare_parameter("steering_accel", 5)
        self.steering_accel = self.get_parameter("steering_accel").value

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)
        # Run publish each iteration of hot loop
        self.create_timer(1.0 / self.system_frequency, self.publish_commands)

        # Publisher to motor
        self.motor_publisher = self.create_publisher(Float32, "cmd_vel", 5)

        # Publisher to steering
        self.steering_publisher = self.create_publisher(Float32, "cmd_turn", 5)

        self.metrics_publisher = self.create_publisher(
            Float32MultiArray, "pathfinder_params", 5
        )

        self.logger.info("Initialize Pathfinder Node")

    def update_params(self, speed: Optional[float], steering: Optional[float]):
        """ Updates speed & steering params from given values """
        self.cmd_count += 1

        if abs(speed) > self.max_speed:
            speed = self.max_speed if speed > 0 else -1 * self.max_speed
        if abs(steering) > self.max_steering:
            speed = self.max_steering if steering > 0 else -1 * self.max_steering

        if speed:
            self.expected_speed = speed
        if steering:
            self.expected_steering = steering

        d_speed = self.speed - self.expected_speed
        d_steer = self.steering - self.expected_steering

        # Update speed by difference or acceleration, whichever abs value is smaller
        if d_speed != 0:
            self.speed += (
                d_speed if abs(d_speed) < self.acceleration else self.acceleration
            )
        if d_steer != 0:
            self.steering += (
                d_steer
                if abs(d_steer) < self.steering_accel
                else (d_steer / abs(d_steer)) * self.steering_accel
            )
        self.publish_commands()

    def publish_commands(self):
        """ Publishes commands from params to steering & motor """
        self.steering_publisher.publish(Float32(data=self.steering))
        self.motor_publisher.publish(Float32(data=self.speed))

        time = self.get_clock().now()

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
        elapsed = (current_time - self.last_log_time).nanoseconds / 1e9

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
        node.get_logger().error("Unhandled exception", exc_info=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
