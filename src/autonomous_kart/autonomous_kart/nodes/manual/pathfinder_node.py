import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


class PathfinderNode(Node):
    def __init__(self):
        super().__init__(
            "pathfinder_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.logger = self.get_logger()
        self.angles = None

        self.steering = 0.0
        self.speed = 0.0

        self.expected_steering = 0.0
        self.expected_speed = 0.0

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now().nanoseconds

        self.declare_parameter("system_frequency", 60)
        self.system_frequency = self.get_parameter("system_frequency").value

        self.declare_parameter("simulation_mode", False)
        self.sim_mode = self.get_parameter("simulation_mode").value

        self.declare_parameter("max_speed", 40)
        self.max_speed = self.get_parameter("max_speed").value

        self.declare_parameter("acceleration", 0.5)
        self.acceleration = self.get_parameter("acceleration").value

        self.declare_parameter("max_steering", 75)
        self.max_steering = self.get_parameter("max_steering").value

        self.declare_parameter("steering_accel", 0.5)
        self.steering_accel = self.get_parameter("steering_accel").value

        assert self.acceleration > 0
        assert self.steering_accel > 0
        assert self.max_speed >= 0
        assert self.max_steering >= 0

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

    def update_params(self, speed: float = 0.0, steering: float = 0.0):
        """Relative changes to speed & steering"""
        self.set_params(self.speed + speed, self.steering + steering)

    def set_params(self, speed: float = 0.0, steering: float = 0.0):
        target_speed = min(self.max_speed, max(0.0, speed))
        target_steering = min(self.max_steering, max(-1 * self.max_steering, steering))

        tolerance = 0.1
        speed_ok = abs(target_speed - self.speed) > tolerance
        steering_ok = abs(target_steering - self.steering) > tolerance

        while speed_ok or steering_ok:
            self.param_helper(target_speed, target_steering)

            speed_ok = abs(target_speed - self.speed) > tolerance
            steering_ok = abs(target_steering - self.steering) > tolerance

    def param_helper(self, speed: float = 0.0, steering: float = 0.0):
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
        current_time = self.get_clock().now().nanoseconds
        elapsed = (current_time - self.last_log_time) / 1e9

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
