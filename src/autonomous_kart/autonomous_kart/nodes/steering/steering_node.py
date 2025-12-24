import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SteeringNode(Node):
    def __init__(self):
        super().__init__(
            "steering_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.sim_mode = self.get_parameter("simulation_mode").value
        self.max_steering = self.get_parameter("max_steering").value
        self.min_steering = self.get_parameter("min_steering").value
        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscribe to turn commands
        self.cmd_turn_sub = self.create_subscription(
            Float32, "cmd_turn", self.cmd_turn_callback, 5
        )

        # Publisher for steering angular velocity
        self.turn_pub = self.create_publisher(Float32, "turn_angle", 5)

        self.current_angle = Float32()
        self.logger = self.get_logger()

        self.logger.info(
            f"Steering Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

    def cmd_turn_callback(self, msg: Float32):
        """Receive turn commands and publish current turn"""
        self.cmd_count += 1
        if msg.data > self.max_steering or msg.data < self.min_steering:
            self.logger.error(f"Bad steering command value: {msg.data}, should be in range: [{self.min_steering}, {self.max_steering}]")
            return

        self.current_angle = msg.data

        # self.logger.info(
        #     f'Turn: angle={msg.data:.2f}'
        # )

        if self.sim_mode:
            self.control_sim_steering(msg)
        else:
            self.control_real_steering(msg)

        self.turn_pub.publish(Float32(data=self.current_angle))

    def control_sim_steering(self, cmd: Float32):
        """Simulation mode - just log the command for now"""
        self.logger.debug(f"SIM: Steering set to commanded angle={cmd.data:.2f}")

    def control_real_steering(self, cmd: Float32):
        """Real mode - control actual hardware here"""
        self.logger.debug(f"REAL: Steering set to commanded angle={cmd.data:.2f}")

    def log_command_rate(self):
        """Log average commands per second every 5 seconds"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_log_time).nanoseconds / 1e9

        if elapsed > 0:
            avg_rate = self.cmd_count / elapsed
            self.logger.info(
                f"Average command rate: {avg_rate:.2f} commands/sec "
                f"(Total: {self.cmd_count} in {elapsed:.1f}s)"
            )

        # Reset counters
        self.cmd_count = 0
        self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    node = SteeringNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.logger.error(traceback.format_exc())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
