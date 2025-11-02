import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from message_filters import Subscriber, ApproximateTimeSynchronizer

import autonomous_kart.nodes.e_comms.e_comms as e_comms


class ECommsNode(Node):
    def __init__(self):
        super().__init__("ECommsNode")
        self.logger = self.get_logger()
        
        # Inputs
        self.motor_percent = None
        self.steering_angle = None

        # Ouputs
        self.spi_buffer = None

        # Logging
        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        # self.declare_parameter('system_frequency', 60)
        # self.system_frequency = self.get_parameter('system_frequency').value

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscribe to pathfinder node for throttle and steering commands
        self.motor_sub = Subscriber(self, Float32, 'cmd_vel')
        self.steering_sub = Subscriber(self, Float32, 'cmd_turn')
        self.ts = ApproximateTimeSynchronizer(
            [self.motor_sub, self.steering_sub],
            queue_size=3,
            slop=0.1  # Time tolerance in seconds
        )
        self.ts.registerCallback(self.cmd_callback)

        # Init finished
        self.logger.info("Initialize EComms Node")

    def cmd_callback(self, motor_msg: Float32, steering_msg: Float32):
        """
        Send the motor and steering command to the Electrical Stack via SPI.
        Part of hot loop so must be efficient
        
        :param motor_msg: Float32 message for motor command (percent throttle)
        :param steering_msg: Float32 message for steering command (steering angle, -90 to 90)
        :return: None
        """
        self.cmd_count += 1

        self.motor_percent = motor_msg.data
        self.steering_angle = steering_msg.data

        self.spi_buffer = e_comms.pack_to_buffer(self.motor_percent, self.steering_angle)
        # TODO: send via SPI

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
    node = ECommsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
