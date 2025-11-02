import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.declare_parameter('simulation_mode', False)
        self.sim_mode = self.get_parameter('simulation_mode').value

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscribe to speed commands
        self.cmd_vel_sub = self.create_subscription(
            Float32,
            'cmd_vel',
            self.cmd_vel_callback,
            5
        )

        # Publisher for motor speed
        self.speed_pub = self.create_publisher(
            Float32,
            'motor_speed',
            5
        )

        self.current_speed = Float32()

        self.get_logger().info(f'Motor Node started - Mode: {"SIM" if self.sim_mode else "REAL"}')

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

    def cmd_vel_callback(self, msg):
        """Receive speed commands and publish current speed"""
        self.cmd_count += 1
        self.current_speed = msg

        # self.get_logger().info(
        #     f'Speed: {msg.data:.2f}'
        # )

        if self.sim_mode:
            self.control_sim_motors(msg)
        else:
            self.control_real_motors(msg)

        self.speed_pub.publish(self.current_speed)

    def control_sim_motors(self, cmd):
        """Simulation mode - just log the command for now"""
        self.get_logger().debug(f'SIM: Motors set to commanded speed: {cmd.data:.2f}')

    def control_real_motors(self, cmd):
        """Real mode - control actual hardware here"""
        self.get_logger().debug(f'REAL: Motors set to commanded speed: {cmd.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error('Unhandled exception', exc_info=True)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass # Context already shutdown, ignore


if __name__ == '__main__':
    main()