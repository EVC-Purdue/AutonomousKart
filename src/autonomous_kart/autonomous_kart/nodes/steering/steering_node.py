import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')

        self.declare_parameter('simulation_mode', False)
        self.sim_mode = self.get_parameter('simulation_mode').value

        # Subscribe to speed commands
        self.cmd_turn_sub = self.create_subscription(
            Float32,
            'cmd_turn',
            self.cmd_turn_callback,
            3
        )

        # Publisher for steering angular velocity
        self.turn_pub = self.create_publisher(
            Float32,
            'turn_angle',
            3
        )

        self.current_angle = Float32()

        self.get_logger().info(f'Steering Node started - Mode: {"SIM" if self.sim_mode else "REAL"}')

    def cmd_turn_callback(self, msg: Float32):
        """Receive speed commands and publish current speed"""
        self.current_angle = msg.data

        self.get_logger().info(
            f'Speed: angle={msg.data:.2f}'
        )

        if self.sim_mode:
            self.control_sim_steering(msg)
        else:
            self.control_real_steering(msg)

        angle_cmd = Float32()
        angle_cmd.data = self.current_angle

        self.turn_pub.publish(angle_cmd)

    def control_sim_steering(self, cmd: Float32):
        """Simulation mode - just log the command for now"""
        self.get_logger().debug(f'SIM: Steering set to commanded angle={cmd.data:.2f}')

    def control_real_steering(self, cmd: Twist):
        """Real mode - control actual hardware here"""
        self.get_logger().debug(f'REAL: Steering set to commanded angle={cmd.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()