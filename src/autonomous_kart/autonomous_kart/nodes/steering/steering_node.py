import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')

        self.declare_parameter('simulation_mode', False)
        self.sim_mode = self.get_parameter('simulation_mode').value

        # Subscribe to speed commands
        self.cmd_turn_sub = self.create_subscription(
            Twist,
            'cmd_turn',
            self.cmd_turn_callback,
            10
        )

        # Publisher for steering angular velocity
        self.turn_pub = self.create_publisher(
            Twist,
            'turn_angle',
            10
        )

        self.current_angle = Twist()

        self.get_logger().info(f'Steering Node started - Mode: {"SIM" if self.sim_mode else "REAL"}')

    def cmd_vel_callback(self, msg: Twist):
        """Receive speed commands and publish current speed"""

        self.current_angle = msg

        self.get_logger().info(
            f'Speed: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )

        if self.sim_mode:
            self.control_sim_steering(msg)
        else:
            self.control_real_steering(msg)

        self.speed_pub.publish(self.current_angle)

    def control_sim_steering(self, cmd: Twist):
        """Simulation mode - just log the command for now"""
        self.get_logger().debug(f'SIM: Steering set to commanded angle: L={cmd.linear.x} A={cmd.angular.z}')

    def control_real_steering(self, cmd: Twist):
        """Real mode - control actual hardware here"""
        self.get_logger().debug(f'REAL: Steering set to commanded angle: L={cmd.linear.x} A={cmd.angular.z}')


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