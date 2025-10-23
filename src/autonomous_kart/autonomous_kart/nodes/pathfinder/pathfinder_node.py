import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder


class PathfinderNode(Node):
    def __init__(self):
        super().__init__('PathfinderNode')
        self.logger = self.get_logger()
        self.angles = None

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        self.declare_parameter('system_frequency', 60)
        self.system_frequency = self.get_parameter('system_frequency').value

        self.declare_parameter('parking_lot', False)
        self.parking_lot = self.get_parameter('parking_lot').value

        if self.parking_lot:
            self.declare_parameter('speed', 5)
            self.speed = self.get_parameter('speed').value

            self.declare_parameter('steering', 0)
            self.steering = self.get_parameter('steering').value

            self.running = True

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscriber to opencv pathfinder for angles
        self.opencv_pathfinder_subscriber = self.create_subscription(
            Float32MultiArray,
            'track_angles',
            self.calculate_path_callback,
            5
        )

        # Publisher to motor
        self.motor_publisher = self.create_publisher(
            Float32,
            'cmd_vel',
            5
        )

        # # Publisher to steering
        self.steering_publisher = self.create_publisher(
            Float32,
            'cmd_turn',
            5
        )

        self.logger.info("Initialize Pathfinder Node")

    def calculate_path_callback(self, msg: Float32MultiArray):
        """
        Calculate commands for steering and motor from opencv_pathfinder efficiently
        Part of hot loop so must be efficient.
        If parking lot mode, drive for 5 seconds at speed with steering then stop.
        :param msg: Float32MultiArray of [left angle from center to base of track from image, right angle ...]
        :return: Publishes commands to motor & steering
        """
        self.cmd_count += 1
        self.angles = (msg.data[0], msg.data[1])
        if not self.parking_lot:
            if not self.running:
                return
            if self.cmd_count > 5 * self.system_frequency:  # 5 seconds
                self.running = False

            motor_speed, steering_angle = pathfinder(msg.data)
        else:
            motor_speed, steering_angle = self.speed, self.steering

        self.steering_publisher.publish(Float32(data=steering_angle))
        self.motor_publisher.publish(Float32(data=motor_speed))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
