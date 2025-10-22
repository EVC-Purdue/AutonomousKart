import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, Float32
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher

from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder


class PathfinderNode(Node):
    def __init__(self):
        super().__init__('PathfinderNode')
        self.logger: RcutilsLogger = self.get_logger()
        self.angles: tuple[float, float] | None = None

        self.cmd_count: int = 0
        self.last_log_time: Time = self.get_clock().now()

        self.declare_parameter('system_frequency', 60)
        self.system_frequency: float = self.get_parameter('system_frequency').value

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscriber to opencv pathfinder for angles
        self.opencv_pathfinder_subscriber: Subscription = self.create_subscription(
            Float32MultiArray,
            'track_angles',
            self.calculate_path_callback,
            5
        )

        # Publisher to motor
        self.motor_publisher: Publisher = self.create_publisher(
            Float32,
            'cmd_vel',
            5
        )

        # # Publisher to steering
        self.steering_publisher: Publisher = self.create_publisher(
            Float32,
            'cmd_turn',
            5
        )

        self.logger.info("Initialize Pathfinder Node")

    def calculate_path_callback(self, msg: Float32MultiArray):
        """
        Calculate commands for steering and motor from opencv_pathfinder efficiently
        Part of hot loop so must be efficient
        :param msg: Float32MultiArray of [left angle from center to base of track from image, right angle ...]
        :return: Publishes commands to motor & steering
        """
        self.cmd_count += 1
        self.angles = (msg.data[0], msg.data[1])

        motor_speed, steering_angle = pathfinder(msg.data)

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
