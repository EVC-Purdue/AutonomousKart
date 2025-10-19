import random
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Float32, Float32MultiArray

from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder


class PathfinderNode(Node):
    def __init__(self):
        super().__init__('PathfinderNode')
        self.logger = self.get_logger()
        self.angles = None

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscriber to opencv pathfinder for angles
        self.opencv_pathfinder_subscriber = self.create_subscription(
            Float32MultiArray,
            'track_angles',
            self.calculate_path_callback,
            1
        )

        # Publisher to motor
        self.motor_publisher = self.create_publisher(
            Float32,
            'cmd_vel',
            1
        )

        # # Publisher to steering
        self.steering_publisher = self.create_publisher(
            Float32,
            'cmd_turn',
            1
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

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        time.sleep(0.1)
        node.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
