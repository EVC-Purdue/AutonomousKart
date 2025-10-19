import random
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray, Float32, Float32MultiArray


class PathfinderNode(Node):
    def __init__(self):
        super().__init__('PathfinderNode')
        self.logger = self.get_logger()
        self.angles = None

        # Subscriber to opencv pathfinder for angles
        self.opencv_pathfinder_subscriber = self.create_subscription(
            Float64MultiArray,
            'track_angles',
            self.calculate_path_callback,
            1
        )

        # Publisher to motor
        self.motor_publisher = self.create_publisher(
            Float32,
            'cmd_vel',
            3
        )

        # # Publisher to steering
        self.steering_publisher = self.create_publisher(
            Float32,
            'cmd_turn',
            3
        )

        self.logger.info("Initialize Pathfinder Node")

    def calculate_path_callback(self, msg: Float64MultiArray):
        """
        Calculate commands for steering and motor from opencv_pathfinder efficiently
        :param msg: Float64MultiArray of [left angle from center to base of track from image, right angle ...]
        :return: Publishes commands to motor & steering
        """
        self.angles = (msg.data[0], msg.data[1])

        motor_speed = 20 * random.random()
        motor_cmd = Float32()
        motor_cmd.data = motor_speed

        steering_angle = 180 * random.random() - 90
        steering_cmd = Float32()
        steering_cmd.data = steering_angle

        self.steering_publisher.publish(steering_cmd)
        self.motor_publisher.publish(motor_cmd)


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
