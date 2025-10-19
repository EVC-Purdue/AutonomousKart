import time

from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image


class PathfinderNode(Node):
    def __init__(self):
        super().__init__('opencv_pathfinder_node')
        self.logger = self.get_logger()
        self.bridge = CvBridge()
        self.frame_count = 0
        self.total_time = 0

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos
        )

        self.logger.info("Pathfinder Node started - subscribed to /camera/image_raw")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.frame_count += 1
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        current_time = self.get_clock().now().to_msg()
        current_time_sec = current_time.sec + current_time.nanosec / 1e9
        self.logger.info(f"Received {self.frame_count} frames, size: {frame.shape}")
        self.logger.info(f"Average Time: {self.total_time / self.frame_count if self.frame_count else 1}")
        self.total_time += current_time_sec - msg_time


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