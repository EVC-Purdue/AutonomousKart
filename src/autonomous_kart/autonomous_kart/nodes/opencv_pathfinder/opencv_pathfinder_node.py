import time

from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher

from autonomous_kart.nodes.opencv_pathfinder.angle_calculator import (
    calculate_track_angles,
)


class OpenCVPathfinderNode(Node):
    def __init__(self):
        super().__init__("opencv_pathfinder_node")
        self.last_log_time = 0
        self.frames_since_last_log: int = 0
        self.logger: RcutilsLogger = self.get_logger()
        self.bridge: CvBridge = CvBridge()
        self.frame_count: int = 0
        self.angle_msg: Float32MultiArray = None

        self.declare_parameter("system_frequency", 60)
        self.system_frequency: float = self.get_parameter("system_frequency").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0, nanoseconds=int(1e9 / self.system_frequency)),
        )

        # Subscribe to camera
        self.image_sub: Subscription = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, qos
        )

        # Publishes two angles in (left angle from center of vision to base of track line, right angle ...)
        self.angle_pub: Publisher = self.create_publisher(
            Float32MultiArray,
            "track_angles",
            5,
        )

        self.logger.info("Pathfinder Node started - subscribed to /camera/image_raw")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.frame_count += 1
        self.frames_since_last_log += 1

        # Calculate FPS every second
        current_time = self.get_clock().now()
        elapsed = (current_time.nanoseconds - self.last_log_time) / 1e9

        if elapsed >= 1.0:  # Log every second
            fps = self.frames_since_last_log / elapsed
            self.logger.info(
                f"Receiving {fps:.1f} fps | Total frames: {self.frame_count}"
            )
            self.last_log_time = current_time.nanoseconds
            self.frames_since_last_log = 0

        # Publish angles
        angles = calculate_track_angles(frame)
        if not self.angle_msg:
            self.angle_msg = Float32MultiArray(data=angles)
        else:
            # self.angle_pub.publish(Float32MultiArray(data=angles))
            self.angle_msg.data = angles
        self.angle_pub.publish(self.angle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVPathfinderNode()

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


if __name__ == "__main__":
    main()
