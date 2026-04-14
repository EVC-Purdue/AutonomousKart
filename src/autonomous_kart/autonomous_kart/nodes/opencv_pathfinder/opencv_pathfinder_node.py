import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import traceback

from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from angle import AngleFinder


class OpenCVPathfinderNode(Node):
    def __init__(self):
        super().__init__(
            "opencv_pathfinder_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.last_log_time = 0
        self.frames_since_last_log = 0
        self.logger = self.get_logger()
        self.bridge = CvBridge()
        self.frame_count = 0
        self.angle_msg = None

        self.system_frequency = self.get_parameter("system_frequency").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0, nanoseconds=int(1e9 / self.system_frequency)),
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, qos
        )

        # Publishes two angles in (left angle from center of vision to base of track line, right angle ...)
        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            "track_angles",
            5,
        )

        self.logger.info("Pathfinder Node started - subscribed to /camera/image_raw")
        self.angle_finder = AngleFinder()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.frame_count += 1
        self.frames_since_last_log += 1

        # Calculate FPS every second
        current_time = self.get_clock().now().nanoseconds
        elapsed = (current_time - self.last_log_time) / 1e9

        if elapsed >= 1.0:  # Log every second
            fps = self.frames_since_last_log / elapsed
            self.logger.info(
                f"Receiving {fps:.1f} fps | Total frames: {self.frame_count}"
            )
            self.last_log_time = current_time
            self.frames_since_last_log = 0

        # Publish angles
        right_angle, left_angle = self.angle_finder.get_img_angles(frame, percent=0.0)
        msg = Float32MultiArray()
        msg.data = [
            float(right_angle) if right_angle is not None else float('nan'),
            float(left_angle) if left_angle is not None else float('nan'),
        ]
        self.angle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = OpenCVPathfinderNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.running = False
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
