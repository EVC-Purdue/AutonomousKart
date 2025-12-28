import traceback

from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from autonomous_kart.nodes.opencv_pathfinder.angle_calculator import (
    calculate_track_angles,
)


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

    def image_callback(self, msg):
        startTime = self.get_clock().now()
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
        endTime = self.get_clock().now()
        elapsedTime = (endTime - startTime).nanoseconds / 1e6
        # self.logger.info(f"Processing time: {elapsedTime:.2f} ms")


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
