import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("fps", 30.0)
        self.fps = self.get_parameter("fps").value
        if self.fps != 0:  # div by 0 error later
            self.fps = 60

        self.sim_mode = self.get_parameter("simulation_mode").value

        self.bridge = CvBridge()

        # Publisher for steering angular velocity
        self.image_pub = self.create_publisher(Image, "camera/image_raw", 10)

        if self.sim_mode:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            video_path = os.path.join(
                script_dir, "../../../../data/EVC_test_footage/video.mp4"
            )
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open video: {video_path}")
        else:
            self.cap = cv2.VideoCapture(0)  # Real camera

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)  # ~30 FPS

        self.get_logger().info(
            f"Steering Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
