import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.logger = self.get_logger()

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("fps", 30.0)
        self.fps = self.get_parameter("fps").value
        self.frame_counter = 0

        if self.fps == 0:  # div by 0 error later
            self.fps = 60

        self.sim_mode = self.get_parameter("simulation_mode").value

        self.bridge = CvBridge()

        # Publisher for steering angular velocity
        self.image_pub = self.create_publisher(Image, "camera/image_raw", 10)

        if self.sim_mode:
            # script_dir = os.path.dirname(os.path.abspath(__file__))
            video_path = "/ws/data/EVC_test_footage/video.mp4"
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self.logger.info(f"Failed to open video: {video_path}")
        else:
            self.cap = cv2.VideoCapture(0)  # Real camera
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.logger.info(f"Video FPS: {self.video_fps}, Given FPS: {self.fps}")
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.logger.info(
            f"Steering Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

    def timer_callback(self):
        """
        Publishes the next frame
        """
        increment = self.video_fps / self.fps
        # Check if increment got to next frame
        if int(self.frame_counter + increment) > int(self.frame_counter):
            if self.sim_mode:
                frame_number = int(self.frame_counter + increment)

                self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("Failed to read frame")

                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                if self.frame_counter % self.fps == 0:
                    self.logger.info(f"Published frame {frame_number}")

            else:
                # TODO: Implement real mode
                pass

        self.frame_counter += increment


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
