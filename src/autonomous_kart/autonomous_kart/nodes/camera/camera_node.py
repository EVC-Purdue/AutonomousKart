import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.last_callback_time = None
        self.logger = self.get_logger()

        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("fps", 10.0)
        self.fps = self.get_parameter("fps").value
        self.frame_counter = 0

        if self.fps == 0:  # div by 0 error later
            self.fps = 10

        self.sim_mode = self.get_parameter("simulation_mode").value

        self.bridge = CvBridge()

        # Publisher for steering angular velocity
        qos = QoSProfile(
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.image_pub = self.create_publisher(Image, "camera/image_raw", qos)

        if self.sim_mode:
            video_path = "/ws/data/EVC_test_footage/video.mp4"
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self.logger.info(f"Failed to open video: {video_path}")
            self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)


            self.latest_frame = None
            self.running = True
            self.frame_lock = threading.Lock()

            self.reader_thread = threading.Thread(target=self.read_frames, daemon=True)
            self.reader_thread.start()

        else:
            self.cap = cv2.VideoCapture(0)  # Real camera

        self.logger.info(f"Video FPS: {self.video_fps}, Given FPS: {self.fps}")
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.logger.info(
            f"Steering Node started - Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

    def timer_callback(self):
        """
        Publishes the next frame
        """
        if not hasattr(self, 'last_callback_time'):
            self.last_callback_time = time.time()

        if self.sim_mode:
           with self.frame_lock:
                if self.latest_frame is not None:
                    pub_start = time.time()

                    self.image_pub.publish(self.latest_frame)
                    pub_time = (time.time() - pub_start) * 1000
                    self.frame_counter += 1
                else:
                    self.logger.warning("Too slow frames - skipping")
                if self.frame_counter % self.fps == 0:
                    now = time.time()
                    actual_rate = 10 / ((now - self.last_callback_time if self.last_callback_time else 0) if self.last_callback_time else 1)
                    self.logger.info(f"Published {self.frame_counter} frames, actual rate: {actual_rate:.1f} fps")
                    self.last_callback_time = now
        else:
            # TODO: Implement real mode
            pass


    def read_frames(self):
        """
        Background thread to read frames for efficiency
        """
        frame_time = 1.0 / self.video_fps

        while self.running:
            start = time.time()
            ret, frame = self.cap.read()

            # Loop video
            if not ret:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            else:
                height, width = frame.shape[:2]
                target_width = 640
                target_height = int(height * (target_width / width))
                resized = cv2.resize(frame, (target_width, target_height))
                msg = self.bridge.cv2_to_imgmsg(resized, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera"
                with self.frame_lock:
                    self.latest_frame = msg

            elapsed = time.time() - start
            sleep_time = frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    from rclpy.executors import MultiThreadedExecutor
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
