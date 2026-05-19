import traceback
import rclpy
import os
import math

from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from autonomous_kart.nodes.opencv_pathfinder.angle import AngleFinder

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
        self.angle_finder = AngleFinder(self.logger)

        self.system_frequency = self.get_parameter("system_frequency").value
        self.debug_mode = self.get_parameter("debug_mode").value
        self.pic_offset = self.get_parameter("pic_offset").value
        self.top_per = self.get_parameter("top_per").value
        self.bottom_per = self.get_parameter("bottom_per").value
        self.pixel_range = self.get_parameter("pixel_range").value
        self.capture_frequency = self.get_parameter("capture_frequency").value
        self.log_dir = self.get_parameter("log_dir").value
        self.log_file = self.get_parameter("log_file").value

        # NEW: Parameter to toggle between live camera processing and simulator raycasts
        # Declare it with a default of False so it doesn't break physical kart deployments
        if not self.has_parameter("use_sim_vision"):
            self.declare_parameter("use_sim_vision", False)
        self.use_sim_vision = self.get_parameter("use_sim_vision").value

        self.log_folder = os.path.join(self.log_dir, self.log_file)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0, nanoseconds=int(1e9 / self.system_frequency)),
        )

        # Publishes two angles: [left_angle, right_angle]
        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            "track_angles",
            5,
        )

        # Conditional Routing based on execution environment
        if self.use_sim_vision:
            self.logger.info("Pathfinder Node running in SIM MODE - subscribing to /sim/camera_angles")
            self.sim_sub = self.create_subscription(
                Float32MultiArray, 
                "sim/camera_angles", 
                self.sim_angles_callback, 
                10
            )
        else:
            self.logger.info("Pathfinder Node running in LIVE MODE - subscribing to /camera/image_raw")
            self.image_sub = self.create_subscription(
                Image, "camera/image_raw", self.image_callback, qos
            )

    def image_callback(self, msg):
        """Processes real image frames from physical hardware or high-fidelity gazebos."""
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.frame_count += 1
        self.frames_since_last_log += 1

        current_time = self.get_clock().now().nanoseconds
        elapsed = (current_time - self.last_log_time) / 1e9

        if elapsed >= 1.0:
            fps = self.frames_since_last_log / elapsed
            self.logger.info(f"Receiving {fps:.1f} fps | Total frames: {self.frame_count}")
            self.last_log_time = current_time
            self.frames_since_last_log = 0

        right_angle, left_angle = self.angle_finder.get_img_angles(
            frame, 
            log_folder=self.log_folder, 
            frame_count=self.frame_count, 
            capture_frequency=self.capture_frequency,
            debug=self.debug_mode, 
            top_per=self.top_per, 
            bottom_per=self.bottom_per,
            pixel_range=self.pixel_range, 
            pic_offset=self.pic_offset
        )

        if right_angle is None or left_angle is None:
            self.logger.warn("Right or Left angle returned None")

        self.publish_angles(left_angle, right_angle)

    def sim_angles_callback(self, msg):
        """Processes pre-calculated geometric intersection rays direct from viz.html."""
        if len(msg.data) >= 2:
            left_angle = msg.data[0]
            right_angle = msg.data[1]
            
            # Simple telemetry frame logging every 20 callbacks (~1 second at 20Hz)
            self.frame_count += 1
            if self.frame_count % 20 == 0:
                self.logger.info(f"[SIM VISION] Catching geometric sweeps -> L: {left_angle:.1f}°, R: {right_angle:.1f}°")
                
            self.publish_angles(left_angle, right_angle)

    def publish_angles(self, left, right):
        """Unified internal publisher for downstream nodes (like your MPC/PID planner)."""
        out_msg = Float32MultiArray()
        out_msg.data = [
            float(left) if left is not None else float('nan'),
            float(right) if right is not None else float('nan'),
        ]
        self.angle_pub.publish(out_msg)


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
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()