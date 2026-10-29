import math
import os
import time
import traceback

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from autonomous_kart.nodes.opencv_pathfinder.kart_perception import KartPerception, PoseBuffer
from autonomous_kart.nodes.opencv_pathfinder.kart_ranging import CameraModel, RangingParams
from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import TrackFrame
from autonomous_kart.nodes.pathfinder.opponents import encode_opponents


class KartDetectorNode(Node):
    """camera/image_raw + odom -> /opponents (see kart_perception.py).

    Runs as its own executable so it can sit on whichever computer has the
    camera (only the few numbers on /opponents cross the network) and never
    blocks the track-angle node or the 60 Hz planner.
    """

    def __init__(self):
        super().__init__(
            "kart_detector_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()
        self.bridge = CvBridge()
        self.poses = PoseBuffer()
        self.perception = None
        self._times_ms = []
        self._last_stats = time.monotonic()

        self.opp_pub = self.create_publisher(Float64MultiArray, "opponents", 5)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)

        model = self._param("model_path", "/ws/models/kart_192x320.onnx", str)
        if not os.path.exists(model):
            # No model yet must not take the stack down: publish nothing and say why.
            self.logger.error(f"kart detector model not found at {model}; /opponents disabled")
            return
        try:
            self.perception = self._build(model)
        except Exception:
            self.logger.error(f"kart detector failed to start:\n{traceback.format_exc()}")
            return

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, "camera/image_raw", self._on_image, qos)
        self.logger.info(f"kart detector running: {model} on {self.perception.detector.providers[0]}")

    def _build(self, model: str) -> KartPerception:
        from autonomous_kart.nodes.opencv_pathfinder.kart_detector import KartDetector

        g = self._param
        detector = KartDetector(
            model, conf=g("conf", 0.35, float), iou=g("iou", 0.5, float),
            providers=g("providers", [], list) or None, threads=g("threads", 2, int),
            fp16=g("fp16", True, bool), engine_cache_dir=g("engine_cache_dir", "/root/.cache/trt_engines", str),
        )
        cam = CameraModel(
            fx=g("camera.fx", 180.0, float), fy=g("camera.fy", 180.0, float),
            cx=g("camera.cx", 180.0, float), cy=g("camera.cy", 101.0, float),
            width_px=g("camera.width_px", 360, int), height_px=g("camera.height_px", 202, int),
            height_m=g("camera.height_m", 1.2, float), pitch_rad=math.radians(g("camera.pitch_deg", 5.0, float)),
            yaw_rad=math.radians(g("camera.yaw_deg", 0.0, float)),
            mount_x_m=g("camera.mount_x_m", 0.0, float), mount_y_m=g("camera.mount_y_m", 0.0, float),
            dist=tuple(g("camera.dist", [], list)),
        )
        ranging = RangingParams(
            kart_height_m=g("ranging.kart_height_m", 1.0, float),
            kart_height_sigma_m=g("ranging.kart_height_sigma_m", 0.2, float),
            center_offset_m=g("ranging.center_offset_m", 0.8, float),
            pitch_sigma_rad=math.radians(g("ranging.pitch_sigma_deg", 1.0, float)),
        )
        line = self._load_line(g("line_path", "", str))
        tracker = {k: p.value for k, p in self.get_parameters_by_prefix("tracker").items()}
        return KartPerception(
            detector, cam, TrackFrame(line), ranging, tracker,
            roi_frac=g("roi_frac", [0.0, 0.0, 1.0, 1.0], list),
            max_range_m=g("max_range_m", 40.0, float),
            ego_xy_sigma_m=g("ego_xy_sigma_m", 0.1, float),
            ego_yaw_sigma_rad=math.radians(g("ego_yaw_sigma_deg", 1.0, float)),
        )

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:  # unstamped odom: fall back to arrival time
            t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        self.poses.add(t, (float(p.x), float(p.y), yaw, float(msg.twist.twist.linear.x)))

    def _on_image(self, msg: Image):
        t0 = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.poses.at(stamp)
        if pose is None:
            self.logger.warning("no ego pose near the image stamp; skipping frame", throttle_duration_sec=5.0)
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        try:
            opps = self.perception.process(frame, stamp, pose)
        except Exception:
            self.logger.error(f"kart perception error:\n{traceback.format_exc()}", throttle_duration_sec=5.0)
            return
        self.opp_pub.publish(Float64MultiArray(data=encode_opponents(opps)))

        self._times_ms.append(1e3 * (time.perf_counter() - t0))
        if time.monotonic() - self._last_stats > 5.0:
            ts = np.asarray(self._times_ms)
            self.logger.info(f"kart detector: {len(ts) / 5.0:.0f} fps, total p50 {np.median(ts):.1f} "
                             f"p99 {np.percentile(ts, 99):.1f} ms, {len(opps)} opponents")
            self._times_ms.clear()
            self._last_stats = time.monotonic()

    def _load_line(self, path: str) -> list:
        rows = []
        with open(path) as f:
            for line in f:
                try:
                    rows.append(tuple(float(x) for x in line.strip().split(",")))
                except ValueError:
                    continue  # header
        return rows

    def _param(self, name, default, cast=None):
        try:
            v = self.get_parameter(name).value
        except Exception:
            v = None
        if v is None:
            return default
        if cast is list:
            return list(v)
        try:
            return cast(v) if cast is not None else v
        except (TypeError, ValueError):
            return default


def main(args=None):
    rclpy.init(args=args)
    node = KartDetectorNode()
    try:
        # Single-threaded on purpose: odom and image callbacks share the pose
        # buffer, and inference (~5 ms) only delays odom by one frame.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
