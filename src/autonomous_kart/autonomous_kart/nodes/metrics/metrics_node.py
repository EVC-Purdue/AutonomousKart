import threading
from collections import deque
from typing import Deque, Dict, Any

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int16


class MetricsNode(Node):
    def __init__(self):
        super().__init__("metrics_node")

        self.declare_parameter("simulation_mode", False)
        self.sim_mode = self.get_parameter("simulation_mode").value

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()
        self.last_avg_rate = 0.0

        self._lock = threading.Lock()
        self._logs: Deque[Dict[str, Any]] = deque(maxlen=1000)

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        # Subscribe to commands
        publishers = {
            "cmd_vel": Float32MultiArray,
            "cmd_turn": Float32,
            "turn_angle": Float32,
            "motor_speed": Float32,
            "track_angles": Float32MultiArray,
            "camera/image_raw": Image,
            "e_comms/pwm_rx/motor": Int16,
            "e_comms/pwm_rx/steering": Int16,
            "pathfinder_params": Float32MultiArray,
        }
        for topic, type in publishers.items():
            self.cmd_vel_sub = self.create_subscription(
                type,
                topic,
                lambda msg, t=topic: self.cmd_callback(t, msg),  # store topic name
                3,  # 3 for lower memory usage - may drop some messages rarely
            )

        self.get_logger().info("Metric Node started")

    def log_command_rate(self):
        """Log average commands per second every 5 seconds"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_log_time).nanoseconds / 1e9

        if elapsed > 0:
            avg_rate = self.cmd_count / elapsed
            self.get_logger().info(
                f"Average command rate: {avg_rate:.2f} commands/sec "
                f"(Total: {self.cmd_count} in {elapsed:.1f}s)"
            )
            self.last_avg_rate = avg_rate
        # Reset counters
        self.cmd_count = 0
        self.last_log_time = current_time

    def cmd_callback(self, topic: str, msg: Any):
        """Subscribes to all nodes and saves responses"""
        self.cmd_count += 1
        self.current_speed = msg

        record: Dict[str, Any] = {
            "topic": topic,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }

        if isinstance(msg, Float32):
            record["value"] = float(msg.data)
        elif isinstance(msg, Int16):
            record["value"] = int(msg.data)
        elif isinstance(msg, Float32MultiArray):
            data_list = list(msg.data)
            record["value"] = data_list[:10]  # Truncate
            record["len"] = len(data_list)
        elif isinstance(msg, Image):
            record["image_meta"] = {
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
            }

        with self._lock:
            self._logs.append(record)

    def get_logs(self):
        with self._lock:
            ret = list(self._logs)
            self._logs.clear()
            return ret


def main(args=None):
    rclpy.init(args=args)

    node = MetricsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error("Unhandled exception", exc_info=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
