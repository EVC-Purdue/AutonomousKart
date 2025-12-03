import threading
from collections import deque
from typing import Deque, Dict, Any

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int16
from rcl_interfaces.msg import Log
import flask

class MetricsNode(Node):
    def __init__(self):
        super().__init__("metrics_node")

        self.declare_parameter("simulation_mode", False)
        self.sim_mode = self.get_parameter("simulation_mode").value

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now()

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
        }
        for publisher, type in publishers.items():
            self.cmd_vel_sub = self.create_subscription(
                type,
                publisher,
                self.cmd_callback,
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

        # Reset counters
        self.cmd_count = 0
        self.last_log_time = current_time

    def cmd_callback(self, msg: Log):
        """Subscribes to all nodes and saves responses"""
        self.cmd_count += 1
        self.current_speed = msg

        record = {
            "name": msg.name,  # node name
            "level": int(msg.level),
            "msg": msg.msg,
            "file": msg.file,
            "function": msg.function,
            "line": int(msg.line),
            "stamp_sec": msg.stamp.sec,
            "stamp_nanosec": msg.stamp.nanosec,
        }

        with self._lock:
            self.logs.append(record)

    def get_logs(self):
        with self._lock:
            ret = list(self._logs)
            self._logs.clear()



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
