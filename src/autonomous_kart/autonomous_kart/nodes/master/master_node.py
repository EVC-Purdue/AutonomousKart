import json
import threading
from enum import Enum

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32



class STATES(Enum):
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    STOPPED = "STOPPED"


class MasterNode(Node):
    # Holds kart state and interacts with world
    def __init__(self):
        super().__init__(
            "master_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()
        self.state = self.get_parameter("system_state").value
        self.system_frequency = self.get_parameter("system_frequency").value
        self.path = self.get_parameter("line_path").value

        assert self.state in [s.value for s in STATES]

        self._lock = threading.Lock()
        self._pub_lock = threading.Lock()
        self._logs = []

        self.state_publisher = self.create_publisher(String, "system_state", 10)

        self.state_subscriber = self.create_subscription(
            String, "system_state", self._state_callback, 10
        )
        # Publish speed + steering for manual mode
        self.manual_publisher = self.create_publisher(
            Float32MultiArray, "manual_commands", 10
        )

        # Metrics subscriber to get logs and send them through API
        self.metrics_subscriber = self.create_subscription(
            String, "metrics/logs", self.logs_callback, 5
        )

        # Odom + command state for viz
        self.odom_data = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "yaw": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "wx": 0.0, "wy": 0.0, "wz": 0.0,
            "speed": 0.0,
            "stamp_ns": 0,
        }
        self.cmd_data = {"motor": 0.0, "steer": 0.0}

        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.odom_callback, 5
        )
        self.cmd_vel_sub = self.create_subscription(
            Float32MultiArray, "cmd_vel_steer", lambda m: None, 5  # placeholder
        )
        # Track cmd_vel and cmd_turn individually
        self.create_subscription(Float32, "cmd_vel", self._velocity_callback, 5)
        self.create_subscription(Float32, "cmd_turn", self._turn_callback, 5)

        # Dynamic line output
        self.dynamic_line_data = {"active": False, "strategy": None, "merge_idx": -1, "points": []}
        self.create_subscription(
            String, "pathfinder/dynamic_line", self.dynamic_line_callback, 1
        )

        self.logger.info("Initialize Master Node")

    def dynamic_line_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self.dynamic_line_data = data

    def get_dynamic_line(self):
        with self._lock:
            return dict(self.dynamic_line_data)

    def logs_callback(self, msg):
        logs = json.loads(msg.data)
        with self._lock:
            self._logs.append(logs)

    def update_state(self, state):
        with self._pub_lock:
            if state not in [s.value for s in STATES]:
                self.logger.error(f"State {state} not recognized")
                return

            self.state = state
            self.state_publisher.publish(String(data=state))

    def _state_callback(self, msg: String):
        if msg.data in [s.value for s in STATES]:
            with self._pub_lock:
                self.state = msg.data

    def get_logs(self):
        logs = list(self._logs)
        with self._lock:
            self._logs.clear()
        return logs

    def manual_control(self, speed, steering):
        with self._pub_lock:
            self.manual_publisher.publish(Float32MultiArray(data=[speed, steering]))

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tl = msg.twist.twist.linear
        ta = msg.twist.twist.angular
        import math
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp = msg.header.stamp
        with self._lock:
            self.odom_data = {
                "x": p.x, "y": p.y, "z": p.z,
                "yaw": yaw,
                "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                "vx": tl.x, "vy": tl.y, "vz": tl.z,
                "wx": ta.x, "wy": ta.y, "wz": ta.z,
                "speed": tl.x,
                "stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            }

    def _velocity_callback(self, msg: Float32):
        with self._lock:
            self.cmd_data["motor"] = msg.data

    def _turn_callback(self, msg: Float32):
        with self._lock:
            self.cmd_data["steer"] = msg.data

    def get_odom(self):
        with self._lock:
            return {**self.odom_data, **self.cmd_data, "state": self.state}


def main(args=None):
    rclpy.init(args=args)

    node = MasterNode()

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
