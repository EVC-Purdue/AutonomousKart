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

        assert self.state in [s.value for s in STATES]

        self._lock = threading.Lock()
        self._pub_lock = threading.Lock()
        self._logs = []

        self.state_publisher = self.create_publisher(String, "system_state", 10)
        # Publish speed + steering for manual mode
        self.manual_publisher = self.create_publisher(
            Float32MultiArray, "manual_commands", 10
        )

        # Metrics subscriber to get logs and send them through API
        self.metrics_subscriber = self.create_subscription(
            String, "metrics/logs", self.logs_callback, 5
        )

        # Odom + command state for viz
        self.odom_data = {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0}
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

        self.logger.info("Initialize Master Node")

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
        import math
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.odom_data = {
                "x": p.x, "y": p.y, "yaw": yaw,
                "speed": msg.twist.twist.linear.x,
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
