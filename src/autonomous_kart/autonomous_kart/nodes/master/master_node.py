import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

STATES = ["IDLE", "MANUAL", "AUTONOMOUS", "STOPPED"]


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

        assert self.state in STATES

        self._lock = threading.Lock()
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
        self.logger.info("Initialize Master Node")

    def logs_callback(self, msg):
        logs = json.loads(msg.data)
        with self._lock:
            self._logs.append(logs)

    def update_state(self, state):
        if state not in STATES:
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
        self.manual_publisher.publish(Float32MultiArray(data=[speed, steering]))


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
