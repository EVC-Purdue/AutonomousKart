import json
import threading
import time
from enum import Enum

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32MultiArray, UInt16, Empty


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
            "pose_cov": [0.0] * 36,
            "twist_cov": [0.0] * 36,
            "stamp_ns": 0,
        }
        self.cmd_data = {"motor": 0.0, "steer": 0.0}

        # e_comms ADCB state snapshot
        self.e_comms_data = {
            "logic_state": "not initialized...",
            "running_mode": "not initialized...",
            "throttle_pwm": 0,
            "steering_pwm": 0,
        }

        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.odom_callback, 5
        )
        # Track combined drive commands for telemetry
        self.create_subscription(Float32MultiArray, "cmd_drive", self._drive_callback, 5)

        # e_comms ADCB telemetry
        self.create_subscription(String, "e_comms/logic_state", self._logic_state_callback, 1)
        self.create_subscription(String, "e_comms/running_mode", self._running_mode_callback, 1)
        self.create_subscription(UInt16, "e_comms/throttle_pwm", self._throttle_pwm_callback, 1)
        self.create_subscription(UInt16, "e_comms/steering_pwm", self._steering_pwm_callback, 1)

        # Dynamic line output
        self.dynamic_line_data = {"active": False, "strategy": None, "merge_idx": -1, "points": []}
        self.create_subscription(
            String, "pathfinder/dynamic_line", self.dynamic_line_callback, 1
        )

        # MPC status snapshot (cost breakdown for /mpc_status endpoint)
        # Field layout matches mpc.py:_publish_status (append-only).
        self.mpc_status_data = {
            "received": False,
            "mode": 0, "success": False, "solve_ms": 0.0,
            "s": 0.0, "d": 0.0, "psi_track": 0.0,
            "v": 0.0, "v_target": 0.0,
            "delta_cmd_deg": 0.0, "accel_cmd": 0.0,
            "cost_total": 0.0, "margin_min": 0.0,
            "res_s": 0.0, "res_d": 0.0,
            "nom_s": 0.0, "nom_d": 0.0,
            "res_es": 0.0, "res_ed": 0.0,
            "samples_trained": 0.0,
            "costs": {
                "d": 0.0, "heading": 0.0, "speed": 0.0,
                "delta": 0.0, "drate": 0.0, "accel": 0.0,
                "boundary": 0.0, "progress": 0.0,
                "terminal_d": 0.0, "terminal_heading": 0.0,
                "a_lat": 0.0,
                "edge": 0.0,
            },
            "x": 0.0, "y": 0.0, "yaw_rad": 0.0,
            "throttle_mps_out": 0.0, "v_s": 0.0, "v_d": 0.0,
            "closest_idx_active": 0, "closest_idx_static": 0,
            "rejoin_active": False, "merge_idx": -1,
            "kappa_local": 0.0, "consec_failures": 0,
            "corridor_half": 0.0,
        }
        self.create_subscription(
            Float32MultiArray, "mpc/status", self._mpc_status_callback, 5
        )

        # GPS status snapshot (fix quality, RTK, sigmas, RTCM stats)
        self.gps_status_data = {"fix_quality": 0, "fix_label": "INVALID"}
        self.create_subscription(String, "gps/status", self._gps_status_callback, 1)

        # Runtime swap publishers: residual mode, planner, racing-line path.
        self.residual_mode_publisher = self.create_publisher(String, "mpc/residual_mode", 1)
        self.planner_publisher = self.create_publisher(String, "pathfinder/planner", 1)
        self.line_publisher = self.create_publisher(String, "pathfinder/line_path", 1)

        # IMU calibration plumbing
        self.imu_calibrate_publisher = self.create_publisher(Empty, "imu/calibrate", 1)
        self.imu_status_data = {"state": "unknown"}
        self.create_subscription(
            String, "imu/calibration_status", self._imu_status_callback, 1
        )

        # Latest IMU reading snapshot
        self.imu_data = {
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
            "stamp_ns": 0,
        }
        self.create_subscription(Imu, "imu", self._imu_callback, 5)

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

    def _mpc_status_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 44:
            return
        snapshot = {
            "received": True,
            "mode": int(data[0]),
            "success": bool(data[1] > 0.5),
            "solve_ms": float(data[2]),
            "s": float(data[3]),
            "d": float(data[4]),
            "psi_track": float(data[5]),
            "v": float(data[6]),
            "v_target": float(data[7]),
            "delta_cmd_deg": float(data[8]),
            "accel_cmd": float(data[9]),
            "cost_total": float(data[10]),
            "margin_min": float(data[11]),
            "res_s": float(data[12]),
            "res_d": float(data[13]),
            "nom_s": float(data[14]),
            "nom_d": float(data[15]),
            "res_es": float(data[16]),
            "res_ed": float(data[17]),
            "samples_trained": float(data[18]),
            "costs": {
                "d": float(data[19]),
                "heading": float(data[20]),
                "speed": float(data[21]),
                "delta": float(data[22]),
                "drate": float(data[23]),
                "accel": float(data[24]),
                "boundary": float(data[25]),
                "progress": float(data[26]),
                "terminal_d": float(data[27]),
                "terminal_heading": float(data[28]),
                "a_lat": float(data[29]),
                "edge": float(data[30]),
            },
            "x": float(data[31]),
            "y": float(data[32]),
            "yaw_rad": float(data[33]),
            "throttle_mps_out": float(data[34]),
            "v_s": float(data[35]),
            "v_d": float(data[36]),
            "closest_idx_active": int(data[37]),
            "closest_idx_static": int(data[38]),
            "rejoin_active": bool(data[39] > 0.5),
            "merge_idx": int(data[40]),
            "kappa_local": float(data[41]),
            "consec_failures": int(data[42]),
            "corridor_half": float(data[43]),
        }
        with self._lock:
            self.mpc_status_data = snapshot

    def get_mpc_status(self):
        with self._lock:
            snap = dict(self.mpc_status_data)
            snap["costs"] = dict(snap["costs"])
            return snap

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
                "pose_cov": list(msg.pose.covariance),
                "twist_cov": list(msg.twist.covariance),
                "stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            }

    def _drive_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        with self._lock:
            self.cmd_data["motor"] = float(msg.data[0])
            self.cmd_data["steer"] = float(msg.data[1])

    def _logic_state_callback(self, msg: String):
        with self._lock:
            self.e_comms_data["logic_state"] = msg.data

    def _running_mode_callback(self, msg: String):
        with self._lock:
            self.e_comms_data["running_mode"] = msg.data

    def _throttle_pwm_callback(self, msg: UInt16):
        with self._lock:
            self.e_comms_data["throttle_pwm"] = msg.data

    def _steering_pwm_callback(self, msg: UInt16):
        with self._lock:
            self.e_comms_data["steering_pwm"] = msg.data

    def get_odom(self):
        with self._lock:
            return {**self.odom_data, **self.cmd_data, "state": self.state}

    def _imu_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.imu_status_data = data

    def get_imu_status(self):
        with self._lock:
            return dict(self.imu_status_data)

    def trigger_imu_calibration(self):
        self.imu_calibrate_publisher.publish(Empty())

    def set_residual_mode(self, mode: str) -> tuple[bool, str]:
        m = (mode or "").strip().lower()
        if m not in ("off", "shadow", "apply"):
            return False, f"mode must be off|shadow|apply (got '{mode}')"
        self.residual_mode_publisher.publish(String(data=m))
        return True, m

    def set_planner(self, planner: str) -> tuple[bool, str]:
        name = (planner or "").strip().lower()
        if name not in ("mpc", "pure_pursuit", "opencv"):
            return False, f"planner must be mpc|pure_pursuit|opencv (got '{planner}')"
        self.planner_publisher.publish(String(data=name))
        return True, name

    def set_line(self, path: str) -> tuple[bool, str]:
        p = (path or "").strip()
        if not p:
            return False, "path is empty"
        self.line_publisher.publish(String(data=p))
        return True, p

    def _imu_callback(self, msg: Imu):
        a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        stamp = msg.header.stamp
        with self._lock:
            self.imu_data = {
                "ax": a.x, "ay": a.y, "az": a.z,
                "gx": g.x, "gy": g.y, "gz": g.z,
                "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                "stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            }

    def get_imu(self):
        with self._lock:
            return dict(self.imu_data)

    def get_e_comms(self):
        with self._lock:
            return dict(self.e_comms_data)

    def _gps_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.gps_status_data = data

    def get_gps_status(self):
        with self._lock:
            return dict(self.gps_status_data)


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
