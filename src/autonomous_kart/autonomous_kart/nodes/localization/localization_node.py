"""
Localization node — publishes Odometry on /odom.

Sim mode:  Integrates bicycle kinematics from cmd_drive [throttle, steering].
Real mode: EKF over GPS xy + GPS-derived heading + GPS-derived speed,
           predicted at system_frequency by the kinematic bicycle model
           driven by cmd_drive steering.
"""
import math
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from .ekf import LocalizationEKF
from .sim_bicycle import BicycleModel


class LocalizationNode(Node):
    def __init__(self):
        super().__init__(
            "localization_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()
        self.sim_mode = self.get_parameter("simulation_mode").value
        self.logger.info(f"E_COMMS mode: {self.sim_mode}")

        # Output
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.sim_mode:
            self._init_sim()
        else:
            self._init_real()

        self.logger.info(
            f"Localization Node started — Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

    def _init_sim(self):
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        v_max = float(self.get_parameter("v_max_mps").value)
        steer_max = float(self.get_parameter("steer_max_deg").value)
        hz = float(self.get_parameter("system_frequency").value)

        self.model = BicycleModel(wheelbase, v_max, steer_max)
        self.dt = 1.0 / hz

        # Spawn at racing line start if available
        self._auto_spawn()

        self.cmd_motor = 0.0
        self.cmd_steer = 0.0

        self.create_subscription(Float32MultiArray, "cmd_drive", self._cb_drive, 5)
        self.create_timer(self.dt, self._sim_step)

    def _auto_spawn(self):
        """Set initial pose from first point of racing line."""
        try:
            line_path = self.get_parameter("line_path").value
            with open(line_path, "r") as f:
                rows = []
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) >= 3:
                        try:
                            rows.append((float(parts[1]), float(parts[2])))
                        except ValueError:
                            continue
                if len(rows) >= 2:
                    x0, y0 = rows[0]
                    dx = rows[1][0] - x0
                    dy = rows[1][1] - y0
                    yaw0 = math.atan2(dy, dx)
                    self.model.reset(x0, y0, yaw0)
                    self.logger.info(
                        f"Spawned at ({x0:.1f}, {y0:.1f}, {math.degrees(yaw0):.0f}°)"
                    )
        except Exception:
            self.logger.warning("line_path missing")
            pass  # No line_path or file missing — start at origin

    def _cb_drive(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self.cmd_motor = float(msg.data[0])
        self.cmd_steer = float(msg.data[1])

    def _sim_step(self):
        x, y, yaw, speed = self.model.step(self.cmd_motor, self.cmd_steer, self.dt)
        self._publish_odom(x, y, yaw, speed)

    def _init_real(self):
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        steer_max_rad = math.radians(float(self.get_parameter("steer_max_deg").value))
        hz = float(self.get_parameter("system_frequency").value)
        self.dt = 1.0 / hz

        self.ekf = LocalizationEKF(wheelbase_m=wheelbase, steer_max_rad=steer_max_rad)

        # Pull tunables from localization.yaml. Fall back to the class defaults
        # if a key is missing (lets the node run even without the yaml).
        self.ekf.q_pos = float(self._param("pos_noise", self.ekf.q_pos))
        self.ekf.q_yaw_rate = float(self._param("yaw_noise", self.ekf.q_yaw_rate))
        self.ekf.q_accel = float(self._param("accel_noise", self.ekf.q_accel))

        self.pseudo_speed_gate = float(self._param("pseudo_speed_gate", 0.5))
        self.pseudo_disp_gate = float(self._param("pseudo_disp_gate", 0.10))
        self.init_disp_min = float(self._param("init_disp_min", 0.30))
        self.gps_max_age = float(self._param("gps_max_age", 0.5))
        self.cmd_drive_max_age = float(self._param("cmd_drive_max_age", 1.0))
        self.gps_dropout_warn = float(self._param("gps_dropout_warn", 2.0))
        self.sigma_xy_floor = float(self._param("sigma_xy_floor", 0.02))

        # Last GPS fix cache, for two-fix init and heading/speed pseudo-measurement.
        self._last_gps_xy = None  # (x, y)
        self._last_gps_t = None  # float seconds
        self._last_gps_sigma_xy = None  # float (m)

        # Cached cmd_drive steering (rad) + timestamp (seconds).
        self._steer_rad = 0.0
        self._steer_stamp = None

        self.create_subscription(Odometry, "gps", self.gps_callback, 10)
        self.create_subscription(Float32MultiArray, "cmd_drive", self._cb_drive_real, 5)
        self.create_timer(self.dt, self._predict_step)

    def _param(self, name, default):
        """Return the parameter value if declared, else `default`."""
        try:
            return self.get_parameter(name).value
        except Exception:
            return default

    def _cb_drive_real(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        # cmd_drive[1] is currently degrees (see CLAUDE.md unit note).
        self._steer_rad = math.radians(float(msg.data[1]))
        self._steer_stamp = self._now_s()

    def _now_s(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def _predict_step(self):
        if not self.ekf.initialized:
            return

        # Use cached steering only if it is fresh; otherwise drive predict with delta=0
        if (
                self._steer_stamp is None
                or (self._now_s() - self._steer_stamp) > self.cmd_drive_max_age
        ):
            steer = 0.0
        else:
            steer = self._steer_rad

        self.ekf.predict(self.dt, steer)

        # Warn (rate-limited via ROS logger throttle) on GPS dropout.
        if self._last_gps_t is not None:
            gap = self._now_s() - self._last_gps_t
            if gap > self.gps_dropout_warn:
                self.logger.warning(
                    f"GPS dropout: {gap:.1f}s since last fix", throttle_duration_sec=2.0
                )

        px, py, yaw, v = self.ekf.x
        cov = self._odom_cov_from_P(self.ekf.P)
        self._publish_odom(px, py, yaw, v, cov=cov)

    @staticmethod
    def _odom_cov_from_P(P: np.ndarray) -> list:
        """Map the 4x4 EKF P into the flat 36-vector of nav_msgs/Odometry.

        Index layout (row-major 6x6 over [x, y, z, roll, pitch, yaw]):
            (0,0)=0    (0,1)=1    (1,0)=6    (1,1)=7
            (0,5)=5    (5,0)=30   (1,5)=11   (5,1)=31
            (5,5)=35
        """
        cov = [0.0] * 36
        cov[0] = float(P[0, 0])
        cov[1] = float(P[0, 1])
        cov[6] = float(P[1, 0])
        cov[7] = float(P[1, 1])
        cov[5] = float(P[0, 2])
        cov[30] = float(P[2, 0])
        cov[11] = float(P[1, 2])
        cov[31] = float(P[2, 1])
        cov[35] = float(P[2, 2])
        # Leave z/roll/pitch (indices 14, 21, 28) at 0 — unobserved.
        return cov

    def gps_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        x, y = float(p.x), float(p.y)
        t = self._now_s()

        sigma_xx = float(msg.pose.covariance[0])
        sigma_yy = float(msg.pose.covariance[7])
        # Floor to avoid div-by-zero in heading variance.
        floor = self.sigma_xy_floor ** 2
        sigma_xx = max(sigma_xx, floor)
        sigma_yy = max(sigma_yy, floor)
        sigma_xy_scalar = math.sqrt(0.5 * (sigma_xx + sigma_yy))
        R_xy = np.diag([sigma_xx, sigma_yy])

        if not self.ekf.initialized:
            # Two-fix initialization: cache, then on a sufficient-displacement
            # second fix, seed state and let the predict timer start publishing.
            if self._last_gps_xy is None:
                self._last_gps_xy = (x, y)
                self._last_gps_t = t
                self._last_gps_sigma_xy = sigma_xy_scalar
                return
            dx = x - self._last_gps_xy[0]
            dy = y - self._last_gps_xy[1]
            d = math.hypot(dx, dy)
            dt_gps = max(t - self._last_gps_t, 1e-3)
            if d < self.init_disp_min or dt_gps > self.gps_max_age:
                # Slide the cache forward and wait for more motion.
                self._last_gps_xy = (x, y)
                self._last_gps_t = t
                self._last_gps_sigma_xy = sigma_xy_scalar
                return
            yaw0 = math.atan2(dy, dx)
            v0 = d / dt_gps
            sigma_yaw_init = sigma_xy_scalar / max(d, self.sigma_xy_floor)
            sigma_v_init = sigma_xy_scalar * math.sqrt(2.0) / dt_gps
            P0 = np.diag([
                sigma_xx,
                sigma_yy,
                sigma_yaw_init ** 2,
                sigma_v_init ** 2,
            ])
            self.ekf.reset(x, y, yaw0, v0, P=P0)
            self.logger.info(
                f"EKF init: pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw0):.1f}° v={v0:.2f} m/s"
            )
            self._last_gps_xy = (x, y)
            self._last_gps_t = t
            self._last_gps_sigma_xy = sigma_xy_scalar
            return

        # Already initialized: run corrections, do NOT publish
        self.ekf.update_gps_xy(x, y, R_xy)

        if self._last_gps_xy is not None and self._last_gps_t is not None:
            dx = x - self._last_gps_xy[0]
            dy = y - self._last_gps_xy[1]
            d = math.hypot(dx, dy)
            dt_gps = max(t - self._last_gps_t, 1e-3)
            v_meas = d / dt_gps
            if (
                    v_meas > self.pseudo_speed_gate
                    and d > self.pseudo_disp_gate
                    and dt_gps < self.gps_max_age
            ):
                yaw_meas = math.atan2(dy, dx)
                sigma_xy = max(self._last_gps_sigma_xy or sigma_xy_scalar,
                               self.sigma_xy_floor)
                var_yaw = (sigma_xy / max(d, self.sigma_xy_floor)) ** 2
                var_v = (sigma_xy * math.sqrt(2.0) / dt_gps) ** 2
                self.ekf.update_heading(yaw_meas, var_yaw)
                self.ekf.update_speed(v_meas, var_v)

        self._last_gps_xy = (x, y)
        self._last_gps_t = t
        self._last_gps_sigma_xy = sigma_xy_scalar

    def _publish_odom(self, x: float, y: float, yaw: float, speed: float, cov=None):
        now = self.get_clock().now().to_msg()
        q = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = speed
        if cov is not None:
            odom.pose.covariance = cov
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
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