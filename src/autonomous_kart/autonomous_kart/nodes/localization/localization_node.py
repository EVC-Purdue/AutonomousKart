"""
Localization node publishes Odometry on /odom.

Sim mode:  Integrates bicycle kinematics from cmd_drive [throttle, steering].
Real mode: EKF over [px, py, yaw, v]. Predict runs in the /imu callback at
           ~100 Hz from gyro_z + accel_x; updates come from GPS xy / VTG
           heading / VTG speed and the VESC wheel-speed topic.
"""
import math
import time
import traceback
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from .ekf import LocalizationEKF, _wrap
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

        # Output
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.sim_mode:
            self._init_sim()
        else:
            self._init_real()

        self.logger.info(
            f"Localization Node started Mode: {'SIM' if self.sim_mode else 'REAL'}"
        )

    def _init_sim(self):
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        v_max = float(self.get_parameter("v_max_mps").value)
        steer_max = float(self.get_parameter("steer_max_deg").value)
        hz = float(self.get_parameter("system_frequency").value)

        # Sim noise + actuator delay
        def _np(name, default):
            return float(self._param(name, default))

        steer_slop_deg = _np("sim_steer_slop_deg", 0.0)
        self.model = BicycleModel(
            wheelbase, v_max, steer_max, steer_slop_deg=steer_slop_deg,
        )
        self.dt = 1.0 / hz

        # Spawn at racing line start if available
        self._auto_spawn()

        self._sim_delay_s = _np("sim_actuator_delay_s", 0.0)
        self._sim_motor_std = _np("sim_cmd_motor_noise_mps", 0.0)
        self._sim_steer_std = _np("sim_cmd_steer_noise_deg", 0.0)
        self._sim_pos_std = _np("sim_pos_noise_m", 0.0)
        self._sim_yaw_std_rad = math.radians(_np("sim_yaw_noise_deg", 0.0))
        self._sim_speed_std = _np("sim_speed_noise_mps", 0.0)
        self._sim_rng = np.random.default_rng(
            int(self._param("sim_noise_seed", 0))
        )

        # Throttle /odom publish to GPS frequency
        sim_odom_hz = _np("sim_odom_rate_hz", hz)
        if sim_odom_hz <= 0.0:
            sim_odom_hz = hz
        self._sim_publish_every = max(1, int(round(hz / sim_odom_hz)))
        self._sim_tick_count = 0

        # Cached "currently-applied" cmd after delay queue dequeues
        self.cmd_motor = 0.0
        self.cmd_steer = 0.0
        self._cmd_queue: deque = deque()  # (apply_time_s, motor, steer)

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
                        f"Spawned at ({x0:.1f}, {y0:.1f}, {math.degrees(yaw0):.0f} deg)"
                    )
        except Exception:
            self.logger.warning("line_path missing")
            pass  # No line_path or file missing start at origin

    def _cb_drive(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        rng = self._sim_rng
        motor = float(msg.data[0])
        steer = float(msg.data[1])
        if self._sim_motor_std > 0.0:
            motor += float(rng.normal(0.0, self._sim_motor_std))
        if self._sim_steer_std > 0.0:
            steer += float(rng.normal(0.0, self._sim_steer_std))
        if self._sim_delay_s > 0.0:
            self._cmd_queue.append(
                (self._now_s() + self._sim_delay_s, motor, steer)
            )
        else:
            # No delay -> take effect immediately.
            self.cmd_motor = motor
            self.cmd_steer = steer

    def _sim_step(self):
        # Pop everything from the delay queue that has aged in
        if self._cmd_queue:
            now = self._now_s()
            while self._cmd_queue and self._cmd_queue[0][0] <= now:
                _, self.cmd_motor, self.cmd_steer = self._cmd_queue.popleft()

        x, y, yaw, speed = self.model.step(
            self.cmd_motor, self.cmd_steer, self.dt
        )

        # Throttle /odom to GPS-like cadence apply output noise + publish
        self._sim_tick_count += 1
        if self._sim_tick_count % self._sim_publish_every != 0:
            return

        rng = self._sim_rng
        if self._sim_pos_std > 0.0:
            x += float(rng.normal(0.0, self._sim_pos_std))
            y += float(rng.normal(0.0, self._sim_pos_std))
        if self._sim_yaw_std_rad > 0.0:
            yaw += float(rng.normal(0.0, self._sim_yaw_std_rad))
        if self._sim_speed_std > 0.0:
            speed = max(0.0, speed + float(rng.normal(0.0, self._sim_speed_std)))

        self._publish_odom(x, y, yaw, speed)

    def _init_real(self):
        self.ekf = LocalizationEKF(
            pos_noise=float(self._param("pos_noise", 0.01)),
            yaw_drift_noise=float(self._param("yaw_drift_noise", 0.01)),
            speed_drift_noise=float(self._param("speed_drift_noise", 0.5)),
        )

        self.gps_dropout_warn = float(self._param("gps_dropout_warn", 2.0))
        self.sigma_xy_floor = float(self._param("sigma_xy_floor", 0.02))
        self.vtg_yaw_var_max = float(self._param("vtg_yaw_var_max", 1.0))
        self.vtg_speed_var_max = float(self._param("vtg_speed_var_max", 100.0))
        self.wheel_speed_var = float(self._param("wheel_speed_var", 1.0e-4))
        self.imu_dropout_warn = float(self._param("imu_dropout_warn", 1.0))

        # Last GPS timestamp, for dropout warnings
        self._last_gps_t = None
        self._imu_seen = False
        self._last_imu_stamp_s = None

        self.create_subscription(Odometry, "gps", self.gps_callback, 10)
        self.create_subscription(Imu, "imu", self._imu_cb, 5)
        self.create_subscription(
            Float32, "e_comms/kart_speed_m_per_s", self._wheel_speed_cb, 5
        )

        # Per-GPS-event EKF snapshot: prior (IMU+wheel only) vs posterior
        # (after GPS fold-in) plus the raw measurement, for offline
        # EKF-vs-GPS innovation analysis.
        self.gps_event_pub = self.create_publisher(
            Float32MultiArray, "localization/gps_event", 10,
        )
        self._gps_event_seq = 0

        # 1 Hz watchdog
        self.create_timer(1.0, self._imu_watchdog)

    def _param(self, name, default):
        """Return the parameter value if declared, else `default`."""
        try:
            val = self.get_parameter(name).value
        except Exception:
            return default
        return default if val is None else val

    def _now_s(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

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
        # Leave z/roll/pitch (indices 14, 21, 28) at 0 unobserved.
        return cov

    def _wheel_speed_cb(self, msg: Float32):
        if not self.ekf.initialized:
            return
        self.ekf.update_speed(float(msg.data), self.wheel_speed_var)

    def _imu_cb(self, msg: Imu):
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        if not self._imu_seen:
            self._imu_seen = True
            self._last_imu_stamp_s = stamp_s
            return

        if not self.ekf.initialized:
            self._last_imu_stamp_s = stamp_s
            return

        dt = stamp_s - self._last_imu_stamp_s
        self._last_imu_stamp_s = stamp_s
        # Float rounding can push an exact 0.1 s gap to 0.1 + 9e-10
        # accept up to 0.1 s + 1 ns so a worst-case 10 Hz IMU still predicts.
        if dt <= 0.0 or dt > 0.1 + 1e-9:
            self.logger.warning(
                f"IMU dt out of range: {dt:.3f}s",
                throttle_duration_sec=2.0,
            )
            return

        accel_x = float(msg.linear_acceleration.x)
        accel_var = float(msg.linear_acceleration_covariance[0])
        omega_z = float(msg.angular_velocity.z)
        omega_var = float(msg.angular_velocity_covariance[8])

        self.ekf.predict(dt, omega_z, accel_x, omega_var, accel_var)

        px, py, yaw, v = self.ekf.x
        cov = self._odom_cov_from_P(self.ekf.P)
        self._publish_odom(px, py, yaw, v, cov=cov, speed_var=float(self.ekf.P[3, 3]))

        if self._last_gps_t is not None:
            gap = self._now_s() - self._last_gps_t
            if gap > self.gps_dropout_warn:
                self.logger.warning(
                    f"GPS dropout: {gap:.1f}s since last fix",
                    throttle_duration_sec=2.0,
                )

    def _imu_watchdog(self):
        if self._last_imu_stamp_s is None:
            return
        gap = self._now_s() - self._last_imu_stamp_s
        if gap > self.imu_dropout_warn:
            self.logger.warning(
                f"IMU dropout: {gap:.1f}s since last sample",
                throttle_duration_sec=2.0,
            )

    def gps_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        x, y = float(p.x), float(p.y)
        t = self._now_s()

        sigma_xx = float(msg.pose.covariance[0])
        sigma_yy = float(msg.pose.covariance[7])
        floor = self.sigma_xy_floor ** 2
        sigma_xx = max(sigma_xx, floor)
        sigma_yy = max(sigma_yy, floor)
        R_xy = np.diag([sigma_xx, sigma_yy])

        # VTG-derived course + speed travel inside the GPS Odometry:
        # orientation carries yaw (ENU, rad) and twist.linear.x carries
        # forward speed (m/s). Validity is signalled by the cov entries
        # the gps_node leaves them at 1e6 when no usable VTG arrived.
        q = msg.pose.pose.orientation
        yaw_meas = 2.0 * math.atan2(float(q.z), float(q.w))
        var_yaw = float(msg.pose.covariance[35])
        v_meas = float(msg.twist.twist.linear.x)
        var_v = float(msg.twist.covariance[0])
        have_yaw = var_yaw < self.vtg_yaw_var_max
        have_speed = var_v < self.vtg_speed_var_max
        # VTG track is course-over-ground = direction of motion. If the kart
        # is rolling in reverse (wheel speed negative), COG points 180° from
        # the body yaw — flip it before folding into the EKF. We trust the
        # signed VESC speed first; fall back to the EKF's own v if VTG speed
        # is unavailable but the filter already has a confident estimate.
        if have_yaw:
            v_for_sign = v_meas if have_speed else (
                float(self.ekf.x[3]) if self.ekf.initialized else 0.0
            )
            if v_for_sign < 0.0:
                yaw_meas = _wrap(yaw_meas + math.pi)

        if not self.ekf.initialized:
            if not self._imu_seen:
                # Wait for IMU before seeding the filter the predict path
                return
            # Seed position from the first fix; if VTG yaw/speed aren't
            # trusted, init them at 0 with large variance and let updates
            # pull them in.
            yaw0 = yaw_meas if have_yaw else 0.0
            var_yaw0 = var_yaw if have_yaw else (math.pi ** 2)
            v0 = v_meas if have_speed else 0.0
            var_v0 = var_v if have_speed else 1.0
            P0 = np.diag([sigma_xx, sigma_yy, var_yaw0, var_v0])
            self.ekf.reset(x, y, yaw0, v0, P=P0)
            self.logger.info(
                f"EKF init: pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw0):.1f} deg "
                f"v={v0:.2f} m/s (vtg_yaw={'y' if have_yaw else 'n'}, vtg_v={'y' if have_speed else 'n'})"
            )
            self._last_gps_t = t
            return

        # Already initialized: run corrections then publish.
        # Snapshot the EKF state (IMU+wheel predict only) just before GPS is
        # folded in, and the posterior after, so /localization/gps_event lets
        # offline analysis see how far IMU+wheel dead-reckoning had drifted.
        prior = self.ekf.x.copy()
        self.ekf.update_gps_xy(x, y, R_xy)
        if have_yaw:
            self.ekf.update_heading(yaw_meas, var_yaw)
        if have_speed:
            self.ekf.update_speed(v_meas, var_v)
        post = self.ekf.x.copy()
        self._last_gps_t = t

        px, py, yaw, v = self.ekf.x
        cov = self._odom_cov_from_P(self.ekf.P)
        self._publish_odom(px, py, yaw, v, cov=cov, speed_var=float(self.ekf.P[3, 3]))

        self._gps_event_seq += 1
        self.gps_event_pub.publish(Float32MultiArray(data=[
            float(self._gps_event_seq),
            float(x), float(y), float(yaw_meas), float(v_meas),
            1.0 if have_yaw else 0.0,
            1.0 if have_speed else 0.0,
            float(prior[0]), float(prior[1]),
            float(prior[2]), float(prior[3]),
            float(post[0]), float(post[1]),
            float(post[2]), float(post[3]),
        ]))

    def _publish_odom(self, x: float, y: float, yaw: float, speed: float,
                      cov=None, speed_var=None):
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
        if speed_var is not None:
            twist_cov = [0.0] * 36
            twist_cov[0] = float(speed_var)
            odom.twist.covariance = twist_cov
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