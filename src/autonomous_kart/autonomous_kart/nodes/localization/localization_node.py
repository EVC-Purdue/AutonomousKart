"""
Localization node — publishes Odometry on /odom.

Sim mode:  Integrates bicycle kinematics from cmd_vel + cmd_turn.
Real mode: TODO: fuse GPS RTK + IMU via Kalman filter / factor graph.
"""
import math
import traceback

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

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

        self.gps_subscriber = None
        self.imu_subscriber = None

        self._last_gps_xy = None
        self._last_gps_t = None
        self._yaw_est = 0.0
        self._speed_est = 0.0
        self._yaw_min_dist = 0.0

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

        self.create_subscription(Float32, "cmd_vel", self._cb_vel, 5)
        self.create_subscription(Float32, "cmd_turn", self._cb_turn, 5)
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

    def _cb_vel(self, msg: Float32):
        self.cmd_motor = msg.data

    def _cb_turn(self, msg: Float32):
        self.cmd_steer = msg.data

    def _sim_step(self):
        x, y, yaw, speed = self.model.step(self.cmd_motor, self.cmd_steer, self.dt)
        self._publish_odom(x, y, yaw, speed)


    def _init_real(self):
        # Passes GPS data
        # TODO: EKF with updates
        self.gps_subscriber = self.create_subscription(
            Odometry, "gps", self.gps_callback, 10
        )
        self._last_gps_xy = None
        self._last_gps_t = None
        self._yaw_est = 0.0
        self._speed_est = 0.0
        self._yaw_min_dist = 0.3  # Hotfix, TODO: Update

    def gps_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        x, y = float(p.x), float(p.y)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_gps_xy is not None:
            dx = x - self._last_gps_xy[0]
            dy = y - self._last_gps_xy[1]
            dist = math.hypot(dx, dy)
            dt = max(t - self._last_gps_t, 1e-3)
            if dist > self._yaw_min_dist:
                self._yaw_est = math.atan2(dy, dx)
            # Low-pass speed so the HUD doesn't twitch
            self._speed_est = 0.7 * self._speed_est + 0.3 * (dist / dt)

        self._last_gps_xy = (x, y)
        self._last_gps_t = t
        self._publish_odom(x, y, self._yaw_est, self._speed_est)


    def _publish_odom(self, x: float, y: float, yaw: float, speed: float):
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