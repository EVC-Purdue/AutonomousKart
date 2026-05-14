import json
import math
import traceback
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from autonomous_kart.nodes.master.master_node import STATES
from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, PlannerInputs
from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner
from autonomous_kart.nodes.pathfinder.planners.opencv import OpenCVPlanner
from autonomous_kart.nodes.pathfinder.planners.pure_pursuit import PurePursuitPlanner
from autonomous_kart.nodes.pathfinder.safety_checker import SafetyChecker

# Adding a planner: import the class above, add an entry here.
PLANNERS = {
    OpenCVPlanner.name: OpenCVPlanner,
    PurePursuitPlanner.name: PurePursuitPlanner,
    MPCPlanner.name: MPCPlanner,
}


class PathfinderNode(Node):
    def __init__(self):
        super().__init__(
            "PathfinderNode",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now().nanoseconds

        self.system_frequency = self._param("system_frequency", 60.0, float)
        self.sim_mode = self._param("simulation_mode", False, bool)

        self.kart = KartConstants(
            v_max_mps=self._param("v_max_mps", 20.0, float),
            wheelbase_m=self._param("wheelbase_m", 1.05, float),
            steer_max_deg=self._param("steer_max_deg", 25.0, float),
            steer_rate_max_degps=self._param("steer_rate_max_degps", 180.0, float),
            a_max_mps2=self._param("a_max_mps2", 2.0, float),
            a_min_mps2=self._param("a_min_mps2", -3.0, float),
            a_lat_max_mps2=self._param("a_lat_max_mps2", 4.0, float),
        )

        # Manual-mode state
        self.steering = 0.0
        self.speed = 0.0
        self.expected_steering = 0.0
        self.expected_speed = 0.0
        self.max_speed = 100.0  # full % of v_max_mps
        self.max_steering = 100.0  # full % of steer_max_deg (each side)
        dt = 1.0 / self.system_frequency
        # Per-tick deltas expressed as %-of-max units (same unit as self.speed).
        self.acceleration = (
                self.kart.a_max_mps2 / self.kart.v_max_mps * dt * 100.0
        )
        self.steering_accel = (
                self.kart.steer_rate_max_degps / self.kart.steer_max_deg * dt * 100.0
        )

        # Racing line
        self.line_path = self._param("line_path", "", str)
        self.racing_line: List[Tuple[float, ...]] = []
        try:
            with open(self.line_path, "r") as f:
                for line in f:
                    try:
                        row = tuple(float(x) for x in line.strip().split(","))
                    except ValueError:
                        continue
                    self.racing_line.append(row)
        except OSError as e:
            self.logger.error(f"Racing line not loadable at {self.line_path}: {e}")

        # Cached inputs to planners
        self.state = self._param("system_state", "IDLE", str)
        self.pose_ready = False
        self.current_xy: Tuple[float, float] = (0.0, 0.0)
        self.current_yaw = 0.0
        self.current_speed_mps = 0.0
        self._latest_track_angles: Optional[Tuple[float, ...]] = None

        # Active planner
        planner_name = self._param("planner", "pure_pursuit", str)
        if planner_name not in PLANNERS:
            raise ValueError(
                f"Unknown planner '{planner_name}'. Available: {list(PLANNERS)}"
            )
        self.planner = None
        if self.racing_line:
            planner_params = {
                k: p.value for k, p in self.get_parameters_by_prefix(planner_name).items()
            }
            self.planner = PLANNERS[planner_name](
                planner_params, self.kart, self.racing_line, logger=self.logger, node=self
            )
            self.logger.info(f"Pathfinder using planner '{planner_name}'")
        else:
            self.logger.error(
                "Pathfinder disabled: racing line is empty (planner not constructed)"
            )

        # Safety wrapper (single chokepoint for every planner)
        safety_params = {
            k: p.value for k, p in self.get_parameters_by_prefix("safety").items()
        }
        self.safety = SafetyChecker(safety_params)

        # Subscriptions
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Float32MultiArray, "track_angles", self._on_track_angles, 5)
        self.create_subscription(String, "system_state", self.update_state, 10)
        self.create_subscription(Float32MultiArray, "manual_commands", self.manual_loop, 5)

        # Publishers
        self.drive_publisher = self.create_publisher(Float32MultiArray, "cmd_drive", 5)
        self.metrics_publisher = self.create_publisher(Float32MultiArray, "pathfinder_params", 5)
        self.dynamic_line_pub = self.create_publisher(String, "pathfinder/dynamic_line", 1)

        # Timers
        self.create_timer(1.0 / self.system_frequency, self._autonomous_tick)
        self.create_timer(0.5, self.publish_dynamic_line)  # 2 Hz telemetry
        self.create_timer(5.0, self.log_command_rate)

        self.logger.info("Initialize Pathfinder Node")

    # Subscriptions

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_xy = (float(p.x), float(p.y))
        self.current_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_speed_mps = float(msg.twist.twist.linear.x)
        self.pose_ready = True

    def _on_track_angles(self, msg: Float32MultiArray):
        self._latest_track_angles = tuple(msg.data) if msg.data else None

    def update_state(self, msg: String):
        if msg.data != self.state:
            self.speed = 0.0
            self.expected_speed = 0.0
            steering_deg = (self.steering / 100.0) * self.kart.steer_max_deg
            self.drive_publisher.publish(Float32MultiArray(data=[0.0, steering_deg]))
        if msg.data not in [s.value for s in STATES]:
            self.logger.error(f"State {msg.data} not recognized")
            return
        self.state = msg.data

    # Autonomous tick

    def _autonomous_tick(self):
        if self.state != STATES.AUTONOMOUS.value:
            return
        if not self.pose_ready or self.planner is None:
            return
        inputs = PlannerInputs(
            pose_xy=self.current_xy,
            yaw_rad=self.current_yaw,
            speed_mps=self.current_speed_mps,
            track_angles=self._latest_track_angles,
            now_ns=self.get_clock().now().nanoseconds,
        )
        try:
            proposed = self.planner.plan(inputs)
        except Exception:
            self.logger.error(f"Planner error:\n{traceback.format_exc()}")
            return
        if proposed is None:
            return
        safe = self.safety.check(proposed, inputs)
        motor_pct, steering_pct = safe
        self.cmd_count += 1
        self.drive_publisher.publish(
            Float32MultiArray(data=[float(motor_pct), float(steering_pct)])
        )

    # Manual mode

    def manual_loop(self, msg: Float32MultiArray):
        self.cmd_count += 1
        if self.state == STATES.MANUAL.value:
            if len(msg.data) < 2:
                self.logger.error(
                    f"Message length of manual loop is {len(msg.data)}: {msg.data}"
                )
            motor_speed, steering = msg.data[0], msg.data[1]
            self.set_manual_speeds(motor_speed, steering)

    def set_manual_speeds(self, speed: float = 0.0, steering: float = 0.0):
        target_speed = min(self.max_speed, max(0.0, speed))
        target_steering = min(self.max_steering, max(-1 * self.max_steering, steering))

        tolerance = 0.1
        speed_ok = abs(target_speed - self.speed) > tolerance
        steering_ok = abs(target_steering - self.steering) > tolerance

        while speed_ok or steering_ok:
            self.manual_speed_helper(target_speed, target_steering)

            speed_ok = abs(target_speed - self.speed) > tolerance
            steering_ok = abs(target_steering - self.steering) > tolerance

    def manual_speed_helper(self, speed: float = 0.0, steering: float = 0.0):
        """Updates absolute speed & steering params from given values"""
        self.cmd_count += 1

        if speed < 0:
            speed = 0.0

        if speed > self.max_speed:
            speed = self.max_speed
        if abs(steering) > self.max_steering:
            steering = self.max_steering if steering > 0 else -1 * self.max_steering

        self.expected_speed = speed
        self.expected_steering = steering

        d_speed = self.expected_speed - self.speed
        d_steer = self.expected_steering - self.steering

        # Update speed by difference or acceleration, whichever abs value is smaller
        if d_speed != 0:
            self.speed += (
                d_speed
                if abs(d_speed) < self.acceleration
                else (d_speed / abs(d_speed)) * self.acceleration
            )
        if d_steer != 0:
            self.steering += (
                d_steer
                if abs(d_steer) < self.steering_accel
                else (d_steer / abs(d_steer)) * self.steering_accel
            )
        self.publish_commands()

    def publish_commands(self):
        """Publishes commands from params to steering & motor"""
        steering_deg = (self.steering / 100.0) * self.kart.steer_max_deg
        self.drive_publisher.publish(Float32MultiArray(data=[self.speed, steering_deg]))

        time = self.get_clock().now().nanoseconds

        self.metrics_publisher.publish(
            Float32MultiArray(
                data=[
                    self.speed,
                    self.expected_speed,
                    self.acceleration,
                    self.max_speed,
                    self.steering,
                    self.expected_steering,
                    self.steering_accel,
                    self.max_steering,
                    time - self.last_log_time,
                ]
            )
        )
        self.last_log_time = time

    # Telemetry

    def publish_dynamic_line(self):
        state = self.planner.dynamic_line_state()
        if state is None:
            payload = {"active": False, "strategy": None, "merge_idx": -1, "points": []}
        else:
            payload = state
        self.dynamic_line_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def log_command_rate(self):
        current_time = self.get_clock().now().nanoseconds
        elapsed = (current_time - self.last_log_time) / 1e9

        if elapsed > 0:
            avg_rate = self.cmd_count / elapsed
            self.logger.debug(
                f"Average command rate: {avg_rate:.2f} commands/sec "
                f"(Total: {self.cmd_count} in {elapsed:.1f}s)"
            )

        self.cmd_count = 0
        self.last_log_time = current_time

    # Helpers

    def _param(self, name, default, cast=None):
        """Read a ROS param with a code-side default if the override is missing
        or NOT_SET. Defaults are a resilience layer; yaml is still canonical."""
        try:
            v = self.get_parameter(name).value
        except Exception:
            v = None
        if v is None:
            v = default
        if cast is not None and v is not None:
            try:
                v = cast(v)
            except (TypeError, ValueError):
                v = default
        return v

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)

    node = PathfinderNode()

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
