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
from autonomous_kart.nodes.pathfinder.planners.mpc_residual import ResidualLearner
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

        # Manual-mode state. /manual_commands is cached here and re-published
        # on cmd_drive at system_frequency by _manual_tick. If no command has
        # arrived within manual_timeout_s the tick falls back to [0, 0] for
        # safety, so a dropped HTTP / ROS connection brings the kart to rest.
        self._manual_speed_mps = 0.0  # m/s
        self._manual_steer_deg = 0.0  # degrees
        self._last_manual_ns = 0
        self.manual_timeout_s = self._param("manual_timeout_s", 0.5, float)
        self.manual_speed_max = self.kart.v_max_mps
        self.manual_steer_max = self.kart.steer_max_deg

        # Racing line
        self.line_path = self._param("line_path", "", str)
        self.racing_line: List[Tuple[float, ...]] = self._load_line_csv(self.line_path)

        # Cached inputs to planners
        self.state = self._param("system_state", "IDLE", str)
        self.pose_ready = False
        self.current_xy: Tuple[float, float] = (0.0, 0.0)
        self.current_yaw = 0.0
        self.current_speed_mps = 0.0
        self._latest_track_angles: Optional[Tuple[float, ...]] = None
        # Raw GPS pose cache, passed through to MPC for offline EKF-vs-GPS diff.
        self.gps_xy: Tuple[float, float] = (math.nan, math.nan)
        self.gps_yaw = math.nan
        self.gps_speed_mps = math.nan

        # Shared residual learner — survives planner / line swaps so training
        # state persists across mode changes.
        residual_params = {
            k: p.value
            for k, p in self.get_parameters_by_prefix("mpc.residual").items()
        }
        self.shared_residual = ResidualLearner(residual_params, 1.0 / self.system_frequency)

        # All planners are constructed; `active_planner_name` selects which
        # one drives cmd_drive. MPC still ticks every frame for telemetry.
        self.active_planner_name = self._param("planner", "pure_pursuit", str)
        if self.active_planner_name not in PLANNERS:
            raise ValueError(
                f"Unknown planner '{self.active_planner_name}'. Available: {list(PLANNERS)}"
            )
        self.planners: dict = {}
        if self.racing_line:
            self.planners = self._build_planners()
            self.logger.info(f"Pathfinder using planner '{self.active_planner_name}'")
        else:
            self.logger.error(
                "Pathfinder disabled: racing line is empty (planners not constructed)"
            )

        # Safety wrapper (single chokepoint for every planner)
        safety_params = {
            k: p.value for k, p in self.get_parameters_by_prefix("safety").items()
        }
        self.safety = SafetyChecker(safety_params)

        # Subscriptions
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Odometry, "gps", self._on_gps, 10)
        self.create_subscription(Float32MultiArray, "track_angles", self._on_track_angles, 5)
        self.create_subscription(String, "system_state", self.update_state, 10)
        self.create_subscription(Float32MultiArray, "manual_commands", self.manual_loop, 5)
        self.create_subscription(String, "mpc/residual_mode", self._on_residual_mode, 1)
        self.create_subscription(String, "pathfinder/planner", self._on_planner_swap, 1)
        self.create_subscription(String, "pathfinder/line_path", self._on_line_swap, 1)

        # Publishers
        self.drive_publisher = self.create_publisher(Float32MultiArray, "cmd_drive", 5)
        self.metrics_publisher = self.create_publisher(Float32MultiArray, "pathfinder_params", 5)
        self.dynamic_line_pub = self.create_publisher(String, "pathfinder/dynamic_line", 1)

        # Timers
        self.create_timer(1.0 / self.system_frequency, self._autonomous_tick)
        self.create_timer(1.0 / self.system_frequency, self._manual_tick)
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

    def _on_gps(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gps_xy = (float(p.x), float(p.y))
        self.gps_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.gps_speed_mps = float(msg.twist.twist.linear.x)

    def _on_track_angles(self, msg: Float32MultiArray):
        self._latest_track_angles = tuple(msg.data) if msg.data else None

    def _on_residual_mode(self, msg: String):
        mode = (msg.data or "").strip().lower()
        if mode not in ("off", "shadow", "apply"):
            self.logger.warning(f"mpc/residual_mode: ignoring '{msg.data}'")
            return
        self.shared_residual.mode = mode
        self.logger.info(f"residual mode -> {mode}")

    def _on_planner_swap(self, msg: String):
        name = (msg.data or "").strip().lower()
        if name not in PLANNERS:
            self.logger.warning(f"pathfinder/planner: unknown '{msg.data}'")
            return
        if name not in self.planners:
            self.logger.warning(f"pathfinder/planner: '{name}' not constructed")
            return
        self.active_planner_name = name
        self.logger.info(f"active planner -> {name}")

    def _on_line_swap(self, msg: String):
        path = (msg.data or "").strip()
        new_line = self._load_line_csv(path)
        if not new_line:
            self.logger.warning(f"pathfinder/line_path: could not load '{path}'")
            return
        self.line_path = path
        self.racing_line = new_line
        # Rebuild planners on the new line; shared residual persists.
        self.planners = self._build_planners()
        self.logger.info(f"line -> {path} ({len(new_line)} pts)")

    def update_state(self, msg: String):
        if msg.data != self.state:
            # Any state transition: zero throttle immediately and drop the
            # cached manual command so a stale value can't replay on re-entry.
            self._manual_speed_mps = 0.0
            self._manual_steer_deg = 0.0
            self._last_manual_ns = 0
            self.drive_publisher.publish(Float32MultiArray(data=[0.0, 0.0]))
        if msg.data not in [s.value for s in STATES]:
            self.logger.error(f"State {msg.data} not recognized")
            return
        self.state = msg.data

    # Autonomous tick

    def _autonomous_tick(self):
        if not self.pose_ready:
            return
        active = self.planners.get(self.active_planner_name)
        if active is None:
            return
        inputs = PlannerInputs(
            pose_xy=self.current_xy,
            yaw_rad=self.current_yaw,
            speed_mps=self.current_speed_mps,
            track_angles=self._latest_track_angles,
            now_ns=self.get_clock().now().nanoseconds,
            gps_xy=self.gps_xy,
            gps_yaw_rad=self.gps_yaw,
            gps_speed_mps=self.gps_speed_mps,
        )

        # Always run MPC.plan() so mpc/status keeps flowing for telemetry, regardless of state or active planner
        mpc = self.planners.get(MPCPlanner.name)
        mpc_proposed = None
        if mpc is not None:
            try:
                mpc_proposed = mpc.plan(inputs)
            except Exception:
                self.logger.error(f"MPC telemetry plan error:\n{traceback.format_exc()}")

        if self.state != STATES.AUTONOMOUS.value:
            return

        if active is mpc:
            proposed = mpc_proposed
        else:
            try:
                proposed = active.plan(inputs)
            except Exception:
                self.logger.error(f"Planner error:\n{traceback.format_exc()}")
                return
        if proposed is None:
            return
        safe = self.safety.check(proposed, inputs)
        motor_mps, steering_deg = safe
        self.cmd_count += 1
        self.drive_publisher.publish(
            Float32MultiArray(data=[float(motor_mps), float(steering_deg)])
        )
        # Keep training the shared residual under non-MPC planners
        if self.active_planner_name != MPCPlanner.name and mpc is not None:
            try:
                mpc.train_step(
                    motor_mps, steering_deg,
                    self.current_xy[0], self.current_xy[1],
                    self.current_yaw, self.current_speed_mps,
                    inputs.now_ns,
                )
            except Exception:
                self.logger.error(f"Residual train_step error:\n{traceback.format_exc()}")

    # Manual mode
    #
    # manual_loop just caches the most recent /manual_commands message; the
    # actual cmd_drive publish happens on a 60 Hz timer (_manual_tick) so the
    # rate matches autonomous mode and a stale-command timeout can step in if
    # the client stops sending.

    def manual_loop(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.logger.error(
                f"manual_commands payload too short: {list(msg.data)}"
            )
            return
        self._manual_speed_mps = float(msg.data[0])
        self._manual_steer_deg = float(msg.data[1])
        self._last_manual_ns = self.get_clock().now().nanoseconds

    def _manual_tick(self):
        if self.state != STATES.MANUAL.value:
            return
        now_ns = self.get_clock().now().nanoseconds
        fresh = (
                self._last_manual_ns > 0
                and (now_ns - self._last_manual_ns) <= int(self.manual_timeout_s * 1e9)
        )
        if fresh:
            speed_mps = max(
                0.0, min(self.manual_speed_max, self._manual_speed_mps)
            )
            steer_deg = max(
                -self.manual_steer_max,
                min(self.manual_steer_max, self._manual_steer_deg),
            )
        else:
            speed_mps = 0.0
            steer_deg = 0.0

        self.cmd_count += 1
        self.drive_publisher.publish(
            Float32MultiArray(data=[float(speed_mps), float(steer_deg)])
        )
        self.metrics_publisher.publish(
            Float32MultiArray(
                data=[
                    float(speed_mps),
                    float(steer_deg),
                    1.0 if fresh else 0.0,
                ]
            )
        )

    # Telemetry

    def publish_dynamic_line(self):
        active = self.planners.get(self.active_planner_name)
        state = active.dynamic_line_state() if active is not None else None
        if state is None:
            payload = {"active": False, "strategy": None, "merge_idx": -1, "points": []}
        else:
            payload = state
        self.dynamic_line_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    # Helpers

    def _load_line_csv(self, path: str) -> List[Tuple[float, ...]]:
        rows: List[Tuple[float, ...]] = []
        try:
            with open(path, "r") as f:
                for line in f:
                    try:
                        row = tuple(float(x) for x in line.strip().split(","))
                    except ValueError:
                        continue
                    rows.append(row)
        except OSError as e:
            self.logger.error(f"Racing line not loadable at {path}: {e}")
        return rows

    def _build_planners(self) -> dict:
        """Construct every registered planner. MPC gets the shared residual
        so training survives planner / line swaps."""
        out: dict = {}
        for name, cls in PLANNERS.items():
            planner_params = {
                k: p.value for k, p in self.get_parameters_by_prefix(name).items()
            }
            kwargs = dict(logger=self.logger, node=self)
            if cls is MPCPlanner:
                kwargs["residual"] = self.shared_residual
            try:
                out[name] = cls(planner_params, self.kart, self.racing_line, **kwargs)
            except Exception as e:
                self.logger.error(f"Failed to construct planner '{name}': {e}")
        return out

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
