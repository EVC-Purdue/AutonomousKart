import math
import traceback
from typing import Tuple, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String
from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder

from autonomous_kart.nodes.master.master_node import STATES

from autonomous_kart.nodes.pathfinder.dynamic_line import DynamicLineManager, KartState
from autonomous_kart.nodes.pathfinder.strategies.rejoin import RejoinStrategy

class PathfinderNode(Node):
    def __init__(self):
        super().__init__(
            "PathfinderNode",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.logger = self.get_logger()
        self.angles = None

        self.cmd_count = 0
        self.last_log_time = self.get_clock().now().nanoseconds

        self.system_frequency = self.get_parameter("system_frequency").value
        self.sim_mode = self.get_parameter("simulation_mode").value

        # Manual params
        self.steering = 0.0
        self.speed = 0.0

        self.expected_steering = 0.0
        self.expected_speed = 0.0

        self.max_speed = self.get_parameter("max_speed").value
        self.acceleration = self.get_parameter("acceleration").value
        self.max_steering = self.get_parameter("max_steering").value
        self.steering_accel = self.get_parameter("steering_accel").value

        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.v_max_mps = float(self.get_parameter("v_max_mps").value)
        self.steer_max_deg = float(self.get_parameter("steer_max_deg").value)

        self.use_velocity_scaled_lookahead = bool(self.get_parameter("use_velocity_scaled_lookahead").value)
        self.lookahead_time_s = float(self.get_parameter("lookahead_time_s").value)
        self.min_lookahead_m = float(self.get_parameter("min_lookahead_m").value)
        self.max_lookahead_m = float(self.get_parameter("max_lookahead_m").value)

        self.use_curvature_regulation = bool(self.get_parameter("use_curvature_regulation").value)
        self.min_radius_m = float(self.get_parameter("min_radius_m").value)
        self.min_reg_speed_pct = float(self.get_parameter("min_reg_speed_pct").value)

        self.approach_dist_m = float(self.get_parameter("approach_dist_m").value)
        self.min_approach_speed_pct = float(self.get_parameter("min_approach_speed_pct").value)

        self.search_window = int(self.get_parameter("search_window").value)
        self.initial_sync_done = bool(self.get_parameter("initial_sync_done").value)
        self.max_resync_dist = float(self.get_parameter("max_resync_dist").value)
        self.max_closed_dist = float(self.get_parameter("max_closed_dist").value)

        self.pose_ready = False
        # self.pose_ready = True  # Dummy until localization works
        self.current_xy = (0.0, 0.0)
        self.current_yaw = 0.0
        self.current_speed_mps = 0.0

        self.localization_sub = self.create_subscription(Odometry, "odom", self.localization, 10)

        self.dynamic_line_pub = self.create_publisher(
            String, "pathfinder/dynamic_line", 1
        )
        self.create_timer(0.5, self.publish_dynamic_line)  # 2 Hz

        self.closest_idx = 0


        self.metrics_publisher = self.create_publisher(
            Float32MultiArray, "pathfinder_params", 5
        )

        # Timer to log average every 5 seconds
        self.create_timer(5.0, self.log_command_rate)

        self.line_path = self.get_parameter("line_path").value
        self.racing_line: List[Tuple[float, float, float]] = []
        with open(self.line_path, 'r') as f:
            for line in f:
                try:
                    p1 = tuple(float(x) for x in line.strip().split(','))
                except ValueError:
                    continue
                self.racing_line.append(p1)
        self.line_manager = DynamicLineManager(self.racing_line, logger=self.logger)
        self.line_manager.register(RejoinStrategy(
            cte_activate=3.0,
            cte_deactivate=1.0,
            merge_lookahead_m=15.0,
            min_turning_radius=self.min_radius_m,
        ))
        # Future: self.line_manager.register(ObstacleAvoidanceStrategy(...))

        self.state = self.get_parameter("system_state").value
        self.state_subscriber = self.create_subscription(String, "system_state", self.update_state, 10)

        # Subscriber to opencv pathfinder for angles
        self.opencv_pathfinder_subscriber = self.create_subscription(
            Float32MultiArray, "track_angles", self.autonomous_control_loop, 5
        )

        self.manual_subscriber = self.create_subscription(
            Float32MultiArray,
            "manual_commands",
            self.manual_loop,
            5
        )

        # Publisher to motor
        self.motor_publisher = self.create_publisher(Float32, "cmd_vel", 5)

        # Publisher to steering
        self.steering_publisher = self.create_publisher(Float32, "cmd_turn", 5)

        self.logger.info("Initialize Pathfinder Node")

    def _yaw_from_quaternion(self, q) -> float:
        # yaw from quaternion (x,y,z,w)
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def localization(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_xy = (float(p.x), float(p.y))
        self.current_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_speed_mps = float(msg.twist.twist.linear.x)
        self.pose_ready = True

    def update_state(self, msg: String):
        if msg.data != self.state:
            # Set speed to zero on state change
            self.speed = 0.0
            self.expected_speed = 0.0
            self.motor_publisher.publish(Float32(data=0.0))
        if msg.data not in [s.value for s in STATES]:
            self.logger.error(f"State {msg.data} not recognized")
            return

        self.state = msg.data

    def autonomous_control_loop(self, msg: Float32MultiArray):
        """
        Calculate commands for steering and motor from opencv_pathfinder efficiently
        Part of hot loop so must be efficient
        :param msg: Float32MultiArray of [left angle from center to base of track from image, right angle ...]
        :return: Publishes commands to motor & steering
        """
        if not self.pose_ready:
            # Wait for localization
            return
        self.cmd_count += 1
        if self.state == STATES.AUTONOMOUS.value:
            # speed in % (0..1) for the controller
            speed_pct = (self.current_speed_mps / self.v_max_mps if self.v_max_mps > 1e-6 else 1.0)
            if speed_pct < 0.0:
                speed_pct = 0.0
            elif speed_pct > 1.0:
                speed_pct = 1.0

            # Choose lookahead distance like Nav2 "velocity-scaled lookahead"
            if self.use_velocity_scaled_lookahead:
                lookahead_m = self.current_speed_mps * self.lookahead_time_s
                if lookahead_m < self.min_lookahead_m:
                    lookahead_m = self.min_lookahead_m
                elif lookahead_m > self.max_lookahead_m:
                    lookahead_m = self.max_lookahead_m
            else:
                lookahead_m = self.min_lookahead_m

            # Keep the racing-line closest_idx fresh every tick so CTE reflects
            # actual proximity, even while a dynamic (rejoin) line is active.
            # Three-tier strategy:
            #   1. First autonomous tick after pose_ready: full O(n) search so
            #      we sync correctly regardless of spawn location.
            #   2. Localization jump / huge off-track excursion: full search
            #      again (windowed search would stay stuck in the old region).
            #   3. Normal operation: bounded forward search with wrap for the
            #      closed racing line.
            if self.racing_line:
                if not self.initial_sync_done:
                    self.closest_idx = self._full_nearest_idx(
                        self.racing_line, self.current_xy
                    )
                    self.initial_sync_done = True
                else:
                    if self.closest_idx < 0 or self.closest_idx >= len(self.racing_line):
                        self.closest_idx = 0
                    row = self.racing_line[self.closest_idx]
                    dx = self.current_xy[0] - float(row[1])
                    dy = self.current_xy[1] - float(row[2])
                    if math.hypot(dx, dy) > self.max_resync_dist:
                        self.closest_idx = self._full_nearest_idx(
                            self.racing_line, self.current_xy
                        )
                    else:
                        self.closest_idx = self._nearest_idx_forward(
                            self.racing_line,
                            self.current_xy,
                            self.closest_idx,
                            window=self.search_window,
                            allow_wrap=True,
                        )

            cte = self.line_manager.compute_cte(
                self.current_xy, self.racing_line, self.closest_idx
            )
            kart_state = KartState(
                xy=self.current_xy,
                yaw=self.current_yaw,
                speed_mps=self.current_speed_mps,
                closest_idx=self.closest_idx,
                cross_track_error=cte,
            )
            self.line_manager.update(kart_state)
            # Manager may snap closest_idx on deactivation.
            self.closest_idx = kart_state.closest_idx

            if self.line_manager.is_active:
                dyn_line, dyn_idx = self.line_manager.get_line_and_idx()
                if not dyn_line or dyn_idx < 0:
                    # get_line_and_idx() can fall back to the closed racing line
                    # Fallback to normal lookahead
                    target_xy, speed_ref_pct = self.pick_lookahead_point(
                        self.racing_line, self.closest_idx, lookahead_m
                    )
                else:
                    # Dynamic lines (e.g. Bezier rejoin) are short and never closed,
                    # so the forward search must NOT wrap — wrapping would let the
                    # search snap back to idx 0 and pick a point behind the kart.
                    dyn_idx = self._nearest_idx_forward(
                        dyn_line,
                        self.current_xy,
                        dyn_idx,
                        window=self.search_window,
                        allow_wrap=False,
                    )
                    target_xy, speed_ref_pct = self.pick_lookahead_point(
                        dyn_line, dyn_idx, lookahead_m
                    )
                    self.line_manager.set_dynamic_closest_idx(dyn_idx)
            else:
                target_xy, speed_ref_pct = self.pick_lookahead_point(
                    self.racing_line, self.closest_idx, lookahead_m
                )

            motor_pct, steering_pct = pathfinder(
                current_xy=self.current_xy,
                target_xy=target_xy,
                yaw_rad=self.current_yaw,
                speed_pct=speed_pct,

                wheelbase_m=self.wheelbase_m,
                v_max_mps=self.v_max_mps,
                steer_max_deg=self.steer_max_deg,

                desired_speed_pct=speed_ref_pct,
                use_velocity_scaled_lookahead=self.use_velocity_scaled_lookahead,
                lookahead_time_s=self.lookahead_time_s,
                min_lookahead_m=self.min_lookahead_m,
                max_lookahead_m=self.max_lookahead_m,

                use_curvature_regulation=self.use_curvature_regulation,
                min_radius_m=self.min_radius_m,
                min_reg_speed_pct=self.min_reg_speed_pct,

                approach_dist_m=self.approach_dist_m,
                min_approach_speed_pct=self.min_approach_speed_pct,
            )
            self.logger.debug(f"Steering/Motor output: {steering_pct}, {motor_pct}")
            self.steering_publisher.publish(Float32(data=steering_pct))
            self.motor_publisher.publish(Float32(data=motor_pct))

    def manual_loop(self, msg: Float32MultiArray):
        self.cmd_count += 1
        if self.state == STATES.MANUAL.value:
            if len(msg.data) < 2:
                self.logger.error(f"Message length of manual loop is {len(msg.data)}: {msg.data}")
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
        self.steering_publisher.publish(Float32(data=self.steering))
        self.motor_publisher.publish(Float32(data=self.speed))

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

    def log_command_rate(self):
        """Log average commands per second every 5 seconds"""
        current_time = self.get_clock().now().nanoseconds
        elapsed = (current_time - self.last_log_time) / 1e9

        if elapsed > 0:
            avg_rate = self.cmd_count / elapsed
            self.get_logger().debug(
                f"Average command rate: {avg_rate:.2f} commands/sec "
                f"(Total: {self.cmd_count} in {elapsed:.1f}s)"
            )

        # Reset counters
        self.cmd_count = 0
        self.last_log_time = current_time

    def _clamp01(self, x: float) -> float:
        if x < 0.0:
            return 0.0
        if x > 1.0:
            return 1.0
        return x

    @staticmethod
    def _nearest_idx_forward(
        line: list,
        xy: Tuple[float, float],
        start_idx: int,
        window: int,
        allow_wrap: bool,
    ) -> int:
        """Bounded forward nearest-index search.

        With allow_wrap=True the index wraps modulo len(line), which is only
        safe on closed tracks. With allow_wrap=False the scan stops at the end
        of the line so a short open path (e.g. a Bezier rejoin) cannot snap
        backwards to a point behind the current progress.
        """
        n = len(line)
        if n == 0:
            return 0
        if start_idx < 0 or start_idx >= n:
            start_idx = 0
        if allow_wrap:
            max_offset = min(window, n)
        else:
            max_offset = min(window, n - start_idx)
        best_i = start_idx
        best_d2 = float("inf")
        cx, cy = xy
        for offset in range(max_offset):
            if allow_wrap:
                i = (start_idx + offset) % n
            else:
                i = start_idx + offset
            dx = line[i][1] - cx
            dy = line[i][2] - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    @staticmethod
    def _full_nearest_idx(line: list, xy: Tuple[float, float]) -> int:
        """O(n) full-line nearest search. Used for initial sync and resync
        when the kart has moved far from the previously tracked index."""
        n = len(line)
        if n == 0:
            return 0
        cx, cy = xy
        best_i = 0
        best_d2 = float("inf")
        for i in range(n):
            dx = line[i][1] - cx
            dy = line[i][2] - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def pick_lookahead_point(
        self,
        line: list,
        closest_idx: int,
        lookahead_m: float,
    ) -> Tuple[Tuple[float, float], float]:
        """Pure-function lookahead pick from a given line and closest index.

        Callers are responsible for keeping closest_idx fresh via the
        `_nearest_idx_forward` / `_full_nearest_idx` helpers. Does not read
        or mutate self.racing_line or self.closest_idx — this decoupling lets
        the autonomous loop run the same logic on the racing line and on a
        dynamic (rejoin) line without a fragile swap-and-restore pattern.

        Returns:
          target_xy: (x_m, y_m) lookahead point on `line` in map frame
          speed_ref_pct: desired speed at that point as fraction of
            v_max_mps in [0..1]

        Core idea (Pure Pursuit / Nav2 RPP): pick a point L meters ahead along
        the path.
        """
        n = len(line)
        if n == 0:
            return self.current_xy, 0.0

        if lookahead_m <= 0.0:
            lookahead_m = 0.01

        if closest_idx < 0 or closest_idx >= n:
            closest_idx = 0

        # Use arc-length s_m to pick the lookahead point
        s0 = float(line[closest_idx][0])  # s_m at closest point
        s_target = s0 + float(lookahead_m)

        # Decide if this is a closed track. Most racing lines are one closed lap.
        s_end = float(line[-1][0])
        closed = s_end > 0.0 and (
            math.hypot(line[0][1] - line[-1][1], line[0][2] - line[-1][2]) <= self.max_closed_dist
        )

        if closed and s_target > s_end:
            s_target -= s_end  # wrap around to start

            # search from start of array
            j = 0
            while j < n and float(line[j][0]) < s_target:
                j += 1
            if j >= n:
                j = n - 1
        else:
            # search forward from closest index (monotonic s)
            j = closest_idx
            while j < n and float(line[j][0]) < s_target:
                j += 1
            if j >= n:
                j = n - 1  # clamp to end if not wrapping

        tx = float(line[j][1])
        ty = float(line[j][2])
        vx_mps = float(line[j][5]) if len(line[j]) > 5 else 0.0
        speed_ref_pct = self._clamp01(vx_mps / self.v_max_mps) if self.v_max_mps > 1e-6 else 0.0

        return (tx, ty), speed_ref_pct

    def publish_dynamic_line(self):
        import json
        if self.line_manager.is_active:
            line = self.line_manager.dynamic_line
            payload = {
                "active": True,
                "strategy": type(self.line_manager.active_strategy).__name__,
                "merge_idx": int(self.line_manager.merge_idx),
                "points": [[float(p[1]), float(p[2])] for p in line],
            }
        else:
            payload = {"active": False, "strategy": None, "merge_idx": -1, "points": []}
        self.dynamic_line_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )


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
