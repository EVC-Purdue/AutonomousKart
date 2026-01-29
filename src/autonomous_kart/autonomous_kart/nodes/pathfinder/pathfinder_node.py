import math
import traceback
from typing import Tuple, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String
from autonomous_kart.nodes.pathfinder.pathfinder import pathfinder

from autonomous_kart.nodes.master.master_node import STATES

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

        # self.pose_ready = False
        self.pose_ready = True  # Dummy until localization works
        self.current_xy = (0.0, 0.0)
        self.current_yaw = 0.0
        self.current_speed_mps = 0.0

        # self.localization_sub = self.create_subscription(Odometry, "odom", self.localization, 10)

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

            target_xy, speed_ref_pct = self.pick_lookahead_point(lookahead_m)
            if (self.cmd_count % 50) == 0:
                self.get_logger().debug(
                    f"pose=({self.current_xy[0]:.2f},{self.current_xy[1]:.2f}) yaw={self.current_yaw:.2f} "
                    f"target=({target_xy[0]:.2f},{target_xy[1]:.2f}) "
                    f"lookahead={lookahead_m:.2f} closest_idx={self.closest_idx} "
                    f"speed_ref_pct={speed_ref_pct:.2f} speed_pct={speed_pct:.2f}"
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
            self.logger.info(f"Steering/Motor output: {steering_pct}, {motor_pct}")
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
        """Updates speed & steering params from given values, speed & steering are absolute"""
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
            self.get_logger().info(
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

    def pick_lookahead_point(self, lookahead_m: float) -> Tuple[Tuple[float, float], float]:
        """
        Returns:
          target_xy: (x_m, y_m) lookahead point on the racing line in map frame
          speed_ref_pct: desired speed at that point as fraction of v_max_mps in [0..1]

        Core idea (Pure Pursuit / Nav2 RPP): pick a point L meters ahead along the path.
        Copied from Nav2
        """
        line = self.racing_line
        n = len(line)
        if n == 0:
            return self.current_xy, 0.0

        if lookahead_m <= 0.0:
            lookahead_m = 0.01

        cx, cy = self.current_xy

        # Find closest index (windowed)
        # Tune these bounds
        start = max(0, self.closest_idx - 50)
        end = min(n, self.closest_idx + 250)

        best_i = self.closest_idx
        best_d2 = float("inf")

        # If closest_idx is uninitialized or line is small, widen to full scan once.
        if best_i < 0 or best_i >= n:
            start, end = 0, n
            best_i = 0

        for i in range(start, end):
            row = line[i]
            x = row[1]  # x_m
            y = row[2]  # y_m
            dx = x - cx
            dy = y - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.closest_idx = best_i

        # Use arc-length s_m to pick the lookahead point
        s0 = float(line[best_i][0])  # s_m at closest point
        s_target = s0 + float(lookahead_m)

        # Decide if this is a closed track. Most racing lines are one closed lap.
        s_end = float(line[-1][0])
        closed = s_end > 0.0 and (math.hypot(line[0][1] - line[-1][1], line[0][2] - line[-1][2]) < 2.0)

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
            j = best_i
            while j < n and float(line[j][0]) < s_target:
                j += 1
            if j >= n:
                j = n - 1  # clamp to end if not wrapping

        tx = float(line[j][1])
        ty = float(line[j][2])
        vx_mps = float(line[j][5])  # vx_mps column
        speed_ref_pct = self._clamp01(vx_mps / self.v_max_mps) if self.v_max_mps > 1e-6 else 0.0

        return (tx, ty), speed_ref_pct


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
