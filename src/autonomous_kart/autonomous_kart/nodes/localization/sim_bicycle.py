"""
Kinematic bicycle model. Pure math, no ROS dependencies. Mostly borrowed from NAV2

State: (x, y, yaw, speed)
Input: (target_mps, steer_deg)
"""
import math


class BicycleModel:
    def __init__(
        self,
        wheelbase_m: float,
        v_max_mps: float,
        steer_max_deg: float,
        accel_tau: float = 0.5,
        brake_tau: float = 0.3,
        steer_slop_deg: float = 0.0,
    ):
        self.wheelbase = wheelbase_m
        self.v_max = v_max_mps
        self.steer_max_rad = math.radians(steer_max_deg)
        self.accel_tau = accel_tau
        self.brake_tau = brake_tau
        # Half-width of the steering deadzone (backlash). Actual wheel angle
        # only updates when the command moves more than this past it.
        self.steer_slop_rad = math.radians(max(0.0, steer_slop_deg))
        self.delta_actual_rad = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0

    def reset(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = 0.0
        self.delta_actual_rad = 0.0

    def step(self, target_mps: float, steer_deg: float, dt: float):
        """Advance one timestep. Returns (x, y, yaw, speed)."""
        target_v = max(0.0, min(self.v_max, target_mps))

        # 1st-order lag
        tau = self.accel_tau if target_v >= self.speed else self.brake_tau
        alpha = dt / (tau + dt)
        self.speed += alpha * (target_v - self.speed)

        # Clamp commanded steering
        delta_cmd = math.radians(steer_deg)
        delta_cmd = max(-self.steer_max_rad, min(self.steer_max_rad, delta_cmd))

        # Mechanical slop / backlash: the actual wheel angle only changes
        # when the command moves past the deadzone band around it.
        slop = self.steer_slop_rad
        if delta_cmd > self.delta_actual_rad + slop:
            self.delta_actual_rad = delta_cmd - slop
        elif delta_cmd < self.delta_actual_rad - slop:
            self.delta_actual_rad = delta_cmd + slop
        delta = self.delta_actual_rad

        # Bicycle kinematics
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        if self.wheelbase > 1e-6:
            self.yaw += (self.speed * math.tan(delta) / self.wheelbase) * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        return self.x, self.y, self.yaw, self.speed