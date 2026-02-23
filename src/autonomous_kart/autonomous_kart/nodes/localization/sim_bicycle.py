"""
Kinematic bicycle model. Pure math, no ROS dependencies. Mostly borrowed from NAV2

State: (x, y, yaw, speed)
Input: (motor_pct 0-100, steer_deg)
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
    ):
        self.wheelbase = wheelbase_m
        self.v_max = v_max_mps
        self.steer_max_rad = math.radians(steer_max_deg)
        self.accel_tau = accel_tau
        self.brake_tau = brake_tau

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0

    def reset(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = 0.0

    def step(self, motor_pct: float, steer_deg: float, dt: float):
        """Advance one timestep. Returns (x, y, yaw, speed)."""
        # Target speed from motor %
        target_v = max(0.0, min(self.v_max, (motor_pct / 100.0) * self.v_max))

        # 1st-order lag
        tau = self.accel_tau if target_v >= self.speed else self.brake_tau
        alpha = dt / (tau + dt)
        self.speed += alpha * (target_v - self.speed)

        # Clamp steering
        delta = math.radians(steer_deg)
        delta = max(-self.steer_max_rad, min(self.steer_max_rad, delta))

        # Bicycle kinematics
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        if self.wheelbase > 1e-6:
            self.yaw += (self.speed * math.tan(delta) / self.wheelbase) * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        return self.x, self.y, self.yaw, self.speed