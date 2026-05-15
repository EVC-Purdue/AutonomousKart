"""
Extended Kalman Filter for kart localization. Pure math, no ROS.

State vector x:
    x[0] = px   x position (m)
    x[1] = py   y position (m)
    x[2] = yaw  heading (rad)
    x[3] = v    forward speed (m/s)

Predict rolls x forward using bicycle kinematics driven by commanded
steering. Update folds in GPS xy directly, plus heading and speed
derived from consecutive GPS fixes (pseudo-measurements, gated by motion).
"""
from __future__ import annotations

import math

import numpy as np


def _wrap(a: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class LocalizationEKF:
    def __init__(
        self,
        wheelbase_m: float,
        steer_max_rad: float,
        pos_noise: float,
        yaw_noise: float,
        accel_noise: float,
    ):
        self.L = float(wheelbase_m)
        self.steer_max = float(steer_max_rad)

        # Process-noise spectral densities (per second).
        self.pos_noise = float(pos_noise)
        self.yaw_noise = float(yaw_noise)
        self.accel_noise = float(accel_noise)

        self.x = np.zeros(4)
        # Big P -> we know nothing about state.
        self.P = np.eye(4) * 1e6
        self.initialized = False

    def reset(
        self,
        px: float,
        py: float,
        yaw: float,
        v: float,
        P: np.ndarray | None = None,
    ) -> None:
        """Seed the filter with a known state from a confident GPS fix."""
        self.x[:] = (px, py, _wrap(yaw), v)
        if P is not None:
            if P.shape != (4, 4):
                raise ValueError(f"P must be 4x4, got {P.shape}")
            self.P = np.array(P, dtype=float)
        else:
            self.P = np.diag([0.25, 0.25, 0.25, 1.0])
        self.initialized = True

    def predict(self, dt: float, steer_rad: float) -> None:
        """Roll the state forward dt seconds using bicycle kinematics."""
        px, py, yaw, v = self.x
        delta = max(-self.steer_max, min(self.steer_max, float(steer_rad)))
        sin_y, cos_y = math.sin(yaw), math.cos(yaw)

        # Advance the mean. v is unchanged only process noise drives it.
        self.x[0] = px + v * cos_y * dt
        self.x[1] = py + v * sin_y * dt
        if self.L > 1e-6:
            self.x[2] = _wrap(yaw + v * math.tan(delta) * dt / self.L)

        # F = how each state derivative depends on each state.
        F = np.eye(4)
        F[0, 2] = -v * sin_y * dt
        F[0, 3] = cos_y * dt
        F[1, 2] = v * cos_y * dt
        F[1, 3] = sin_y * dt
        if self.L > 1e-6:
            F[2, 3] = math.tan(delta) * dt / self.L

        # Q = uncertainty we accept this step (scales with dt).
        Q = np.diag([self.pos_noise, self.pos_noise, self.yaw_noise, self.accel_noise]) * dt

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)  # Force symmetry against float drift.

    def update_gps_xy(self, x: float, y: float, R_xy: np.ndarray) -> None:
        """Fold in a GPS position fix (m); R_xy is the 2x2 GPS covariance."""
        H = np.zeros((2, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        z = np.array([float(x), float(y)])
        self._linear_update(H, z, np.asarray(R_xy, dtype=float))

    def update_heading(self, yaw_meas: float, var: float) -> None:
        """Fold in a heading observation (rad), e.g. derived from GPS displacement."""
        H = np.zeros((1, 4))
        H[0, 2] = 1.0
        z = np.array([float(yaw_meas)])
        # Yaw is circular wrap the innovation.
        innovation = np.array([_wrap(float(yaw_meas) - self.x[2])])
        self._linear_update(H, z, np.array([[float(var)]]), innovation=innovation)

    def update_speed(self, v_meas: float, var: float) -> None:
        """Fold in a speed observation (m/s)."""
        H = np.zeros((1, 4))
        H[0, 3] = 1.0
        z = np.array([float(v_meas)])
        self._linear_update(H, z, np.array([[float(var)]]))

    def _linear_update(
        self,
        H: np.ndarray,
        z: np.ndarray,
        R: np.ndarray,
        innovation: np.ndarray | None = None,
    ) -> None:
        """Fold a linear measurement into the state and shrink P accordingly."""
        if innovation is None:
            innovation = z - H @ self.x

        # S = how much we expect this measurement to disagree with our prediction.
        S = H @ self.P @ H.T + R
        # K = how much to trust the measurement vs the prior
        K = np.linalg.solve(S, H @ self.P).T

        # Pull the state toward the measurement.
        self.x = self.x + K @ innovation
        self.x[2] = _wrap(self.x[2])

        # Joseph form, stays numerically stable across many updates.
        I = np.eye(self.P.shape[0])
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
