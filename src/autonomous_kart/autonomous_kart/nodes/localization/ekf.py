"""
Extended Kalman Filter for localization. Pure math, no ROS

State: x = [px, py, yaw, v]   (m, m, rad, m/s)
Control input (for predict): steer angle (rad), clamped to ±steer_max.
Measurements:
    - GPS xy (always on a GPS fix)
    - GPS-derived heading pseudo-measurement (gated by motion)
    - GPS-derived speed pseudo-measurement (gated by motion)
"""
from __future__ import annotations

import math

import numpy as np


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LocalizationEKF:
    def __init__(self, wheelbase_m: float, steer_max_rad: float):
        self.L = float(wheelbase_m)
        self.steer_max = float(steer_max_rad)

        # State [px, py, yaw, v]
        self.x = np.zeros(4, dtype=np.float64)
        # Huge covariance until reset() is called
        self.P = np.eye(4, dtype=np.float64) * 1e6
        self.initialized = False

        # Process-noise spectral densities (overridable by the node from YAML)
        self.q_pos = 0.01       # m²/s
        self.q_yaw_rate = 0.05  # rad²/s
        self.q_accel = 4.0      # (m/s²)²·s

    def reset(self, px: float, py: float, yaw: float, v: float,
              P: np.ndarray | None = None) -> None:
        self.x[:] = (px, py, _wrap(yaw), v)
        if P is not None:
            assert P.shape == (4, 4)
            self.P = np.array(P, dtype=np.float64, copy=True)
        else:
            self.P = np.diag([0.25, 0.25, 0.25, 1.0])
        self.initialized = True

    def predict(self, dt: float, steer_rad: float) -> None:
        """Advance the EKF mean and covariance by dt seconds.

        Process model (continuous):
            ṗx  = v·cos(yaw)
            ṗy  = v·sin(yaw)
            ẏaw = v·tan(δ) / L         (δ clamped to ±steer_max, L safe-guarded)
            v̇   = 0                    (random walk; noise applied via Q)
        """
        px, py, yaw, v = self.x
        delta = max(-self.steer_max, min(self.steer_max, float(steer_rad)))

        # --- Mean ---
        sin_y, cos_y = math.sin(yaw), math.cos(yaw)
        self.x[0] = px + v * cos_y * dt
        self.x[1] = py + v * sin_y * dt
        if self.L > 1e-6:
            self.x[2] = _wrap(yaw + v * math.tan(delta) * dt / self.L)
        # v unchanged.

        # --- Covariance: P = F P Fᵀ + Q ---
        # F = ∂f/∂x evaluated at the *previous* mean.
        F = np.eye(4, dtype=np.float64)
        F[0, 2] = -v * sin_y * dt
        F[0, 3] = cos_y * dt
        F[1, 2] = v * cos_y * dt
        F[1, 3] = sin_y * dt
        if self.L > 1e-6:
            F[2, 3] = math.tan(delta) * dt / self.L

        Q = np.diag([
            self.q_pos * dt,
            self.q_pos * dt,
            self.q_yaw_rate * dt,
            self.q_accel * dt,
        ])

        self.P = F @ self.P @ F.T + Q
        # Re-symmetrize to fight numerical drift.
        self.P = 0.5 * (self.P + self.P.T)

    def update_gps_xy(self, x: float, y: float, R_xy: np.ndarray) -> None:
        """Standard EKF update with measurement z = [x_gps, y_gps]."""
        H = np.zeros((2, 4), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        z = np.array([float(x), float(y)], dtype=np.float64)
        self._linear_update(H, z, np.asarray(R_xy, dtype=np.float64))

    def update_heading(self, yaw_meas: float, var: float) -> None:
        """Pseudo-measurement: z = yaw, derived from GPS displacement when moving."""
        H = np.zeros((1, 4), dtype=np.float64)
        H[0, 2] = 1.0
        z = np.array([float(yaw_meas)], dtype=np.float64)
        innovation = np.array([_wrap(float(yaw_meas) - self.x[2])], dtype=np.float64)
        R = np.array([[float(var)]], dtype=np.float64)
        self._linear_update(H, z, R, innovation=innovation)

    def update_speed(self, v_meas: float, var: float) -> None:
        """Pseudo-measurement: z = v, derived from GPS displacement / dt."""
        H = np.zeros((1, 4), dtype=np.float64)
        H[0, 3] = 1.0
        z = np.array([float(v_meas)], dtype=np.float64)
        R = np.array([[float(var)]], dtype=np.float64)
        self._linear_update(H, z, R)

    def _linear_update(self, H: np.ndarray, z: np.ndarray, R: np.ndarray,
                       innovation: np.ndarray | None = None) -> None:
        """Joseph-form update for numerical stability.

        If `innovation` is None it is computed as z - H @ x.
        Callers (e.g. heading update) may supply a pre-wrapped innovation.
        """
        if innovation is None:
            innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[2] = _wrap(self.x[2])
        I = np.eye(self.P.shape[0])
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
