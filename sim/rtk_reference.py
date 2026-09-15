"""Smoothed-RTK reference trajectory used as truth for the sensor-noise fits.

Every sensor model in ``sim/sensor_noise.py`` is fitted against this. RTK-fixed
positions carry a reported 3 cm sigma at 10 Hz, an order of magnitude better
than any other stream on the kart, so a constant-acceleration smoother over
them gives a position, velocity, course and yaw-rate reference good enough to
treat as truth when grading the IMU, the wheel encoder and VTG.

The online EKF is NOT a valid reference for this: it disagrees with raw GPS by
0.1 to 0.6 m, which is larger than most of the errors being measured.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# A fix whose innovation exceeds this many chi-square-2 units is dropped rather
# than allowed to drag the reference. RTK glitches of tens of metres occur at
# roughly 0.04% of samples while the receiver still reports a 3 cm sigma.
DEFAULT_NIS_GATE = 100.0


@dataclass
class Reference:
    """Smoothed trajectory on the GPS timestamp grid."""
    t: np.ndarray           # (N,) seconds, GPS header stamps
    x: np.ndarray           # (N,) m
    y: np.ndarray           # (N,) m
    vx: np.ndarray          # (N,) m/s
    vy: np.ndarray          # (N,) m/s
    ax: np.ndarray          # (N,) m/s^2
    ay: np.ndarray          # (N,) m/s^2
    speed: np.ndarray       # (N,) m/s, ground speed
    course: np.ndarray      # (N,) rad, ENU heading of the velocity vector
    psi_dot: np.ndarray     # (N,) rad/s, yaw rate of the velocity vector
    accel_long: np.ndarray  # (N,) m/s^2, along-track acceleration
    rejected: np.ndarray    # (N,) bool, fixes the NIS gate dropped
    nis: np.ndarray         # (N,) normalised innovation squared, forward pass
    jerk_density: float     # m^2/s^5, the q that was used


def _transition(dt: float) -> np.ndarray:
    F = np.eye(6)
    for b in (0, 3):
        F[b, b + 1] = dt
        F[b, b + 2] = 0.5 * dt * dt
        F[b + 1, b + 2] = dt
    return F


def _process_cov(dt: float, q: float) -> np.ndarray:
    block = q * np.array([
        [dt ** 5 / 20.0, dt ** 4 / 8.0, dt ** 3 / 6.0],
        [dt ** 4 / 8.0, dt ** 3 / 3.0, dt ** 2 / 2.0],
        [dt ** 3 / 6.0, dt ** 2 / 2.0, dt],
    ])
    Q = np.zeros((6, 6))
    Q[:3, :3] = block
    Q[3:, 3:] = block
    return Q


def smooth_xy(
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    var_x: np.ndarray,
    var_y: np.ndarray,
    q: float,
    nis_gate: float = DEFAULT_NIS_GATE,
) -> dict:
    """Fixed-interval RTS smoother on ``[x, vx, ax, y, vy, ay]``.

    ``q`` is the jerk spectral density in m^2/s^5. Per-sample measurement
    variances come from the receiver's own covariance, so a float fix is
    down-weighted rather than discarded.
    """
    n = len(t)
    m = np.zeros((n, 6))
    P = np.zeros((n, 6, 6))
    m_pred = np.zeros((n, 6))
    P_pred = np.zeros((n, 6, 6))
    H = np.zeros((2, 6))
    H[0, 0] = 1.0
    H[1, 3] = 1.0

    xk = np.array([x[0], 0.0, 0.0, y[0], 0.0, 0.0])
    Pk = np.diag([var_x[0], 25.0, 25.0, var_y[0], 25.0, 25.0])
    nis = np.full(n, np.nan)
    rejected = np.zeros(n, dtype=bool)

    for k in range(n):
        if k > 0:
            dt = float(t[k] - t[k - 1])
            F = _transition(dt)
            xk = F @ xk
            Pk = F @ Pk @ F.T + _process_cov(dt, q)
        m_pred[k] = xk
        P_pred[k] = Pk

        R = np.diag([var_x[k], var_y[k]])
        S = H @ Pk @ H.T + R
        innovation = np.array([x[k], y[k]]) - H @ xk
        nis[k] = float(innovation @ np.linalg.solve(S, innovation))
        if k > 5 and nis[k] > nis_gate:
            rejected[k] = True
            m[k] = xk
            P[k] = Pk
            continue

        K = Pk @ H.T @ np.linalg.inv(S)
        xk = xk + K @ innovation
        Pk = (np.eye(6) - K @ H) @ Pk
        Pk = 0.5 * (Pk + Pk.T)
        m[k] = xk
        P[k] = Pk

    xs = m.copy()
    for k in range(n - 2, -1, -1):
        F = _transition(float(t[k + 1] - t[k]))
        C = P[k] @ F.T @ np.linalg.inv(P_pred[k + 1])
        xs[k] = m[k] + C @ (xs[k + 1] - m_pred[k + 1])

    return {"state": xs, "nis": nis, "rejected": rejected}


def fit_jerk_density(
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    var_x: np.ndarray,
    var_y: np.ndarray,
    grid: np.ndarray | None = None,
) -> float:
    """Pick the jerk density that makes the forward filter consistent.

    Median NIS is matched to the median of chi-square with two degrees of
    freedom rather than the mean, because a handful of metre-scale RTK glitches
    otherwise drags q up until the smoother simply follows the noise.
    """
    grid = np.logspace(-1.0, 3.0, 21) if grid is None else grid
    best_q, best_err = float(grid[0]), float("inf")
    for q in grid:
        out = smooth_xy(t, x, y, var_x, var_y, float(q))
        keep = ~out["rejected"][5:]
        nis = out["nis"][5:][keep]
        err = abs(float(np.nanmedian(nis)) - 1.386)
        if err < best_err:
            best_q, best_err = float(q), err
    return best_q


def reference(
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    var_x: np.ndarray,
    var_y: np.ndarray,
    q: float | None = None,
) -> Reference:
    """Smoothed position, velocity, course and yaw rate over one moving stint."""
    if q is None:
        q = fit_jerk_density(t, x, y, var_x, var_y)
    out = smooth_xy(t, x, y, var_x, var_y, q)
    s = out["state"]
    vx, vy, ax, ay = s[:, 1], s[:, 4], s[:, 2], s[:, 5]
    speed = np.hypot(vx, vy)
    return Reference(
        t=np.asarray(t, dtype=float), x=s[:, 0], y=s[:, 3],
        vx=vx, vy=vy, ax=ax, ay=ay,
        speed=speed,
        course=np.arctan2(vy, vx),
        psi_dot=(vx * ay - vy * ax) / np.maximum(speed * speed, 1e-6),
        accel_long=(vx * ax + vy * ay) / np.maximum(speed, 1e-3),
        rejected=out["rejected"], nis=out["nis"], jerk_density=float(q),
    )
