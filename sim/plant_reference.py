"""Fused 60 Hz reference trajectory used as truth for the learned plant.

RTK position is the only sub-centimetre sensor but it arrives at 10 Hz. The
sign-corrected gyro and the scale-corrected wheel speed arrive at 100 Hz, so
fusing all three gives heading and speed that are genuinely observed between
GPS fixes. Sensor scales, biases and noise come from sim/model/sensors.json.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sim.sensor_noise import SensorNoiseModel

PLANT_HZ = 60.0

# Random-walk process noise on the two driven states. Yaw acceleration and
# longitudinal jerk are what the kart can change fastest; these are loose
# enough that the measurements dominate and tight enough to smooth.
Q_OMEGA = 4.0     # (rad/s^2)^2 per second
Q_ACCEL = 25.0    # (m/s^3)^2 per second

# Floor of process noise on every state, and a ridge on the smoother gain.
# Q carrying entries only for omega and a lets the position block collapse
# toward singular after a 3 cm fix, which is what makes the backward pass
# explode. Both are far below any real uncertainty, so they bias nothing.
Q_FLOOR = 1e-9
JITTER = 1e-9

# The receiver emits an occasional single corrupt fix: 3 of the 33131 fixes in
# the September bags land hundreds of kilometres away, each reporting the same
# 3 cm sigma every good RTK fix reports. Weighting by the reported covariance
# is what makes the smoother trustworthy everywhere else, so the filter has no
# statistical reason to doubt one, and a single 889 km innovation ran a stint
# to 82396 m/s. They arrive alone, with the fixes either side of them where the
# kart actually was, so a fix no neighbour can reach at a physical speed is
# dropped before the filter ever sees it.
GPS_MAX_SPEED_MPS = 30.0

IX, IY, IPSI, IV, IOMEGA, IA = range(6)


@dataclass
class FusedReference:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    v: np.ndarray
    omega: np.ndarray
    accel: np.ndarray


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def reachable_fixes(gps_t, gps_x, gps_y) -> np.ndarray:
    """Mask of fixes reachable from a neighbour at `GPS_MAX_SPEED_MPS`.

    An interior fix survives if either neighbour can reach it, so a spike has
    to be isolated to be dropped and a genuine gap in the fix stream costs
    nothing. An endpoint is judged against the one neighbour it has.
    """
    n = len(gps_t)
    keep = np.ones(n, dtype=bool)
    if n < 2:
        return keep
    step = np.hypot(np.diff(np.asarray(gps_x, dtype=float)),
                    np.diff(np.asarray(gps_y, dtype=float)))
    jump = step / np.maximum(np.diff(np.asarray(gps_t, dtype=float)), 1e-6)
    jump = jump > GPS_MAX_SPEED_MPS       # jump[i] spans fixes i and i+1
    keep[0] = not jump[0]
    keep[-1] = not jump[-1]
    if n > 2:
        keep[1:-1] = ~(jump[:-1] & jump[1:])
    return keep


def _predict(state, P, dt):
    x, y, psi, v, omega, a = state
    F = np.eye(6)
    F[IX, IPSI] = -v * math.sin(psi) * dt
    F[IX, IV] = math.cos(psi) * dt
    F[IY, IPSI] = v * math.cos(psi) * dt
    F[IY, IV] = math.sin(psi) * dt
    F[IPSI, IOMEGA] = dt
    F[IV, IA] = dt

    nxt = np.array([
        x + v * math.cos(psi) * dt,
        y + v * math.sin(psi) * dt,
        _wrap(psi + omega * dt),
        v + a * dt,
        omega,
        a,
    ])
    # A floor on every diagonal, not just the two driven states: with zeros on
    # the position block it collapses toward singular after a 3 cm fix, which is
    # what makes the backward pass explode.
    Q = np.eye(6) * (Q_FLOOR * dt)
    Q[IOMEGA, IOMEGA] = Q_OMEGA * dt
    Q[IA, IA] = Q_ACCEL * dt
    return nxt, F @ P @ F.T + Q


def _update(state, P, H, z, r, wrap_index=None):
    S = float(H @ P @ H.T) + r
    K = (P @ H.T) / S
    innovation = z - float(H @ state)
    if wrap_index is not None:
        innovation = _wrap(innovation)
    state = state + K * innovation
    state[IPSI] = _wrap(state[IPSI])
    P = (np.eye(6) - np.outer(K, H)) @ P
    return state, 0.5 * (P + P.T)


def fuse(gps_t, gps_x, gps_y, gps_var_x, course_t, course,
         gyro_t, gyro_z, wheel_t, wheel_v,
         noise: SensorNoiseModel, hz: float = PLANT_HZ,
         gps_var_y=None) -> FusedReference:
    """Extended RTS smoother over [x, y, psi, v, omega, a] on a uniform grid.

    Each fix is weighted by its own reported covariance. A stint holds fixes
    of several qualities -- `moving_stints` keeps a brief RTK float inside a
    stint on the understanding that the smoother down-weights it -- and a
    single median variance for the stint is exactly what fails to do that: a
    20 cm float fix then pulls as hard as a 2 cm fixed one.
    """
    var_x = np.asarray(gps_var_x, dtype=float)
    var_y = var_x if gps_var_y is None else np.asarray(gps_var_y, dtype=float)
    keep = reachable_fixes(gps_t, gps_x, gps_y)
    gps_t, gps_x, gps_y = gps_t[keep], gps_x[keep], gps_y[keep]
    var_x, var_y = var_x[keep], var_y[keep]

    dt = 1.0 / hz
    t0 = max(float(gps_t[0]), float(gyro_t[0]), float(wheel_t[0]))
    t1 = min(float(gps_t[-1]), float(gyro_t[-1]), float(wheel_t[-1]))
    grid = np.arange(t0, t1, dt)
    n = len(grid)

    # The initial covariance is a prior over the whole stint, so it takes the
    # stint's typical fix quality rather than the first fix's.
    r_xy = float(np.median(var_x))
    r_course = noise.vtg_course.sigma_core ** 2
    r_gyro = noise.gyro_z.sigma_core ** 2
    r_wheel = noise.wheel_speed.sigma_core ** 2

    j0 = int(np.argmin(np.abs(gps_t - grid[0])))
    psi0 = float(np.interp(grid[0], course_t, np.unwrap(course)))
    state = np.array([gps_x[j0], gps_y[j0], _wrap(psi0),
                      float(np.interp(grid[0], wheel_t, wheel_v)) / noise.wheel_speed.scale,
                      0.0, 0.0])
    P = np.diag([r_xy, r_xy, 0.25, 1.0, 1.0, 25.0])

    m_pred = np.zeros((n, 6)); P_pred = np.zeros((n, 6, 6))
    m_post = np.zeros((n, 6)); P_post = np.zeros((n, 6, 6))

    H_x = np.zeros(6); H_x[IX] = 1.0
    H_y = np.zeros(6); H_y[IY] = 1.0
    H_psi = np.zeros(6); H_psi[IPSI] = 1.0
    H_omega = np.zeros(6); H_omega[IOMEGA] = noise.gyro_z.scale
    H_v = np.zeros(6); H_v[IV] = noise.wheel_speed.scale

    for k, tk in enumerate(grid):
        if k > 0:
            state, P = _predict(state, P, dt)
        m_pred[k] = state; P_pred[k] = P
        lo, hi = tk - dt / 2.0, tk + dt / 2.0

        # Lags from sensors.json are fitted by best_lag() in sensor_noise.py,
        # which scores a candidate lag by correlating v_ref(t_meas - lag)
        # against v_meas(t_meas); the fitted lag is the one where that lines
        # up, so a sample stamped T satisfies v_meas(T) ~ v_ref(T - lag), i.e.
        # it describes the truth at T - lag. To place that sample at the grid
        # time g it actually describes, we need T - lag = g, i.e. T = g + lag,
        # so a grid window [lo, hi] should pull samples with T in
        # [lo + lag, hi + lag] - equivalently, samples whose (T - lag) falls
        # in [lo, hi].
        sel = np.searchsorted(gps_t, [lo, hi])
        for i in range(sel[0], sel[1]):
            state, P = _update(state, P, H_x, float(gps_x[i]), float(var_x[i]))
            state, P = _update(state, P, H_y, float(gps_y[i]), float(var_y[i]))
        sel = np.searchsorted(course_t, [lo, hi])
        for i in range(sel[0], sel[1]):
            state, P = _update(state, P, H_psi, float(course[i]), r_course, wrap_index=IPSI)
        sel = np.searchsorted(gyro_t - noise.gyro_z.lag_s, [lo, hi])
        for i in range(sel[0], sel[1]):
            z = float(gyro_z[i]) - noise.gyro_z.bias
            state, P = _update(state, P, H_omega, z, r_gyro)
        sel = np.searchsorted(wheel_t - noise.wheel_speed.lag_s, [lo, hi])
        for i in range(sel[0], sel[1]):
            state, P = _update(state, P, H_v, float(wheel_v[i]) - noise.wheel_speed.bias, r_wheel)
        m_post[k] = state; P_post[k] = P

    smoothed = m_post.copy()
    for k in range(n - 2, -1, -1):
        F = np.eye(6)
        x, y, psi, v, omega, a = m_post[k]
        F[IX, IPSI] = -v * math.sin(psi) * dt
        F[IX, IV] = math.cos(psi) * dt
        F[IY, IPSI] = v * math.cos(psi) * dt
        F[IY, IV] = math.sin(psi) * dt
        F[IPSI, IOMEGA] = dt
        F[IV, IA] = dt
        # solve, not inv: P_pred goes ill-conditioned wherever a 3 cm RTK fix
        # has just collapsed the position block, and inv() there returns garbage
        # that the backward recursion then compounds. JITTER keeps it positive
        # definite; on 2026-09-13 bags the plain form diverged to v = -82 km/s.
        A = P_pred[k + 1] + JITTER * np.eye(6)
        C = np.linalg.solve(A.T, (P_post[k] @ F.T).T).T
        delta = smoothed[k + 1] - m_pred[k + 1]
        delta[IPSI] = _wrap(delta[IPSI])
        smoothed[k] = m_post[k] + C @ delta
        smoothed[k, IPSI] = _wrap(smoothed[k, IPSI])

    return FusedReference(t=grid, x=smoothed[:, IX], y=smoothed[:, IY],
                          psi=smoothed[:, IPSI], v=smoothed[:, IV],
                          omega=smoothed[:, IOMEGA], accel=smoothed[:, IA])
