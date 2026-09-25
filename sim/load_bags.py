"""Bag to aligned 60 Hz npz loader for the data-driven sim.

See docs/superpowers/specs/2026-05-18-data-driven-sim-design.md section 4.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

# Per-tick columns produced by the loader. Keep this list authoritative —
# downstream code reads `AlignedBag.field_names()`.
NUM_THROTTLE_HIST = 3
NUM_STEER_HIST = 4


@dataclass
class AlignedBag:
    t: np.ndarray              # (N,) seconds since bag start
    cmd_throttle: np.ndarray   # (N,) % (0..100)
    cmd_steer: np.ndarray      # (N,) deg
    cmd_throttle_hist: np.ndarray  # (N, NUM_THROTTLE_HIST)
    cmd_steer_hist: np.ndarray     # (N, NUM_STEER_HIST)
    v: np.ndarray              # (N,) m/s, low-pass filtered wheel speed
    psi_dot: np.ndarray        # (N,) rad/s, bias-corrected gyro_z in base_link FLU
    accel_x: np.ndarray        # (N,) m/s^2, forward accel in FLU; diagnostic only
    dv_dt: np.ndarray          # (N,) m/s^2 (central diff of v)
    state_mode: np.ndarray     # (N,) string-encoded state per CLAUDE.md STATES
    odom_x: np.ndarray         # (N,) m, validation truth
    odom_y: np.ndarray         # (N,) m
    odom_yaw: np.ndarray       # (N,) rad
    autonomous: np.ndarray     # (N,) bool — True iff state_mode == "AUTONOMOUS"
    track_angle_right: np.ndarray = None   # (N,) deg, from /track_angles
    track_angle_left: np.ndarray = None    # (N,) deg, from /track_angles
    mpc_d: np.ndarray = None               # (N,) m, /mpc/status idx 4
    mpc_psi_track: np.ndarray = None       # (N,) rad, /mpc/status idx 5
    mpc_cost: np.ndarray = None            # (N,) /mpc/status idx 10
    mpc_margin_min: np.ndarray = None      # (N,) /mpc/status idx 11

    def field_names(self) -> List[str]:
        return [f for f in self.__dataclass_fields__]

    def to_npz_dict(self) -> Dict[str, np.ndarray]:
        return {name: getattr(self, name) for name in self.field_names()}

    @classmethod
    def from_npz(cls, path: str) -> "AlignedBag":
        data = np.load(path, allow_pickle=True)  # allow_pickle needed for string arrays (state_mode)
        kwargs = {name: data[name] for name in cls.__dataclass_fields__ if name in data}
        n = kwargs["t"].shape[0] if "t" in kwargs else next(iter(kwargs.values())).shape[0]
        for name in cls.__dataclass_fields__:
            if name not in kwargs:
                kwargs[name] = np.full(n, np.nan)
        return cls(**kwargs)


def zoh_align(
    sample_t: np.ndarray,
    sample_v: np.ndarray,
    grid_t: np.ndarray,
) -> np.ndarray:
    """Zero-order hold from irregular (sample_t, sample_v) onto grid_t.

    For each g in grid_t, returns the value of the most-recent sample with
    sample_t <= g. Samples that pre-date the first grid point fill with the
    first available sample (no NaN gap at the start).
    """
    if sample_t.size == 0:
        raise ValueError("empty sample stream")
    if not np.all(np.diff(sample_t) >= 0):
        order = np.argsort(sample_t)
        sample_t = sample_t[order]
        sample_v = sample_v[order]
    idx = np.searchsorted(sample_t, grid_t, side="right") - 1
    idx = np.clip(idx, 0, sample_t.size - 1)
    return sample_v[idx]


def central_diff(x: np.ndarray, dt: float) -> np.ndarray:
    """Central difference with reflected endpoints (returns same shape as x)."""
    out = np.empty_like(x, dtype=np.float64)
    out[1:-1] = (x[2:] - x[:-2]) / (2.0 * dt)
    out[0] = (x[1] - x[0]) / dt
    out[-1] = (x[-1] - x[-2]) / dt
    return out


def make_history(values: np.ndarray, depth: int) -> np.ndarray:
    """Per-tick history block. Row i is [values[i-1], ..., values[i-depth]].
    For i < k, pads with values[0] (matches the online learner startup)."""
    n = values.size
    out = np.zeros((n, depth), dtype=values.dtype)
    for k in range(1, depth + 1):
        out[:, k - 1] = np.concatenate(
            [np.repeat(values[0], min(k, n)), values[:max(0, n - k)]]
        )
    return out


# Recorded /imu has already been rotated by imu_node, so nothing is re-applied
# to a bag taken after the 2026-09-14 mount fix. Bags before it carry a frame
# where the gyro z axis is negated and the accelerometer x and y are swapped:
# the old R_MOUNT had the chip a quarter turn out, and scaling the accelerometer
# by a negative default_g mirrored it against the gyro.
LEGACY_IMU_MOUNT = True


def imu_to_flu(gyro_xyz: np.ndarray, accel_xyz: np.ndarray,
               legacy: bool = LEGACY_IMU_MOUNT):
    """Published /imu -> (yaw rate, forward acceleration) in base_link FLU."""
    if legacy:
        return -gyro_xyz[:, 2], accel_xyz[:, 1]
    return gyro_xyz[:, 2], accel_xyz[:, 0]


def load_gyro_bias(cache_path: str) -> np.ndarray:
    """Read the gyro bias from imu_calibration.json. Returns (3,) zeros if
    missing — callers may choose to error instead."""
    import json
    import os
    if not os.path.isfile(cache_path):
        return np.zeros(3, dtype=np.float64)
    with open(cache_path) as f:
        data = json.load(f)
    return np.asarray(data.get("gyro_bias", [0.0, 0.0, 0.0]), dtype=np.float64)


TOPICS_NEEDED = {
    "/cmd_drive",
    "/imu",
    "/odom",
    "/e_comms/kart_speed_m_per_s",
    "/system_state",
}
# RL-residual extras (docs/rl_residual_plan.md). Not required — align_streams
# NaN-fills these when the stream is empty, which every bag is today (no
# historical bag has camera data; see scripts/bag_health_check.py).
OPTIONAL_TOPICS = {"/track_angles", "/mpc/status"}

# /mpc/status payload order (autonomous_kart/nodes/pathfinder/planners/mpc.py,
# the `plan()` status-publish block, 88 floats total). Keep in sync if that
# payload's shape ever changes.
MPC_STATUS_D_IDX = 4
MPC_STATUS_PSI_TRACK_IDX = 5
MPC_STATUS_COST_IDX = 10
MPC_STATUS_MARGIN_IDX = 11


def read_bag_streams(mcap_path: str, topics=None):
    """Return {topic: [(t_seconds, msg), ...]} from one mcap file."""
    from mcap_ros2.reader import read_ros2_messages
    topics = topics if topics is not None else (TOPICS_NEEDED | OPTIONAL_TOPICS)
    streams = {t: [] for t in topics}
    for evt in read_ros2_messages(mcap_path, topics=topics):
        t = evt.publish_time_ns * 1e-9
        streams[evt.channel.topic].append((t, evt.ros_msg))
    return streams


def _zoh_align_optional(stream, grid_t: np.ndarray, extractor, n_values: int = 1):
    """Like zoh_align, but for a possibly-empty optional stream — returns
    NaN-filled array(s) matching grid_t instead of raising when empty."""
    if not stream:
        if n_values == 1:
            return np.full(grid_t.shape, np.nan)
        return tuple(np.full(grid_t.shape, np.nan) for _ in range(n_values))
    t = np.array([s[0] for s in stream])
    values = np.array([extractor(m) for _, m in stream], dtype=np.float64)
    if n_values == 1:
        return zoh_align(t, values, grid_t)
    return tuple(zoh_align(t, values[:, i], grid_t) for i in range(n_values))


def _track_angle_fields(m):
    right = m.data[0] if len(m.data) > 0 else float("nan")
    left = m.data[1] if len(m.data) > 1 else float("nan")
    return [right, left]


def _mpc_status_fields(m):
    def get(i):
        return m.data[i] if len(m.data) > i else float("nan")
    return [get(MPC_STATUS_D_IDX), get(MPC_STATUS_PSI_TRACK_IDX),
            get(MPC_STATUS_COST_IDX), get(MPC_STATUS_MARGIN_IDX)]


def align_streams(
    streams,
    system_frequency: float,
    gyro_bias_xyz: np.ndarray,
    wheel_v_lowpass_hz: float = 10.0,
) -> AlignedBag:
    cmd = streams["/cmd_drive"]
    imu = streams["/imu"]
    odom = streams["/odom"]
    wheel = streams["/e_comms/kart_speed_m_per_s"]
    state = streams["/system_state"]

    if not cmd or not imu or not odom or not wheel:
        raise ValueError("bag missing one of /cmd_drive,/imu,/odom,/e_comms/kart_speed_m_per_s")

    t0 = max(s[0][0] for s in (cmd, imu, odom, wheel))
    t1 = min(s[-1][0] for s in (cmd, imu, odom, wheel))
    dt = 1.0 / system_frequency
    grid = np.arange(t0, t1, dt)
    if grid.size < 60:
        raise ValueError(f"bag too short: only {grid.size} ticks at {system_frequency} Hz")

    cmd_t = np.array([t for t, _ in cmd])
    cmd_throttle_raw = np.array([m.data[0] if len(m.data) >= 1 else 0.0 for _, m in cmd])
    cmd_steer_raw = np.array([m.data[1] if len(m.data) >= 2 else 0.0 for _, m in cmd])
    cmd_throttle = zoh_align(cmd_t, cmd_throttle_raw, grid)
    cmd_steer = zoh_align(cmd_t, cmd_steer_raw, grid)

    wheel_t = np.array([t for t, _ in wheel])
    wheel_v_raw = np.array([m.data for _, m in wheel])
    v_raw = zoh_align(wheel_t, wheel_v_raw, grid)
    v = _lowpass_1pole(v_raw, dt, wheel_v_lowpass_hz)

    imu_t = np.array([t for t, _ in imu])
    gyro_xyz_raw = np.array([
        [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
        for _, m in imu
    ])
    accel_xyz_raw = np.array([
        [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        for _, m in imu
    ])
    psi_dot_flu, accel_fwd = imu_to_flu(
        gyro_xyz_raw - gyro_bias_xyz[None, :], accel_xyz_raw)
    psi_dot = zoh_align(imu_t, psi_dot_flu, grid)
    accel_x = zoh_align(imu_t, accel_fwd, grid)

    odom_t = np.array([t for t, _ in odom])
    odom_x = zoh_align(odom_t, np.array([m.pose.pose.position.x for _, m in odom]), grid)
    odom_y = zoh_align(odom_t, np.array([m.pose.pose.position.y for _, m in odom]), grid)
    odom_yaw = zoh_align(odom_t, np.array([_quat_to_yaw(m.pose.pose.orientation) for _, m in odom]), grid)

    if state:
        state_t = np.array([t for t, _ in state])
        state_str = np.array([m.data for _, m in state], dtype=object)
        idx = np.clip(np.searchsorted(state_t, grid, side="right") - 1, 0, state_t.size - 1)
        state_mode = state_str[idx]
    else:
        state_mode = np.full(grid.size, "UNKNOWN", dtype=object)
    autonomous = (state_mode == "AUTONOMOUS")

    dv_dt = central_diff(v, dt)
    cmd_throttle_hist = make_history(cmd_throttle, NUM_THROTTLE_HIST)
    cmd_steer_hist = make_history(cmd_steer, NUM_STEER_HIST)

    track_angle_right, track_angle_left = _zoh_align_optional(
        streams.get("/track_angles", []), grid, _track_angle_fields, n_values=2)
    mpc_d, mpc_psi_track, mpc_cost, mpc_margin_min = _zoh_align_optional(
        streams.get("/mpc/status", []), grid, _mpc_status_fields, n_values=4)

    return AlignedBag(
        t=grid - grid[0],
        cmd_throttle=cmd_throttle, cmd_steer=cmd_steer,
        cmd_throttle_hist=cmd_throttle_hist, cmd_steer_hist=cmd_steer_hist,
        v=v, psi_dot=psi_dot, accel_x=accel_x, dv_dt=dv_dt,
        state_mode=state_mode,
        odom_x=odom_x, odom_y=odom_y, odom_yaw=odom_yaw,
        autonomous=autonomous,
        track_angle_right=track_angle_right, track_angle_left=track_angle_left,
        mpc_d=mpc_d, mpc_psi_track=mpc_psi_track,
        mpc_cost=mpc_cost, mpc_margin_min=mpc_margin_min,
    )


def _lowpass_1pole(x: np.ndarray, dt: float, fc_hz: float) -> np.ndarray:
    rc = 1.0 / (2.0 * np.pi * fc_hz)
    alpha = dt / (rc + dt)
    out = np.empty_like(x, dtype=np.float64)
    out[0] = x[0]
    for i in range(1, x.size):
        out[i] = out[i - 1] + alpha * (x[i] - out[i - 1])
    return out


def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny, cosy))


def find_bag_file(bag_dir: str) -> str:
    """Return the canonical mcap path for a bag directory, preferring
    recovered.mcap when present (for truncated bags like 215024)."""
    import glob
    import os
    cand = os.path.join(bag_dir, "recovered.mcap")
    if os.path.isfile(cand):
        return cand
    matches = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))
    if not matches:
        raise FileNotFoundError(f"no .mcap in {bag_dir}")
    return matches[0]


def main():
    import argparse
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="path to a bag directory (e.g. bags/mpc_run_...)")
    ap.add_argument("--out", required=True, help="output .npz path")
    ap.add_argument("--system-frequency", type=float, default=60.0)
    ap.add_argument("--gyro-bias-cache",
                    default=os.path.join(os.getcwd(), "imu_calibration.json"))
    args = ap.parse_args()

    mcap_path = find_bag_file(args.bag)
    print(f"reading {mcap_path}")
    streams = read_bag_streams(mcap_path)
    bias = load_gyro_bias(args.gyro_bias_cache)
    print(f"gyro_bias = {bias}")
    bag = align_streams(streams, args.system_frequency, bias)

    print(
        f"  ticks={bag.t.size}  duration={bag.t[-1]:.1f}s  "
        f"auto_frac={bag.autonomous.mean():.2f}  "
        f"v[mean={bag.v.mean():.2f}, max={bag.v.max():.2f}]  "
        f"psi_dot[std={bag.psi_dot.std():.2f}]"
    )
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    np.savez_compressed(args.out, **bag.to_npz_dict())
    print(f"wrote {args.out}")


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
