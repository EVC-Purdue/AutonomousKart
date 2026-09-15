"""Sensor noise models fitted from RTK-referenced bag data.

The sim's localization path hands the planner truth: in simulation mode
``localization_node`` integrates a bicycle model and publishes it as ``odom``
with zero covariance, so no GPS, IMU, EKF or wheel encoder exists. On the kart
the planner is handed an EKF estimate built from four noisy streams. These
models describe what each of those streams actually delivers relative to a
smoothed-RTK reference, so a simulated EKF can be driven the way the real one
is.

Each channel is

    measurement = scale * truth + bias + white + correlated

where the correlated term is an Ornstein-Uhlenbeck process. The split matters:
a filter averages white noise away but follows a slowly varying offset, and on
this hardware most of the error in RTK position and all of it in wheel speed
sits in the correlated term.

Fitted by ``python -m sim.sensor_noise``; the artifact is sim/model/sensors.json.
Nothing here is wired into the simulator yet.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Tuple

import numpy as np

from sim.rtk_reference import reference

MAD_TO_SIGMA = 1.4826

# A fix is "RTK fixed" when the receiver's reported sigma is at the 2 cm scale;
# gps_node maps fix quality onto sigma, so this is a quality test, not a guess.
RTK_SIGMA_MAX_M = 0.05
# gps_node reports 1e6 variance on a VTG heading or speed whose gates failed.
# Anything at that scale carries no information and is dropped, both when the
# channel is fitted and wherever it is consumed.
VTG_VAR_MAX = 1e5
MOVING_SPEED_MPS = 1.5
STILL_SPEED_MPS = 0.15
MIN_MOVING_S = 4.0
MIN_STILL_S = 20.0   # must exceed several times the RTK correlation time
GPS_GLITCH_M = 0.20


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
@dataclass
class ChannelNoise:
    """One scalar sensor channel.

    ``lag_s`` is positive when the measurement trails truth. It is quoted
    against the GPS header-stamp timeline, which is itself later than the
    physical fix instant because gps_node stamps on its own 10 Hz timer.
    """
    scale: float = 1.0
    bias: float = 0.0
    sigma_white: float = 0.0
    sigma_ou: float = 0.0
    tau_s: float = 1.0
    lag_s: float = 0.0
    rate_hz: float = 0.0
    r2: float = float("nan")
    sigma_core: float = 0.0
    excess_kurtosis: float = 0.0
    tail_rate: float = 0.0
    tail_rms: float = 0.0
    n_samples: int = 0

    @property
    def sigma_total(self) -> float:
        return math.hypot(self.sigma_white, self.sigma_ou)


@dataclass
class SensorNoiseModel:
    """The four streams the EKF consumes, plus provenance."""
    gps_x: ChannelNoise = field(default_factory=ChannelNoise)
    gps_y: ChannelNoise = field(default_factory=ChannelNoise)
    vtg_speed: ChannelNoise = field(default_factory=ChannelNoise)
    vtg_course: ChannelNoise = field(default_factory=ChannelNoise)
    gyro_z: ChannelNoise = field(default_factory=ChannelNoise)
    accel_x: ChannelNoise = field(default_factory=ChannelNoise)
    wheel_speed: ChannelNoise = field(default_factory=ChannelNoise)
    gps_glitch_rate: float = 0.0
    gyro_bias_still: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_sigma_still: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_mean_still: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_sigma_still: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    fitted_on: List[str] = field(default_factory=list)
    moving_minutes: float = 0.0
    still_minutes: float = 0.0
    notes: Dict[str, str] = field(default_factory=dict)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "SensorNoiseModel":
        with open(path) as f:
            raw = json.load(f)
        channels = {k: ChannelNoise(**v) for k, v in raw.items()
                    if isinstance(v, dict) and "sigma_white" in v}
        rest = {k: v for k, v in raw.items() if k not in channels}
        for k in ("gyro_bias_still", "gyro_sigma_still",
                  "accel_mean_still", "accel_sigma_still"):
            if k in rest:
                rest[k] = tuple(rest[k])
        return cls(**channels, **rest)


class ChannelSampler:
    """Draws a measurement stream from a ChannelNoise, carrying OU state."""

    def __init__(self, channel: ChannelNoise, dt: float, rng: np.random.Generator):
        self.c = channel
        self.dt = float(dt)
        self.rng = rng
        self._a = math.exp(-self.dt / max(channel.tau_s, 1e-6))
        self._step_sigma = channel.sigma_ou * math.sqrt(max(1.0 - self._a ** 2, 0.0))
        self._ou = float(rng.normal(0.0, channel.sigma_ou)) if channel.sigma_ou > 0 else 0.0

    def step(self, truth: float) -> float:
        self._ou = self._a * self._ou + self.rng.normal(0.0, self._step_sigma)
        white = self.rng.normal(0.0, self.c.sigma_white) if self.c.sigma_white > 0 else 0.0
        return self.c.scale * float(truth) + self.c.bias + self._ou + white

    def sample(self, truth: np.ndarray) -> np.ndarray:
        return np.array([self.step(v) for v in np.asarray(truth, dtype=float)])


# ---------------------------------------------------------------------------
# Estimators
# ---------------------------------------------------------------------------
def robust_sigma(residual: np.ndarray) -> float:
    r = np.asarray(residual, dtype=float)
    return float(MAD_TO_SIGMA * np.median(np.abs(r - np.median(r))))


def excess_kurtosis(residual: np.ndarray) -> float:
    """Zero for a Gaussian. Large positive means a tight core with heavy tails,
    which is where a single fitted sigma stops describing the channel."""
    r = np.asarray(residual, dtype=float)
    r = r - r.mean()
    m2 = float(np.mean(r ** 2))
    return float(np.mean(r ** 4) / (m2 * m2) - 3.0) if m2 > 0 else 0.0


def autocovariance(residual: np.ndarray, max_lag: int) -> np.ndarray:
    r = np.asarray(residual, dtype=float)
    r = r - r.mean()
    out = np.empty(max_lag + 1)
    for k in range(max_lag + 1):
        a, b = (r[:-k], r[k:]) if k else (r, r)
        out[k] = float(np.mean(a * b)) if len(a) > 10 else np.nan
    return out


TAIL_K = 4.0


def tail_stats(residual: np.ndarray) -> Tuple[float, float, float]:
    """Core sigma, excursion rate beyond TAIL_K core sigmas, and their rms size.

    Every channel here has a tight core and rare large excursions: wheel speed
    reaches an excess kurtosis of 43 because a single driven axle reads wrong
    through a corner, and GPS throws metre-scale glitches while still reporting
    a 3 cm sigma. Fitting one Gaussian to the whole variance makes ordinary
    driving far noisier than it is, so the Gaussian is fitted to the core and
    the excursions are reported alongside it rather than folded in.
    """
    core = robust_sigma(residual)
    if core <= 0:
        return core, 0.0, 0.0
    centred = np.asarray(residual, dtype=float) - np.median(residual)
    out = np.abs(centred) > TAIL_K * core
    rms = float(np.sqrt(np.mean(centred[out] ** 2))) if out.any() else 0.0
    return core, float(out.mean()), rms


def scale_to_core(sigma_white: float, sigma_ou: float, core: float) -> Tuple[float, float]:
    """Renormalise a white/OU pair so its total sigma is the measured core.

    The split identifies how the error divides between a white and a correlated
    term; the core says how big it is where the channel is behaving. Keeping the
    ratio and setting the magnitude from the core makes a draw from the model
    match ordinary driving instead of the variance of the excursions.
    """
    total = math.hypot(sigma_white, sigma_ou)
    if total <= 0 or core <= 0:
        return sigma_white, sigma_ou
    k = core / total
    return sigma_white * k, sigma_ou * k


def split_white_ou(residual: np.ndarray, dt: float, max_lag: int = 60) -> Tuple[float, float, float]:
    """Split a residual into a white part and an OU part.

    ``C(0) = sigma_white^2 + sigma_ou^2`` while ``C(k>0) = sigma_ou^2
    exp(-k dt / tau)``, so the lags from one upward identify the correlated
    part on their own and lag zero yields the white part by difference.
    Returns ``(sigma_white, sigma_ou, tau_s)``.
    """
    C = autocovariance(residual, max_lag)
    lags = np.arange(1, max_lag + 1)
    positive = C[1:] > 0
    if positive.sum() < 5:
        return float(np.sqrt(max(C[0], 0.0))), 0.0, 0.0
    keep = lags[positive][:max(5, int(positive.sum() * 0.8))]
    y = np.log(C[1:][positive][:len(keep)])
    A = np.column_stack([-keep * dt, np.ones(len(keep))])
    inv_tau, log_var = np.linalg.lstsq(A, y, rcond=None)[0]
    tau = float(1.0 / inv_tau) if inv_tau > 1e-9 else float("inf")
    var_ou = float(np.exp(log_var))
    return float(np.sqrt(max(C[0] - var_ou, 0.0))), float(np.sqrt(var_ou)), tau


def fit_channel(
    truth: np.ndarray,
    measured: np.ndarray,
    dt: float,
    rate_hz: float,
    fix_bias: bool = False,
) -> ChannelNoise:
    """Least-squares ``measured = scale * truth + bias``, then split the residual.

    ``fix_bias`` pins the intercept at zero. Use it wherever zero truth must
    give zero measurement: with a narrow speed range the scale and the
    intercept trade off, and the free-intercept fit does not generalise.
    """
    truth = np.asarray(truth, dtype=float)
    measured = np.asarray(measured, dtype=float)
    if fix_bias:
        scale = float(np.dot(truth, measured) / np.dot(truth, truth))
        bias = 0.0
    else:
        A = np.column_stack([truth, np.ones(len(truth))])
        scale, bias = (float(v) for v in np.linalg.lstsq(A, measured, rcond=None)[0])
    residual = measured - (scale * truth + bias)
    var = float(np.var(measured))
    core, tail_rate, tail_rms = tail_stats(residual)
    sigma_white, sigma_ou, tau = split_white_ou(
        np.clip(residual[:60000], -TAIL_K * core, TAIL_K * core), dt)
    sigma_white, sigma_ou = scale_to_core(sigma_white, sigma_ou, core)
    return ChannelNoise(
        scale=scale, bias=bias, sigma_white=sigma_white, sigma_ou=sigma_ou,
        tau_s=tau, rate_hz=rate_hz,
        r2=float(1.0 - residual.var() / var) if var > 0 else float("nan"),
        sigma_core=core, excess_kurtosis=excess_kurtosis(residual),
        tail_rate=tail_rate, tail_rms=tail_rms,
        n_samples=int(len(truth)),
    )


def best_lag(
    t_ref: np.ndarray, v_ref: np.ndarray,
    t_meas: np.ndarray, v_meas: np.ndarray,
    lags: np.ndarray,
) -> Tuple[float, float]:
    """Lag of the measured stream against the reference, by peak |correlation|.

    Absolute correlation, because a sign-inverted channel peaks negative and an
    argmax over the signed value returns the wrong lag entirely.
    """
    best = (0.0, 0.0)
    for lag in lags:
        r = np.interp(t_meas - lag, t_ref, v_ref)
        ok = np.isfinite(r) & np.isfinite(v_meas)
        if ok.sum() < 50:
            continue
        c = float(np.corrcoef(r[ok], v_meas[ok])[0, 1])
        if abs(c) > abs(best[1]):
            best = (float(lag), c)
    return best


# ---------------------------------------------------------------------------
# Segmentation
# ---------------------------------------------------------------------------
def fix_speed(t: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Ground speed from consecutive fixes, centred so it does not lead motion."""
    v = np.zeros(len(t))
    if len(t) < 3:
        return v
    v[1:-1] = np.hypot(x[2:] - x[:-2], y[2:] - y[:-2]) / np.maximum(t[2:] - t[:-2], 1e-3)
    v[0], v[-1] = v[1], v[-2]
    return v


def contiguous(mask: np.ndarray, t: np.ndarray, min_dur: float) -> List[Tuple[int, int]]:
    """Contiguous True runs lasting at least min_dur seconds."""
    out: List[Tuple[int, int]] = []
    i, n = 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1] and (t[j + 1] - t[j]) < 0.5:
            j += 1
        if t[j] - t[i] >= min_dur:
            out.append((i, j + 1))
        i = j + 1
    return out


def moving_stints(t, x, y, min_dur: float = MIN_MOVING_S) -> List[Tuple[int, int]]:
    """Stints are cut on motion alone.

    Fixes that are not RTK stay inside a stint so that a brief float does not
    chop it in half: the smoother down-weights them through their reported
    covariance, and the noise fits mask them out afterwards.
    """
    return contiguous(fix_speed(t, x, y) > MOVING_SPEED_MPS, t, min_dur)


def still_stints(t, x, y, sigma_ok: np.ndarray, min_dur: float = MIN_STILL_S) -> List[Tuple[int, int]]:
    return contiguous(sigma_ok & (fix_speed(t, x, y) < STILL_SPEED_MPS), t, min_dur)


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------
TOPICS = ("/gps", "/imu", "/e_comms/kart_speed_m_per_s", "/cmd_drive")

# Bags predating the 2026-09-14 mount fix carry /imu a quarter turn out, with
# the accelerometer additionally mirrored against the gyro by a negative
# default_g. Channels are converted to base_link FLU on read so the fitted
# model describes the corrected kart; set False for bags taken after the fix.
LEGACY_IMU_MOUNT = True


def imu_to_flu(gyro_xyz: np.ndarray, accel_xyz: np.ndarray,
               legacy: bool = LEGACY_IMU_MOUNT):
    """Published /imu (N, 3) pairs -> base_link FLU."""
    if not legacy:
        return gyro_xyz, accel_xyz
    gyro = np.column_stack([-gyro_xyz[:, 1], -gyro_xyz[:, 0], -gyro_xyz[:, 2]])
    accel = np.column_stack([accel_xyz[:, 1], accel_xyz[:, 0], accel_xyz[:, 2]])
    return gyro, accel


def run_mcaps(run_dir: str) -> List[str]:
    """The run's mcap splits in recording order."""
    import glob
    paths = (glob.glob(os.path.join(run_dir, "bag", "*.mcap"))
             or glob.glob(os.path.join(run_dir, "*.mcap")))
    paths = [p for p in paths if os.path.getsize(p) > 0]
    if not paths:
        raise FileNotFoundError(f"no non-empty mcap under {run_dir}")
    return sorted(paths, key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))


def read_streams(run_dir: str) -> dict:
    """GPS, IMU and wheel speed from one run.

    GPS and IMU are keyed on the publisher's header stamp, which is the clock
    the EKF integrates and the only one that is evenly spaced: recorder receipt
    times arrive in bursts as short as 0.1 ms and make every rate-derived
    quantity meaningless. Wheel speed is a std_msgs/Float32 with no header, so
    receipt time is all it has.
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    g = {k: [] for k in ("t", "x", "y", "var_x", "var_y", "yaw", "yaw_var",
                         "speed", "speed_var")}
    imu = {k: [] for k in ("t", "gyro", "accel")}
    wheel = {"t": [], "v": []}
    cmd = {"t": [], "steer": [], "throttle": []}
    for path in run_mcaps(run_dir):
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, channel, msg, ros in reader.iter_decoded_messages(topics=list(TOPICS)):
                if channel.topic == "/gps":
                    q = ros.pose.pose.orientation
                    stamp = ros.header.stamp
                    g["t"].append(stamp.sec + stamp.nanosec * 1e-9)
                    g["x"].append(ros.pose.pose.position.x)
                    g["y"].append(ros.pose.pose.position.y)
                    g["var_x"].append(ros.pose.covariance[0])
                    g["var_y"].append(ros.pose.covariance[7])
                    g["yaw"].append(math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))
                    g["yaw_var"].append(ros.pose.covariance[35])
                    g["speed"].append(ros.twist.twist.linear.x)
                    g["speed_var"].append(ros.twist.covariance[0])
                elif channel.topic == "/imu":
                    stamp = ros.header.stamp
                    imu["t"].append(stamp.sec + stamp.nanosec * 1e-9)
                    a, b = ros.angular_velocity, ros.linear_acceleration
                    imu["gyro"].append((a.x, a.y, a.z))
                    imu["accel"].append((b.x, b.y, b.z))
                elif channel.topic == "/cmd_drive":
                    # No header on this topic either, so like wheel speed it is
                    # keyed on recorder receipt time, not the publisher's clock.
                    cmd["t"].append(msg.log_time * 1e-9)
                    cmd["throttle"].append(ros.data[0])
                    cmd["steer"].append(ros.data[1])
                else:
                    wheel["t"].append(msg.log_time * 1e-9)
                    wheel["v"].append(ros.data)
    out = {f"gps_{k}": np.asarray(v, dtype=float) for k, v in g.items()}
    out["imu_t"] = np.asarray(imu["t"], dtype=float)
    out["imu_gyro"], out["imu_accel"] = imu_to_flu(
        np.asarray(imu["gyro"], dtype=float).reshape(-1, 3),
        np.asarray(imu["accel"], dtype=float).reshape(-1, 3))
    out["wheel_t"] = np.asarray(wheel["t"], dtype=float)
    out["wheel_v"] = np.asarray(wheel["v"], dtype=float)
    out["cmd_t"] = np.asarray(cmd["t"], dtype=float)
    out["cmd_steer"] = np.asarray(cmd["steer"], dtype=float)
    out["cmd_throttle"] = np.asarray(cmd["throttle"], dtype=float)
    return out


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------
GPS_DT, IMU_DT, WHEEL_DT = 0.10, 0.01, 0.01
LAG_GRID = np.arange(-0.6, 0.601, 0.01)


def channel_lag(samples) -> float:
    """The lag a channel ships: the median over per-stint (lag, correlation)
    fits. A median, because one stint whose correlation peak lands badly must
    not move the alignment every other stint is fitted at."""
    return float(np.median(np.asarray(samples)[:, 0])) if samples else 0.0


def _pools() -> dict:
    keys = ("gyro_truth", "gyro_meas", "accel_truth", "accel_meas",
            "wheel_truth", "wheel_meas", "vs_truth", "vs_meas",
            "course_truth", "course_meas", "course_speed",
            "res_x", "res_y", "still_x", "still_y", "still_gyro", "still_accel")
    return {k: [] for k in keys}


# Channels whose transport lag is measured, and the pools they fill. Their
# truth/measurement pairs are held back until every stint has been seen so the
# alignment can use the channel's median lag; see the end of `collect`.
LAGGED_CHANNELS = {"gyro_z": ("gyro_truth", "gyro_meas"),
                   "wheel_speed": ("wheel_truth", "wheel_meas")}


def collect(run_dirs: List[str]) -> Tuple[dict, dict, dict]:
    """Pool truth/measurement pairs over every moving stint in every run."""
    P = _pools()
    lags = {"gyro_z": [], "wheel_speed": []}
    pending = {k: [] for k in LAGGED_CHANNELS}
    totals = {"moving_s": 0.0, "still_s": 0.0, "gps_glitch": 0, "gps_n": 0}

    for run_dir in run_dirs:
        d = read_streams(run_dir)
        t, x, y = d["gps_t"], d["gps_x"], d["gps_y"]
        if len(t) < 50:
            continue
        sigma_ok = (np.sqrt(d["gps_var_x"]) < RTK_SIGMA_MAX_M) & \
                   (np.sqrt(d["gps_var_y"]) < RTK_SIGMA_MAX_M)

        for i0, i1 in still_stints(t, x, y, sigma_ok):
            totals["still_s"] += t[i1 - 1] - t[i0]
            P["still_x"].append(x[i0:i1] - x[i0:i1].mean())
            P["still_y"].append(y[i0:i1] - y[i0:i1].mean())
            m = (d["imu_t"] >= t[i0]) & (d["imu_t"] <= t[i1 - 1])
            if m.sum() > 100:
                P["still_gyro"].append(d["imu_gyro"][m])
                P["still_accel"].append(d["imu_accel"][m])

        for i0, i1 in moving_stints(t, x, y):
            ref = reference(t[i0:i1], x[i0:i1], y[i0:i1],
                            d["gps_var_x"][i0:i1], d["gps_var_y"][i0:i1])
            keep = (~ref.rejected) & sigma_ok[i0:i1]
            if keep.sum() < 20:
                continue
            totals["moving_s"] += ref.t[-1] - ref.t[0]
            rx = (x[i0:i1] - ref.x)[keep]
            ry = (y[i0:i1] - ref.y)[keep]
            P["res_x"].append(rx)
            P["res_y"].append(ry)
            totals["gps_glitch"] += int(np.sum(np.hypot(rx, ry) > GPS_GLITCH_M))
            totals["gps_n"] += int(keep.sum())

            lo, hi = ref.t[0] + 0.5, ref.t[-1] - 0.5
            mi = (d["imu_t"] >= lo) & (d["imu_t"] <= hi)
            if mi.sum() > 200:
                P["accel_truth"].append(np.interp(d["imu_t"][mi], ref.t, ref.accel_long))
                P["accel_meas"].append(d["imu_accel"][mi, 0])
                lags["gyro_z"].append(best_lag(ref.t, ref.psi_dot, d["imu_t"][mi],
                                               d["imu_gyro"][mi, 2], LAG_GRID))
                pending["gyro_z"].append((ref.t, ref.psi_dot, d["imu_t"][mi],
                                          d["imu_gyro"][mi, 2]))
            mw = (d["wheel_t"] >= lo) & (d["wheel_t"] <= hi)
            if mw.sum() > 200:
                lags["wheel_speed"].append(best_lag(ref.t, ref.speed, d["wheel_t"][mw],
                                                    d["wheel_v"][mw], LAG_GRID))
                pending["wheel_speed"].append((ref.t, ref.speed, d["wheel_t"][mw],
                                               d["wheel_v"][mw]))
            sl = slice(i0, i1)
            ok_v = (d["gps_speed_var"][sl] < VTG_VAR_MAX) & keep
            P["vs_truth"].append(ref.speed[ok_v])
            P["vs_meas"].append(d["gps_speed"][sl][ok_v])
            ok_y = (d["gps_yaw_var"][sl] < VTG_VAR_MAX) & keep
            P["course_truth"].append(ref.course[ok_y])
            P["course_meas"].append(d["gps_yaw"][sl][ok_y])
            P["course_speed"].append(ref.speed[ok_y])

    # Truth is sampled at the time each measurement describes, not the time it
    # is stamped: best_lag fits the lag at which v_meas(T) matches
    # v_ref(T - lag), so that is where the truth has to be read. Fitting on the
    # stamped time charges the transport lag to the noise term, and sigma_core
    # is the weight plant_reference.fuse hands the channel -- an inflated one
    # under-weights the 100 Hz streams in the very reference the 60 Hz target
    # rests on. The median over stints is the lag the model ships and fuse
    # aligns with, so it is the one applied.
    for name, (truth_key, meas_key) in LAGGED_CHANNELS.items():
        lag = channel_lag(lags[name])
        for ref_t, ref_v, meas_t, meas_v in pending[name]:
            P[truth_key].append(np.interp(meas_t - lag, ref_t, ref_v))
            P[meas_key].append(meas_v)

    pooled = {k: (np.concatenate(v) if v else np.array([])) for k, v in P.items()}
    return pooled, lags, totals


def fit(run_dirs: List[str]) -> Tuple[SensorNoiseModel, dict]:
    P, lags, totals = collect(run_dirs)
    model = SensorNoiseModel(
        fitted_on=[os.path.basename(r.rstrip("/")) for r in run_dirs],
        moving_minutes=totals["moving_s"] / 60.0,
        still_minutes=totals["still_s"] / 60.0,
    )

    # RTK position: fitted on stationary data, where truth is exactly constant
    # and the whole residual is therefore sensor error. The moving residual is
    # a cross-check only, since the smoother absorbs anything slow enough to
    # look like motion.
    for axis, res in (("gps_x", P["still_x"]), ("gps_y", P["still_y"])):
        core, tail_rate, tail_rms = tail_stats(res)
        sw, so, tau = split_white_ou(np.clip(res, -TAIL_K * core, TAIL_K * core), GPS_DT)
        sw, so = scale_to_core(sw, so, core)
        setattr(model, axis, ChannelNoise(
            scale=1.0, bias=0.0, sigma_white=sw, sigma_ou=so, tau_s=tau,
            rate_hz=1.0 / GPS_DT, sigma_core=core,
            excess_kurtosis=excess_kurtosis(res), tail_rate=tail_rate,
            tail_rms=tail_rms, n_samples=int(len(res))))
    model.gps_glitch_rate = totals["gps_glitch"] / max(totals["gps_n"], 1)

    model.gyro_z = fit_channel(P["gyro_truth"], P["gyro_meas"], IMU_DT, 1.0 / IMU_DT)
    model.accel_x = fit_channel(P["accel_truth"], P["accel_meas"], IMU_DT, 1.0 / IMU_DT)
    model.wheel_speed = fit_channel(P["wheel_truth"], P["wheel_meas"], WHEEL_DT,
                                    1.0 / WHEEL_DT, fix_bias=True)
    model.vtg_speed = fit_channel(P["vs_truth"], P["vs_meas"], GPS_DT, 1.0 / GPS_DT)

    err = np.arctan2(np.sin(P["course_meas"] - P["course_truth"]),
                     np.cos(P["course_meas"] - P["course_truth"]))
    core, tail_rate, tail_rms = tail_stats(err)
    sw, so, tau = split_white_ou(np.clip(err, -TAIL_K * core, TAIL_K * core), GPS_DT)
    sw, so = scale_to_core(sw, so, core)
    model.vtg_course = ChannelNoise(
        scale=1.0, bias=float(err.mean()), sigma_white=sw, sigma_ou=so, tau_s=tau,
        rate_hz=1.0 / GPS_DT, sigma_core=core, excess_kurtosis=excess_kurtosis(err),
        tail_rate=tail_rate, tail_rms=tail_rms, n_samples=int(len(err)))

    for name, samples in lags.items():
        if samples:
            getattr(model, name).lag_s = channel_lag(samples)

    g = P["still_gyro"] if P["still_gyro"].size else np.zeros((1, 3))
    a = P["still_accel"] if P["still_accel"].size else np.zeros((1, 3))
    model.gyro_bias_still = tuple(float(g[:, i].mean()) for i in range(3))
    model.gyro_sigma_still = tuple(float(g[:, i].std()) for i in range(3))
    model.accel_mean_still = tuple(float(a[:, i].mean()) for i in range(3))
    model.accel_sigma_still = tuple(float(a[:, i].std()) for i in range(3))

    model.notes = {
        "frame": "channels are base_link FLU. Bags predating the 2026-09-14 "
                 "mount fix are converted on read by imu_to_flu.",
        "accel_x": "low R^2 because true along-track acceleration is small "
                   "against the vibration floor, not because the axis is wrong.",
        "wheel_speed": "intercept pinned at zero; the free-intercept fit does "
                       "not generalise across runs. Residual is entirely "
                       "correlated, not white.",
        "lag_s": "quoted against the GPS header-stamp timeline, which trails "
                 "the physical fix because gps_node stamps on its own timer.",
    }
    return model, P


CHANNEL_POOLS = {
    "gyro_z": ("gyro_truth", "gyro_meas", IMU_DT, "rad/s"),
    "accel_x": ("accel_truth", "accel_meas", IMU_DT, "m/s^2"),
    "wheel_speed": ("wheel_truth", "wheel_meas", WHEEL_DT, "m/s"),
    "vtg_speed": ("vs_truth", "vs_meas", GPS_DT, "m/s"),
}


def channel_residual(name: str, model: SensorNoiseModel, pools: dict) -> np.ndarray:
    """Residual of a held-out pool against the fitted channel."""
    if name in ("gps_x", "gps_y"):
        # Fitted on stationary data, so graded on stationary data. The moving
        # residual is a different quantity: the smoother absorbs anything slow
        # enough to look like motion, which is most of this channel's error.
        return pools["still_" + name[-1]]
    if name == "vtg_course":
        err = pools["course_meas"] - pools["course_truth"]
        return np.arctan2(np.sin(err), np.cos(err))
    tk, mk, _, _ = CHANNEL_POOLS[name]
    c = getattr(model, name)
    return pools[mk] - (c.scale * pools[tk] + c.bias)


def model_residual(name: str, model: SensorNoiseModel, n: int, seed: int = 0) -> np.ndarray:
    """Draw the same length of residual from the fitted channel."""
    c = getattr(model, name)
    dt = 1.0 / c.rate_hz if c.rate_hz > 0 else 0.1
    sampler = ChannelSampler(
        ChannelNoise(scale=0.0, bias=0.0, sigma_white=c.sigma_white,
                     sigma_ou=c.sigma_ou, tau_s=c.tau_s, rate_hz=c.rate_hz),
        dt, np.random.default_rng(seed))
    return sampler.sample(np.zeros(n))


def validate(model: SensorNoiseModel, run_dirs: List[str]) -> Tuple[List[dict], dict]:
    """Compare the fitted channels against runs they were not fitted on.

    Each row carries the held-out empirical statistics, the same statistics
    re-fitted on the held-out data, and the statistics of a draw from the
    fitted model, so a channel can fail either by drifting between runs or by
    being the wrong shape.
    """
    pools, _, totals = collect(run_dirs)
    rows = []
    for name in ("gps_x", "gps_y", "gyro_z", "accel_x", "wheel_speed",
                 "vtg_speed", "vtg_course"):
        c = getattr(model, name)
        held = channel_residual(name, model, pools)
        if held.size < 100:
            continue
        drawn = model_residual(name, model, min(len(held), 60000))
        refit = None
        if name in CHANNEL_POOLS:
            tk, mk, dt, _ = CHANNEL_POOLS[name]
            refit = fit_channel(pools[tk], pools[mk], dt, c.rate_hz,
                                fix_bias=(name == "wheel_speed"))
        rows.append(dict(
            channel=name,
            fitted_scale=c.scale, held_scale=refit.scale if refit else float("nan"),
            fitted_sigma=c.sigma_core, held_sigma=robust_sigma(held),
            model_sigma=robust_sigma(drawn),
            fitted_kurt=c.excess_kurtosis, held_kurt=excess_kurtosis(held),
            fitted_rho1=float(np.corrcoef(
                *(lambda r: (r[:-1], r[1:]))(held))[0, 1]) if held.size > 10 else float("nan"),
            model_rho1=float(np.corrcoef(drawn[:-1], drawn[1:])[0, 1]),
            n=int(held.size)))
    return rows, pools


def report(rows: List[dict], pools: dict | None = None) -> None:
    print(f"{'channel':<13}{'scale fit':>10}{'scale held':>11}"
          f"{'sigma fit':>11}{'sigma held':>11}{'sigma model':>12}"
          f"{'rho1 held':>10}{'rho1 model':>11}{'kurt held':>10}{'n':>9}")
    for r in rows:
        print(f"{r['channel']:<13}{r['fitted_scale']:>10.4f}{r['held_scale']:>11.4f}"
              f"{r['fitted_sigma']:>11.4f}{r['held_sigma']:>11.4f}{r['model_sigma']:>12.4f}"
              f"{r['fitted_rho1']:>10.2f}{r['model_rho1']:>11.2f}"
              f"{r['held_kurt']:>10.1f}{r['n']:>9d}")
    if pools is not None and pools["res_x"].size > 50:
        moving = 0.5 * (robust_sigma(pools["res_x"]) + robust_sigma(pools["res_y"]))
        still = 0.5 * (robust_sigma(pools["still_x"]) + robust_sigma(pools["still_y"]))
        print(f"\nRTK position cross-check: {still * 100:.2f} cm stationary against "
              f"{moving * 100:.2f} cm while moving, the same runs. Motion inflates the "
              f"white part; the correlated part is only identifiable at rest.")


FIGURE_CHANNELS = (("gps_x", "RTK position x", "m"),
                   ("gyro_z", "IMU gyro z", "rad/s"),
                   ("accel_x", "IMU accel x", "m/s$^2$"),
                   ("wheel_speed", "VESC wheel speed", "m/s"),
                   ("vtg_course", "VTG course", "rad"))


def figure(model: SensorNoiseModel, pools: dict, path: str) -> None:
    """Held-out residual against a draw from the fitted model, per channel."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    real, fit_c = "#111111", "#1f6fb4"
    n = len(FIGURE_CHANNELS)
    fig_, axes = plt.subplots(2, n, figsize=(3.0 * n, 6.2))
    for col, (name, label, unit) in enumerate(FIGURE_CHANNELS):
        c = getattr(model, name)
        held = channel_residual(name, model, pools)
        drawn = model_residual(name, model, min(len(held), 60000))
        span = 6.0 * max(c.sigma_core, 1e-9)
        edges = np.linspace(-span, span, 61)

        A = axes[0, col]
        A.hist(held - np.median(held), bins=edges, density=True, color=real,
               alpha=0.18, lw=0)
        A.hist(held - np.median(held), bins=edges, density=True, histtype="step",
               color=real, lw=1.5)
        A.hist(drawn, bins=edges, density=True, histtype="step", color=fit_c, lw=1.5)
        A.set_yscale("log")
        A.set_title(f"{label}", fontsize=10, loc="left")
        A.set_xlabel(f"residual ({unit})", fontsize=8.5)
        A.tick_params(labelsize=7.5)
        A.spines[["top", "right"]].set_visible(False)
        if col == 0:
            A.set_ylabel("density", fontsize=9)

        dt = 1.0 / c.rate_hz if c.rate_hz > 0 else GPS_DT
        max_lag = 40
        lags = np.arange(max_lag + 1) * dt
        ah = autocovariance(held[:60000], max_lag)
        ad = autocovariance(drawn, max_lag)
        B = axes[1, col]
        B.axhline(0.0, color="0.6", lw=0.8)
        B.plot(lags, ah / ah[0], color=real, lw=1.5)
        B.plot(lags, ad / ad[0], color=fit_c, lw=1.5)
        B.set_ylim(-0.2, 1.05)
        B.set_xlabel("lag (s)", fontsize=8.5)
        B.tick_params(labelsize=7.5)
        B.spines[["top", "right"]].set_visible(False)
        if col == 0:
            B.set_ylabel("autocorrelation", fontsize=9)

    fig_.legend(handles=[Line2D([], [], color=real, lw=2, label="held-out runs"),
                         Line2D([], [], color=fit_c, lw=2, label="draw from the fitted model")],
                loc="lower center", ncol=2, frameon=False, fontsize=9.5)
    fig_.suptitle("Fitted sensor noise against runs it was not fitted on", fontsize=12)
    fig_.subplots_adjust(left=0.06, right=0.99, top=0.90, bottom=0.14, wspace=0.28, hspace=0.42)
    fig_.savefig(path, dpi=150)


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", nargs="*", default=[], help="run directories to fit on")
    ap.add_argument("--out", help="sensors.json to write")
    ap.add_argument("--validate-on", nargs="*", default=[],
                    help="held-out run directories to grade the fit against")
    ap.add_argument("--load", help="grade an existing sensors.json instead of fitting")
    ap.add_argument("--figure", help="write the validation figure here")
    args = ap.parse_args()
    if args.load:
        model = SensorNoiseModel.load(args.load)
    else:
        model, _ = fit(args.runs)
    if args.out:
        model.save(args.out)
        print(f"wrote {args.out}: {model.moving_minutes:.1f} min moving, "
              f"{model.still_minutes:.1f} min stationary, "
              f"{len(model.fitted_on)} runs")
    if args.validate_on:
        rows, pools = validate(model, args.validate_on)
        report(rows, pools)
        if args.figure:
            figure(model, pools, args.figure)
            print(f"wrote {args.figure}")


if __name__ == "__main__":
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
