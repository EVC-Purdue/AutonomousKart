"""Bags to pooled training pairs for the learned plant.

Training pools every transition; which bag it came from carries no information.
Scoring needs contiguous windows, because a 120-step rollout has to be compared
against two contiguous seconds of truth, and because neighbouring 60 Hz samples
are near-duplicates that would leak across a random split.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional

import numpy as np

from sim.plant_reference import FusedReference, PLANT_HZ, fuse, reachable_fixes
from sim.sensor_noise import (RTK_SIGMA_MAX_M, VTG_VAR_MAX, moving_stints,
                              read_streams)

HISTORY = 5
FEATURE_NAMES = ("v", "psi_dot", "delta_cmd", "throttle_sp")

# The heavy kart's re-identified wheel slew (CLAUDE.md season learnings:
# steer_rate_max_degps fell from 180 to 33.7 when the kart gained weight).
# MPC demanded 91-127 deg/s p90 on these bags, so for seconds at a time the
# command and the wheel are different signals, and `delta_cmd` carries the
# wheel angle rather than the request.
STEER_RATE_MAX_DEGPS = 33.7


def slew_limit(cmd, hz: float, start: float = 0.0) -> np.ndarray:
    """Integrate a steering command through the actuator's rate limit."""
    cmd = np.asarray(cmd, dtype=float)
    step = STEER_RATE_MAX_DEGPS / float(hz)
    out = np.empty_like(cmd)
    w = float(start)
    for i, c in enumerate(cmd):
        out[i] = w
        w += float(np.clip(c - w, -step, step))
    return out
TARGET_NAMES = ("alpha", "a_long")
MIRROR_FEATURE_SIGN = np.array([1.0, -1.0, -1.0, 1.0])
# alpha is a yaw quantity and flips under a left-right reflection; the
# longitudinal acceleration does not.
MIRROR_TARGET_SIGN = np.array([-1.0, 1.0])


@dataclass
class Segment:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    v: np.ndarray
    omega: np.ndarray
    delta_cmd: np.ndarray        # wheel angle, after the actuator rate limit
    throttle_sp: np.ndarray
    split: str
    # The command the planner published, before the actuator. The model reads
    # `delta_cmd`; this is what a runtime plant is handed and slews for itself,
    # and it is the only way to drive the runtime and the trainer from one
    # source and still land on the same window.
    cmd_raw: Optional[np.ndarray] = None


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def targets_from_reference(ref: FusedReference) -> np.ndarray:
    """Per-tick (angular acceleration, longitudinal acceleration).

    The earlier target set -- body-frame ds, dn, dpsi, dv -- could not work.
    `ds` is `v * dt` and `dpsi` is `psi_dot * dt` to R^2 = 1.000000 against two
    of the model's own inputs, so a model scored on them learns to copy its
    input and never has to represent what a command does. Measured on the
    trained result: steering sensitivity 0.00000 rad/s per degree for the
    linear fit and 0.00081 for the network, against 0.02514 for a bicycle
    model. Both were persistence predictors that scored well because yaw rate
    stays autocorrelated past a second.

    `dn` is dropped outright: the reference comes from a unicycle smoother with
    no lateral velocity state, so side-slip is identically zero there by
    construction and no amount of data can teach it.

    What commands actually drive is acceleration, so that is what is predicted.
    Position and heading follow by integration that is definitional rather than
    a kart model, and reproduces a 145 s stint to 0.18 m given true targets.
    """
    dt = float(np.median(np.diff(ref.t)))
    return np.column_stack([np.diff(ref.omega) / dt, np.diff(ref.v) / dt])


def windows_from_segment(seg: Segment) -> Tuple[np.ndarray, np.ndarray]:
    ref = FusedReference(t=seg.t, x=seg.x, y=seg.y, psi=seg.psi, v=seg.v,
                         omega=seg.omega, accel=np.zeros_like(seg.v))
    tg = targets_from_reference(ref)
    per_tick = np.column_stack([seg.v, seg.omega, seg.delta_cmd, seg.throttle_sp])
    m = len(seg.t) - HISTORY
    feats = np.empty((m, HISTORY, len(FEATURE_NAMES)))
    for k in range(m):
        feats[k] = per_tick[k:k + HISTORY]
    return feats, tg[HISTORY - 1:HISTORY - 1 + m]


def mirror(features: np.ndarray, targets: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Left-right reflection. A kart is symmetric, so this is a valid sample."""
    return features * MIRROR_FEATURE_SIGN, targets * MIRROR_TARGET_SIGN


def mirror_rollout(window0: np.ndarray, cmd_seq: np.ndarray,
                   target_seq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`mirror` for a whole rollout sequence.

    Reflecting one training pair is not enough for the rollout loss: the
    sequence is driven by `cmd_seq` and scored against `target_seq`, so every
    row the rollout touches has to be reflected by the same signs or the
    mirrored sample describes a kart that steers one way and turns the other.
    `cmd_seq` carries only the command half of a tick, so it takes the command
    half of MIRROR_FEATURE_SIGN.
    """
    return (window0 * MIRROR_FEATURE_SIGN,
            cmd_seq * MIRROR_FEATURE_SIGN[2:],
            target_seq * MIRROR_TARGET_SIGN)


class Whitener:
    def __init__(self, mean=None, std=None):
        self.mean = mean
        self.std = std

    def fit(self, a: np.ndarray) -> "Whitener":
        flat = a.reshape(-1, a.shape[-1]) if a.ndim > 2 else a
        self.mean = flat.mean(axis=0)
        self.std = np.maximum(flat.std(axis=0), 1e-8)
        return self

    def apply(self, a: np.ndarray) -> np.ndarray:
        return (a - self.mean) / self.std

    def invert(self, a: np.ndarray) -> np.ndarray:
        return a * self.std + self.mean

    def to_dict(self) -> dict:
        return {"mean": self.mean.tolist(), "std": self.std.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "Whitener":
        return cls(np.asarray(d["mean"], dtype=float), np.asarray(d["std"], dtype=float))


def _zoh(sample_t, sample_v, grid):
    """Zero-order hold of a command stream onto `grid`.

    A grid tick before the first sample holds zero, and a stream with no
    samples at all holds zero throughout: back-filling the first command
    over the ticks that precede it puts a command the kart had not yet been
    given into the window, and a manual-driving bag carries no /cmd_drive to
    back-fill from. `build_segments` drops the ticks that would take the zero.
    """
    sample_v = np.asarray(sample_v, dtype=float)
    out = np.zeros(len(grid))
    if sample_v.size == 0:
        return out
    idx = np.searchsorted(sample_t, grid, side="right") - 1
    have = idx >= 0
    out[have] = sample_v[idx[have]]
    return out


SPLIT_MODULUS = 7
VAL_RESIDUE = 2
TEST_RESIDUE = 4


def assign_splits(keys: List[float]) -> List[str]:
    """Deterministic train/val/test tag per stint, by rank in `keys`.

    `keys` is the stratification variable, and it is mean speed rather than
    duration. Ranking by duration says nothing about which regimes a split
    covers: on the 2026-09-13 bags it handed validation 68 s that never left
    3.7-4.0 m/s and test 17 s that never left 1.9-2.1 m/s, below the training
    tenth percentile. On held-out data that narrow every plant looks locally
    linear, so the splits could not tell a learned model from a straight line.
    Ranking by speed spreads each split across the range instead.

    Stints are ranked by key, descending; rank 0 is the largest.
    `VAL_RESIDUE` of every `SPLIT_MODULUS` ranks goes to val, `TEST_RESIDUE`
    goes to test, everything else trains — holding out 2/`SPLIT_MODULUS` of
    the stints. A random coin flip over a duration budget can hand an entire
    split to one stint's draw; this can't: once there are more than a
    handful of stints, val and test are both guaranteed non-empty, and both
    get long and short stints alike.
    """
    order = sorted(range(len(keys)), key=lambda i: -keys[i])
    tags = [""] * len(keys)
    for rank, idx in enumerate(order):
        if rank % SPLIT_MODULUS == VAL_RESIDUE:
            tags[idx] = "val"
        elif rank % SPLIT_MODULUS == TEST_RESIDUE:
            tags[idx] = "test"
        else:
            tags[idx] = "train"
    return tags


# A fused reference is only believable if it stays near the RTK fixes that
# produced it. The smoother can diverge on a stint -- on 2026-09-13 bags two of
# 24 ran away to v = -82 km/s and positions 99 km from fixes accurate to 3 cm --
# and a diverged stint silently destroys the whitener for every other stint,
# because std is set by the garbage rather than the signal. Nothing in a
# synthetic test can catch this, so it is gated on the data itself.
REFERENCE_MAX_OFFSET_M = 5.0
REFERENCE_MAX_SPEED_MPS = 25.0
REFERENCE_MAX_OMEGA_RADPS = 10.0


def reference_is_plausible(ref, gps_t, gps_x, gps_y, rtk_ok) -> bool:
    """True when the fused reference tracks its own RTK fixes and stays physical."""
    if not np.all(np.isfinite(ref.x)) or not np.all(np.isfinite(ref.v)):
        return False
    if np.abs(ref.v).max() > REFERENCE_MAX_SPEED_MPS:
        return False
    if np.abs(ref.omega).max() > REFERENCE_MAX_OMEGA_RADPS:
        return False
    # The same mask fuse applied: a fix it dropped is not evidence against
    # the reference it built without it.
    sel = np.asarray(rtk_ok, dtype=bool) & reachable_fixes(gps_t, gps_x, gps_y)
    if sel.sum() < 2:
        return False
    fx = np.interp(gps_t[sel], ref.t, ref.x)
    fy = np.interp(gps_t[sel], ref.t, ref.y)
    offset = np.hypot(fx - gps_x[sel], fy - gps_y[sel])
    return bool(np.max(offset) <= REFERENCE_MAX_OFFSET_M)


def build_segments(run_dirs: List[str], noise) -> List[Segment]:
    """One Segment per RTK-backed moving stint, tagged train / val / test.

    Every filter (RTK, the wheel-overlap guard, the minimum fused length)
    runs first; `assign_splits` then ranks and tags only the stints that
    actually survive into the returned list, not the larger pre-filter
    population, so the ~20% held out is measured against what's returned.
    """
    from sim.sensor_noise import read_streams as _read
    raw = []
    for rd in run_dirs:
        d = _read(rd)
        t, x, y = d["gps_t"], d["gps_x"], d["gps_y"]
        if len(t) < 50:
            continue
        ok = (np.sqrt(d["gps_var_x"]) < RTK_SIGMA_MAX_M) & (np.sqrt(d["gps_var_y"]) < RTK_SIGMA_MAX_M)
        for i0, i1 in moving_stints(t, x, y):
            if ok[i0:i1].mean() < 0.8:
                continue
            raw.append((os.path.basename(rd.rstrip("/")), d, i0, i1))

    built, rejected = [], []
    for run, d, i0, i1 in raw:
        # A sensor stream (e.g. the VESC wheel speed) can drop out for the
        # rest of a run; fuse's own t0/t1 window would then be empty and it
        # raises. Mirror that window here and skip the stint instead.
        t0 = max(d["gps_t"][i0], d["imu_t"][0], d["wheel_t"][0])
        t1 = min(d["gps_t"][i1 - 1], d["imu_t"][-1], d["wheel_t"][-1])
        if t1 <= t0:
            continue
        # gps_node sets the heading variance to 1e6 whenever the VTG gates
        # fail, and sensor_noise.collect masks those samples out when it fits
        # the course channel. Fold one into the training truth at the fitted
        # variance and the reference follows a heading the receiver disowned,
        # on the channel that dominates omega.
        ok_yaw = d["gps_yaw_var"][i0:i1] < VTG_VAR_MAX
        if ok_yaw.sum() < 2:
            continue
        ref = fuse(d["gps_t"][i0:i1], d["gps_x"][i0:i1], d["gps_y"][i0:i1],
                   d["gps_var_x"][i0:i1],
                   d["gps_t"][i0:i1][ok_yaw], d["gps_yaw"][i0:i1][ok_yaw],
                   d["imu_t"], d["imu_gyro"][:, 2], d["wheel_t"], d["wheel_v"],
                   noise, gps_var_y=d["gps_var_y"][i0:i1])
        # Recomputed from d, not reused from the first loop: `ok` there belongs
        # to whichever run was processed last, not to this stint's run.
        rtk_ok = ((np.sqrt(d["gps_var_x"][i0:i1]) < RTK_SIGMA_MAX_M)
                  & (np.sqrt(d["gps_var_y"][i0:i1]) < RTK_SIGMA_MAX_M))
        if not reference_is_plausible(ref, d["gps_t"][i0:i1], d["gps_x"][i0:i1],
                                      d["gps_y"][i0:i1], rtk_ok):
            # `rd` here would be whichever run the first loop finished on,
            # not this stint's -- the run name travels with the stint instead.
            rejected.append((run,
                             float(d["gps_t"][i1 - 1] - d["gps_t"][i0]),
                             float(np.abs(ref.v).max()),
                             float(np.abs(ref.omega).max())))
            continue
        # Ticks before the first recorded command are dropped rather than
        # filled: there is no command to hold there, and a zero throttle at
        # speed is a command the kart was never given. A run with no
        # /cmd_drive at all -- a manual-driving bag -- drops entirely. Both
        # happen before assign_splits, so the held-out fraction is measured
        # against the stints that actually survive.
        cmd_t = d["cmd_t"]
        if len(cmd_t) == 0:
            continue
        first = int(np.searchsorted(ref.t, cmd_t[0]))
        if len(ref.t) - first < HISTORY + 130:
            continue
        dur = d["gps_t"][i1 - 1] - d["gps_t"][i0]
        built.append((dur, ref, first, cmd_t, d["cmd_steer"], d["cmd_throttle"]))

    if rejected:
        # Loud, because a silently dropped stint is indistinguishable from a
        # stint that never existed, and these are the ones worth investigating.
        print(f"build_segments: rejected {len(rejected)} stint(s) whose fused "
              f"reference is not plausible:")
        for run, dur, mv, mo in rejected:
            print(f"  {run} {dur:.1f}s  max|v| {mv:.1f} m/s  max|omega| {mo:.1f} rad/s")

    # Stratified on mean speed, not duration: see assign_splits.
    tags = assign_splits([float(np.mean(ref.v)) for _, ref, *_ in built])

    segments = []
    for (dur, ref, first, cmd_t, cmd_steer, cmd_throttle), split in zip(built, tags):
        sl = slice(first, None)
        raw = _zoh(cmd_t, cmd_steer, ref.t[sl])
        segments.append(Segment(
            t=ref.t[sl], x=ref.x[sl], y=ref.y[sl], psi=ref.psi[sl], v=ref.v[sl],
            omega=ref.omega[sl],
            delta_cmd=slew_limit(raw, PLANT_HZ, start=float(raw[0])),
            throttle_sp=_zoh(cmd_t, cmd_throttle, ref.t[sl]),
            split=split, cmd_raw=raw))
    return segments
