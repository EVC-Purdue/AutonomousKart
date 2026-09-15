"""Two-second clip acceptance: does the sim leave the track where the kart did?

A rollout table says how far the sim drifts on average. It does not say
whether the sim reproduces the events that matter, which are the moments the
planner put the kart off the racing line. This scores that directly: find the
2 s windows where the recorded kart left the track and the ones where it
stayed on, hand the plant the kart's own state at the start of each, and ask
which side of the track edge it ends up on.

Labels take a margin. 2.0 m is the planner's `track_half_width_m` corridor
parameter, not the physical track, so a kart 2.2 m off the line may simply be
cutting a corner: a clip counts as a departure only past `OFF_M` and only if
it stays out for a quarter of the window, and as on-track only inside `ON_M`.
Clips between the two are ambiguous and are excluded rather than guessed at.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

# Track edge, and the margins that keep corner-cutting out of the labels.
HALF_WIDTH_M = 2.0
OFF_M = 3.0
ON_M = 1.5
OFF_FRACTION = 0.25
CLIP_S = 2.0
MIN_SPEED_MPS = 3.5
START_ON_LINE_M = 1.0
# Past this a "departure" is a corrupt GPS fix, not a kart: three of the 33131
# September fixes land hundreds of kilometres away.
MAX_PLAUSIBLE_M = 20.0


@dataclass
class Clip:
    run: str
    t0: float
    label: str          # "off" or "on"
    peak_m: float
    mean_speed: float


def distance_to_line(px, py, line_x, line_y) -> np.ndarray:
    """Perpendicular distance from each point to the closed racing line."""
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    ax, ay = np.asarray(line_x, dtype=float), np.asarray(line_y, dtype=float)
    vx, vy = np.roll(ax, -1) - ax, np.roll(ay, -1) - ay
    L2 = np.maximum(vx * vx + vy * vy, 1e-9)
    out = np.empty(len(px))
    for i, (X, Y) in enumerate(zip(px, py)):
        tt = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0.0, 1.0)
        out[i] = float(np.min(np.hypot(X - (ax + tt * vx), Y - (ay + tt * vy))))
    return out


def find_clips(t, x, y, v, line_x, line_y, stride_s: float = 0.5) -> List[Clip]:
    """Label every 2 s window of a run as a departure, on-track, or neither."""
    t = np.asarray(t, dtype=float)
    d = distance_to_line(x, y, line_x, line_y)
    clips: List[Clip] = []
    for t0 in np.arange(float(t[0]) + CLIP_S, float(t[-1]) - CLIP_S, stride_s):
        m = (t >= t0) & (t <= t0 + CLIP_S)
        if m.sum() < 25 or float(np.min(v[m])) < MIN_SPEED_MPS:
            continue
        if d[m][0] > START_ON_LINE_M:
            continue          # a clip that starts off the line tests nothing
        peak = float(d[m].max())
        out_frac = float((d[m] > HALF_WIDTH_M).mean())
        if OFF_M < peak < MAX_PLAUSIBLE_M and out_frac > OFF_FRACTION:
            label = "off"
        elif peak < ON_M:
            label = "on"
        else:
            continue
        clips.append(Clip(run="", t0=round(float(t0), 2), label=label,
                          peak_m=round(peak, 2),
                          mean_speed=round(float(np.mean(v[m])), 2)))
    return clips


def seed_window(ref, wheel, throttle, k0, history: int):
    """The HISTORY rows a plant is handed at the start of a clip.

    Taken from the fused reference, which is the frame the plant was trained
    in. Seeding from /mpc/status EKF telemetry instead charges the plant for
    the estimator: the EKF sits 0.12-0.81 m from the reference on these runs,
    which is most of a departure, and it was the difference between
    reproducing 1 of 5 departures and 3 of 3.
    """
    rows = []
    for j in range(k0 - history + 1, k0 + 1):
        rows.append((float(ref.v[j]), float(ref.omega[j]), float(wheel[j]),
                     float(throttle[j])))
    return rows


def score_clips(results: Sequence[tuple]) -> Dict[str, float]:
    """(label, sim_peak_m) pairs -> how often the sim agreed with the kart."""
    off = [p for lab, p in results if lab == "off"]
    on = [p for lab, p in results if lab == "on"]
    off_hit = sum(1 for p in off if p > HALF_WIDTH_M)
    on_hit = sum(1 for p in on if p <= HALF_WIDTH_M)
    total = len(off) + len(on)
    return {
        "off_reproduced": off_hit, "off_total": len(off),
        "on_reproduced": on_hit, "on_total": len(on),
        "agreement": (off_hit + on_hit) / total if total else float("nan"),
    }
