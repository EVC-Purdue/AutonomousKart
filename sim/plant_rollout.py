"""Open-loop rollout scoring for any plant stepper.

Every candidate plant is scored by this one function so the gray-box, the
linear baseline and the network are comparable. Error is reported at five
horizons because a one-step number is what let the current plant pass review
while being 1.64 m wrong at two seconds.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Sequence

import numpy as np

from sim.plant_dataset import HISTORY, Segment
from sim.plant_reference import PLANT_HZ

HORIZON_STEPS = (5, 10, 20, 60, 120)
DT = 1.0 / PLANT_HZ

# The one speed clamp, shared by rollout_errors, plant_train.rollout_loss and
# LearnedPlant.step. A clamp that lives in only one of the three means the
# table scores a plant the sim does not run. The ceiling is the kart's
# v_max_mps (params/pathfinder.yaml); a learned plant has no structural bound
# of its own, so without it a compounding speed error runs away unnoticed.
V_MIN, V_MAX = 0.0, 12.0


def clamp_speed(v: float) -> float:
    return min(V_MAX, max(V_MIN, v))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rollout_errors(stepper: Callable, segments: List[Segment],
                   horizons: Sequence[int] = HORIZON_STEPS,
                   splits: Sequence[str] = ("test",),
                   stride: int = 6) -> Dict[int, dict]:
    """Roll `stepper` forward from every valid start point and score it.

    The rollout is driven by the recorded commands, so the only thing under
    test is the plant. `stride` subsamples start points; at 60 Hz a stride of 6
    gives a start every 100 ms, which is far more than enough and keeps the
    evaluation quick.
    """
    longest = max(horizons)
    horizon_set = set(horizons)  # only these step counts get recorded below
    acc = {h: {"pos": [], "head": [], "speed": []} for h in horizons}
    dt = 1.0 / PLANT_HZ

    for seg in segments:
        if seg.split not in splits:
            continue
        n = len(seg.t)
        last_start = n - longest - 1
        for k0 in range(HISTORY - 1, max(HISTORY - 1, last_start + 1), stride):
            window = np.column_stack([seg.v, seg.omega, seg.delta_cmd,
                                      seg.throttle_sp])[k0 - HISTORY + 1:k0 + 1].copy()
            x, y, psi = seg.x[k0], seg.y[k0], seg.psi[k0]
            v, psi_dot = seg.v[k0], seg.omega[k0]
            for step in range(1, longest + 1):
                k = k0 + step - 1
                alpha, a_long = stepper(window, seg.delta_cmd[k], seg.throttle_sp[k])
                # Definitional integration, not a kart model: position and
                # heading advance on the OLD v and psi_dot, which is what the
                # reference itself does, then the accelerations update them.
                x += math.cos(psi) * v * DT
                y += math.sin(psi) * v * DT
                psi = _wrap(psi + psi_dot * DT)
                v = clamp_speed(v + a_long * DT)
                psi_dot = psi_dot + alpha * DT
                nxt = k0 + step
                window = np.roll(window, -1, axis=0)
                window[-1] = (v, psi_dot, seg.delta_cmd[nxt], seg.throttle_sp[nxt])
                if step in horizon_set:
                    acc[step]["pos"].append(math.hypot(x - seg.x[nxt], y - seg.y[nxt]))
                    acc[step]["head"].append(abs(_wrap(psi - seg.psi[nxt])))
                    acc[step]["speed"].append(abs(v - seg.v[nxt]))

    out = {}
    for h in horizons:
        pos = np.asarray(acc[h]["pos"])
        head = np.degrees(np.asarray(acc[h]["head"]))
        spd = np.asarray(acc[h]["speed"])
        out[h] = {
            "pos_median": float(np.median(pos)) if pos.size else float("nan"),
            "pos_p90": float(np.percentile(pos, 90)) if pos.size else float("nan"),
            "heading_median_deg": float(np.median(head)) if head.size else float("nan"),
            "heading_p90_deg": float(np.percentile(head, 90)) if head.size else float("nan"),
            "speed_median": float(np.median(spd)) if spd.size else float("nan"),
            "n": int(pos.size),
        }
    return out
