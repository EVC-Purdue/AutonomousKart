"""Acceptance tables for the learned plant.

Reports rollout error at five horizons for the gray-box, the linear baseline
and the network, so the spec's gate can be read off one output. The network's
weights are trained separately (sim/plant_train.py) and may not exist yet;
this report must still run and produce the other two tables when they don't.
"""
from __future__ import annotations

import math
import os
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from sim.learned_plant import LearnedPlant
from sim.plant_dataset import HISTORY, Segment, Whitener, mirror
from sim.plant_models import LinearARX, as_stepper
from sim.plant_reference import PLANT_HZ
from sim.plant_rollout import HORIZON_STEPS, clamp_speed, rollout_errors
from sim.plant_train import pooled_windows


def rollout_table(named_steppers: Dict[str, Callable], segments: List[Segment],
                  splits=("test",)) -> str:
    lines = [f"{'plant':<18}{'steps':>7}{'sec':>7}{'pos med':>10}{'pos p90':>10}"
             f"{'head med':>10}{'speed med':>11}{'n':>8}"]
    for name, stepper in named_steppers.items():
        out = rollout_errors(stepper, segments, splits=splits)
        for h in HORIZON_STEPS:
            r = out[h]
            lines.append(f"{name:<18}{h:>7}{h / 60.0:>7.3f}{r['pos_median']:>10.3f}"
                         f"{r['pos_p90']:>10.3f}{r['heading_median_deg']:>10.2f}"
                         f"{r['speed_median']:>11.3f}{r['n']:>8}")
    return "\n".join(lines)


def fit_linear(segments: List[Segment]) -> Callable:
    """LinearARX fit on the pooled train split, with mirror augmentation and
    whitening, wrapped into the (window, delta_cmd, throttle_sp) stepper form."""
    feats, tg = pooled_windows(segments, splits=("train",))
    mf, mt = mirror(feats, tg)
    feats = np.concatenate([feats, mf])
    tg = np.concatenate([tg, mt])
    fw, tw = Whitener().fit(feats), Whitener().fit(tg)
    linear = LinearARX().fit(fw.apply(feats), tw.apply(tg))
    return as_stepper(linear, fw, tw)


def graybox_stepper(dt: float = 1.0 / PLANT_HZ) -> Callable:
    """The shipped gray-box, driven through the same stepper contract.

    Spec section 11 requires all three candidates on the same windows. The
    0.66 m / 5.1 deg bar in section 2 was measured by a different method, so
    without this row the table has nothing comparable to beat.

    `DataSim` carries one piece of state the feature window does not name,
    the actual wheel angle, so it is reconstructed from the window's yaw rate
    through the same kinematic relation DataSim integrates, and the sim is
    stepped one tick from the window's last row. One instance is reused and
    re-seeded per call: the contract is stateless, and nothing may survive
    between calls.
    """
    from sim.data_sim import DataSim
    sim = DataSim()

    def stepper(window, delta_cmd, throttle_sp):
        v0 = float(window[-1, 0])
        psi_dot = float(window[-1, 1])
        sim.reset(0.0, 0.0, 0.0, v=v0)
        L = sim.params.wheelbase_m
        if L > 1e-6 and abs(v0) > 1e-3:
            sim._delta_actual_rad = math.atan(psi_dot * L / v0)
        sim.step(float(throttle_sp), float(delta_cmd), dt)
        # The contract is accelerations now, so read back the yaw rate DataSim
        # produced over the tick and difference both channels.
        psi_dot_next = sim.yaw / dt
        return (psi_dot_next - psi_dot) / dt, (sim.v - v0) / dt
    return stepper


def load_learned(weights_path: str, meta_path: str) -> Optional[Callable]:
    """The network as a stepper, or None if `weights_path` doesn't exist.

    Training is deliberately deferred, so sim/model/plant_nn.pt may not be
    on disk yet -- that is not an error here.
    """
    if not os.path.isfile(weights_path):
        return None
    plant = LearnedPlant.from_files(weights_path, meta_path)

    def nn_stepper(window, delta_cmd, throttle_sp):
        # The same hull accounting `step` does. Predicting straight off
        # `_predict` bypasses it, and spec section 12's promise that a query
        # outside the data announces itself then holds nowhere the table can
        # see. `plant` is attached so `print_report` can read the count.
        plant._check_hull(window[-1])
        return tuple(plant._predict(window))

    nn_stepper.plant = plant
    return nn_stepper


# Central-difference probes, both a size a driver would actually ask for,
# held over a third of a second: one tick is shorter than the gray-box's
# 33.7 deg/s wheel slew limit needs to separate a +2 deg command from a -2 deg
# one, so a single-step probe reads exactly zero for a plant that does steer.
STEER_PROBE_DEG = 2.0
THROTTLE_PROBE_MPS = 1.0
SENSITIVITY_STEPS = 20


def _perturbed_rollout(stepper, seg, per_tick, k0, steps, d_off, th_off):
    """Yaw rate and speed after `steps` ticks with every command offset.

    The offset rides on the recorded commands rather than replacing them, so
    the probe is a perturbation of the trajectory the kart actually drove.
    """
    window = per_tick[k0 - HISTORY + 1:k0 + 1].copy()
    v, psi_dot = float(seg.v[k0]), float(seg.omega[k0])
    for step in range(1, steps + 1):
        k = k0 + step - 1
        window[-1, 2] = seg.delta_cmd[k] + d_off
        window[-1, 3] = seg.throttle_sp[k] + th_off
        alpha, a_long = stepper(window, window[-1, 2], window[-1, 3])
        v = clamp_speed(v + a_long / PLANT_HZ)
        psi_dot += alpha / PLANT_HZ
        nxt = k0 + step
        window = np.roll(window, -1, axis=0)
        window[-1] = (v, psi_dot, seg.delta_cmd[nxt] + d_off,
                      seg.throttle_sp[nxt] + th_off)
    return psi_dot, v


def command_sensitivity(stepper, segments: List[Segment], splits=("test",),
                        stride: int = 60,
                        steps: int = SENSITIVITY_STEPS) -> Dict[str, float]:
    """How far the plant's state moves when its commands move.

    Rollout error cannot see a plant that ignores its inputs: both plants
    fitted against the old `dpsi` and `ds` targets scored well at every
    horizon by copying the window's own yaw rate and speed forward, which is
    what those targets were. This drives the commands directly, one central
    difference per start tick: `steer` is the yaw rate a degree of extra
    steering buys after `steps` ticks, in rad/s per degree, and `throttle` is
    the speed an extra m/s of setpoint buys, dimensionless. The median over
    start ticks, so one saturated corner cannot carry the number.
    """
    steer, throttle = [], []
    for seg in segments:
        if seg.split not in splits:
            continue
        per_tick = np.column_stack([seg.v, seg.omega, seg.delta_cmd,
                                    seg.throttle_sp])
        for k0 in range(HISTORY - 1, len(seg.t) - steps, stride):
            args = (stepper, seg, per_tick, k0, steps)
            w_hi, _ = _perturbed_rollout(*args, STEER_PROBE_DEG, 0.0)
            w_lo, _ = _perturbed_rollout(*args, -STEER_PROBE_DEG, 0.0)
            _, v_hi = _perturbed_rollout(*args, 0.0, THROTTLE_PROBE_MPS)
            _, v_lo = _perturbed_rollout(*args, 0.0, -THROTTLE_PROBE_MPS)
            steer.append((w_hi - w_lo) / (2.0 * STEER_PROBE_DEG))
            throttle.append((v_hi - v_lo) / (2.0 * THROTTLE_PROBE_MPS))
    return {"steer": float(np.median(steer)) if steer else float("nan"),
            "throttle": float(np.median(throttle)) if throttle else float("nan")}


def sensitivity_table(named_steppers: Dict[str, Callable],
                      segments: List[Segment], splits=("test",)) -> str:
    lines = [f"{'plant':<18}{'d(psi_dot)/d(steer)':>22}{'d(v)/d(throttle)':>20}",
             f"{'':<18}{'rad/s per deg':>22}{'m/s per m/s':>20}"]
    for name, stepper in named_steppers.items():
        s = command_sensitivity(stepper, segments, splits=splits)
        lines.append(f"{name:<18}{s['steer']:>22.5f}{s['throttle']:>20.5f}")
    return "\n".join(lines)


def build_candidates(segments: List[Segment], weights_path: str,
                     meta_path: str) -> Tuple[Dict[str, Callable], bool]:
    """The gray-box and the linear baseline, plus the network when its
    weights are on disk."""
    candidates: Dict[str, Callable] = {"graybox": graybox_stepper(),
                                       "linear_arx": fit_linear(segments)}
    nn_stepper = load_learned(weights_path, meta_path)
    if nn_stepper is not None:
        candidates["learned_nn"] = nn_stepper
    return candidates, nn_stepper is not None


def _print_hull(candidates: Dict[str, Callable]) -> None:
    """How much of a table was scored outside the training hull.

    Reset per table so the May check reports its own rate rather than
    September's plus May's.
    """
    for name, stepper in candidates.items():
        plant = getattr(stepper, "plant", None)
        if plant is None:
            continue
        pct = 100.0 * plant.out_of_hull / max(plant.queries, 1)
        print(f"{name}: {plant.out_of_hull} of {plant.queries} queries "
              f"({pct:.1f}%) outside the training hull")
        plant.out_of_hull = plant.queries = 0


def print_report(segments: List[Segment], weights_path: str, meta_path: str,
                 aux_segments: Optional[List[Segment]] = None) -> None:
    candidates, have_nn = build_candidates(segments, weights_path, meta_path)
    if not have_nn:
        print(f"learned_nn weights not found at {weights_path} "
              f"(training deliberately deferred) -- reporting graybox and "
              f"linear_arx only.")

    print("=== September test windows ===")
    print(rollout_table(candidates, segments, splits=("test",)))
    _print_hull(candidates)

    print(f"\n=== command sensitivity over {SENSITIVITY_STEPS} ticks, "
          f"September test windows ===")
    print(sensitivity_table(candidates, segments, splits=("test",)))

    if aux_segments:
        # The auxiliary May bags don't share the September split assignment
        # and gate nothing, so every segment is scored regardless of the
        # train/val/test tag assign_splits gave it.
        for seg in aux_segments:
            seg.split = "test"
        print("\n=== auxiliary May check, gates nothing ===")
        print(rollout_table(candidates, aux_segments, splits=("test",)))
        _print_hull(candidates)


def main() -> None:
    import argparse

    from sim.plant_dataset import build_segments
    from sim.sensor_noise import SensorNoiseModel

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--sensors", default="sim/model/sensors.json")
    ap.add_argument("--weights", default="sim/model/plant_nn.pt")
    ap.add_argument("--meta", default="sim/model/plant_nn.json")
    ap.add_argument("--aux-runs", nargs="*", default=[])
    args = ap.parse_args()

    noise = SensorNoiseModel.load(args.sensors)
    segments = build_segments(args.runs, noise)
    aux_segments = build_segments(args.aux_runs, noise) if args.aux_runs else None
    print_report(segments, args.weights, args.meta, aux_segments)


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
