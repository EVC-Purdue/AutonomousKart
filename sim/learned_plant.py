"""Runtime plant backed by the learned model.

Exposes exactly the interface sim/data_sim.py's DataSim does, so
sim/closed_loop.py can swap backends without knowing which is which. The model
integrates at a fixed 60 Hz; a longer dt is sub-stepped, and the command is held
across the sub-steps, which is what the kart sees too.
"""
from __future__ import annotations

import json
import math
from collections import deque

import numpy as np

from sim.plant_dataset import (FEATURE_NAMES, HISTORY, STEER_RATE_MAX_DEGPS,
                               Whitener)
from sim.plant_rollout import clamp_speed


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LearnedPlant:
    def __init__(self, ensemble, meta: dict):
        self.ensemble = ensemble
        self.meta = meta
        self.hz = float(meta.get("plant_hz", 60.0))
        self.dt = 1.0 / self.hz
        self.fw = Whitener.from_dict(meta["feature_whitener"])
        self.tw = Whitener.from_dict(meta["target_whitener"])
        self.coverage = meta.get("coverage", {})
        self.out_of_hull = 0
        self.queries = 0
        self.x = self.y = self.yaw = self.v = self.psi_dot = 0.0
        self.wheel_deg = 0.0
        self._hist = deque(maxlen=HISTORY)

    @classmethod
    def from_meta(cls, meta_path: str, ensemble=None) -> "LearnedPlant":
        with open(meta_path) as f:
            meta = json.load(f)
        return cls(ensemble, meta)

    @classmethod
    def from_linear(cls, meta_path: str) -> "LearnedPlant":
        """Load a linear ARX plant, which shares this runtime exactly.

        `LinearARX` exposes the same `.predict` the ensemble does, so the
        whitening, hull check, integration and sub-stepping are all the code
        already exercised for the network. Five-fold CV put the linear model at
        0.169 +/- 0.062 m at one second against the network's 0.232 +/- 0.127
        and the gray-box's 0.861 +/- 0.281, so it ships on equal accuracy, half
        the variance and 84 parameters instead of 2072.
        """
        from sim.plant_models import LinearARX
        with open(meta_path) as f:
            meta = json.load(f)
        return cls(LinearARX.from_dict(meta["linear"]), meta)

    @classmethod
    def from_files(cls, weights_path: str, meta_path: str) -> "LearnedPlant":
        from sim.plant_models import PlantEnsemble
        return cls.from_meta(meta_path, PlantEnsemble.load(weights_path))

    @property
    def speed(self) -> float:
        return self.v

    def reset(self, x: float, y: float, yaw: float, v: float = 0.0,
              history=None) -> None:
        """Place the kart and seed the window the model reads.

        The model's whole input is the last HISTORY ticks, so a reset that
        leaves them at zero hands a kart doing 6 m/s a window that says it is
        stopped. `v` is the speed to start at; with `history` left None the
        window is HISTORY rows of steady cruise at `v` -- no yaw rate, no
        steer, and throttle setpoint equal to `v`, since the throttle command
        is a speed setpoint and at steady state it equals the speed.

        `history` overrides that with the rows a handoff actually knows,
        oldest first, each (v, psi_dot, delta_cmd, throttle_sp).

        The default, v = 0, is a standing start, and its window sits outside
        the training hull -- segments are cut on motion above 1.5 m/s, so the
        model has never seen a stationary kart. `out_of_hull` counts it.
        """
        self.x, self.y, self.yaw, self.v = float(x), float(y), float(yaw), float(v)
        # psi_dot is plant state now, not just a feature: the model predicts its
        # derivative, so a reset has to say what it starts at.
        self.psi_dot = float(history[-1][1]) if history is not None else 0.0
        # The wheel starts where the handoff says it is, not centred.
        self.wheel_deg = float(history[-1][2]) if history is not None else 0.0
        rows = ([(self.v, 0.0, 0.0, self.v)] * HISTORY if history is None
                else [tuple(float(c) for c in row) for row in history])
        if len(rows) != HISTORY:
            raise ValueError(f"history needs {HISTORY} rows, got {len(rows)}")
        self._hist.clear()
        for row in rows:
            self._hist.append(row)

    def _check_hull(self, tick) -> bool:
        """Count one query against the training hull; True when it falls outside.

        Spec section 12 asks that a query outside the data announce itself, so
        both the count and the total are kept: an extrapolation rate is what a
        reader of the acceptance tables can act on.
        """
        self.queries += 1
        for name, value in zip(FEATURE_NAMES, tick):
            lo_hi = self.coverage.get(name)
            if lo_hi is not None and not (lo_hi[0] <= value <= lo_hi[1]):
                self.out_of_hull += 1
                return True
        return False

    def _predict(self, window: np.ndarray) -> np.ndarray:
        z = self.fw.apply(window[None, ...])
        return self.tw.invert(self.ensemble.predict(z))[0]

    def step(self, target_mps: float, steer_deg: float, dt: float):
        n_sub = max(1, int(round(dt / self.dt)))
        for _ in range(n_sub):
            # The command being applied belongs to the current tick, which is
            # the window's last row. Training and rollout pair them the same way.
            v_now, psi_dot_now = self._hist[-1][0], self._hist[-1][1]
            # The model's steering feature is the wheel angle, not the
            # request. `slew_limit` records the wheel before advancing it, so
            # the tick is predicted at the angle the wheel currently holds and
            # the actuator moves afterwards; predicting at the advanced angle
            # would hand the model a window one tick ahead of training's.
            self._hist[-1] = (v_now, psi_dot_now, self.wheel_deg, float(target_mps))
            self._check_hull(self._hist[-1])
            window = np.asarray(self._hist, dtype=float)
            alpha, a_long = self._predict(window)
            # Definitional integration: position and heading advance on the old
            # v and psi_dot, exactly as the reference does, then the learned
            # accelerations update them.
            self.x += math.cos(self.yaw) * self.v * self.dt
            self.y += math.sin(self.yaw) * self.v * self.dt
            self.yaw = _wrap(self.yaw + self.psi_dot * self.dt)
            self.v = clamp_speed(self.v + a_long * self.dt)
            self.psi_dot = self.psi_dot + alpha * self.dt
            step_max = STEER_RATE_MAX_DEGPS * self.dt
            self.wheel_deg += float(np.clip(float(steer_deg) - self.wheel_deg,
                                            -step_max, step_max))
            # The next tick's command is unknown until the next call, so carry
            # this one forward; the next call overwrites it before predicting.
            self._hist.append((self.v, self.psi_dot,
                               self.wheel_deg, float(target_mps)))
        return self.x, self.y, self.yaw, self.v
