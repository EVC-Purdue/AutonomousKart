"""Recursive least-squares residual learner for the MPC planner.

Runs in shadow mode by default — trains/predicts/logs but does NOT affect
control output. The MPC planner consults `predict` only when mode == "apply".

The learner predicts the residual delta_s / delta_d between the nominal
bicycle-model rollout and the actual kart motion over a target horizon
(default 0.5 s). A delayed sample with the actual measured displacement
arrives `horizon_steps` ticks later, at which point we update theta with
the standard RLS recursion using a shared P matrix.
"""

from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np


def _features(d: float, v_s: float, v_d: float, kappa: float,
              steer_hist: Tuple[float, float, float, float],
              throttle_hist: Tuple[float, float, float],
              nom_ds: float, nom_dd: float) -> np.ndarray:
    return np.array([
        1.0, d, v_s, v_d, kappa,
        steer_hist[0], steer_hist[1], steer_hist[2], steer_hist[3],
        throttle_hist[0], throttle_hist[1], throttle_hist[2],
        nom_ds, nom_dd,
    ], dtype=np.float64)


NUM_FEATURES = 14


class ResidualLearner:
    def __init__(self, params: dict, solve_dt: float):
        """solve_dt is the wall-time between successive plan() calls
        (= 1/system_frequency), used to size the push/step delay."""
        self.mode = str(params.get("mode", "shadow")).lower()  # off | shadow | apply
        self.target_horizon_s = float(params.get("target_horizon_s", 0.5))
        self.horizon_steps = max(1, int(round(self.target_horizon_s / max(solve_dt, 1e-3))))
        self.lam = float(params.get("forgetting_factor", 0.99))
        self.min_speed = float(params.get("min_train_speed_mps", 0.5))
        self.err_window = int(params.get("error_window", 200))

        p0 = float(params.get("initial_cov", 1000.0))
        self.theta_s = np.zeros(NUM_FEATURES)
        self.theta_d = np.zeros(NUM_FEATURES)
        self.P = np.eye(NUM_FEATURES) * p0

        # Buffer holds the last `horizon_steps` snapshots; pop when target arrives.
        self._pending: Deque[Tuple[np.ndarray, float, float, float, float]] = deque(
            maxlen=self.horizon_steps + 1
        )

        # Rolling errors for shadow-vs-nominal diagnostics.
        self._err_nom_s: Deque[float] = deque(maxlen=self.err_window)
        self._err_nom_d: Deque[float] = deque(maxlen=self.err_window)
        self._err_res_s: Deque[float] = deque(maxlen=self.err_window)
        self._err_res_d: Deque[float] = deque(maxlen=self.err_window)

        self.last_pred_s = 0.0
        self.last_pred_d = 0.0
        self.samples_trained = 0

    @property
    def enabled(self) -> bool:
        return self.mode in ("shadow", "apply")

    def push(self, phi: np.ndarray, s_t: float, d_t: float,
             nom_ds: float, nom_dd: float, speed: float) -> None:
        """Stash current features + Frenet pose so we can score it `horizon_steps`
        ticks later when the actual delta is known."""
        if not self.enabled:
            return
        if speed < self.min_speed:
            # Still buffer for delay alignment, but mark invalid by NaN in phi[0]?
            # Simpler: only push valid samples; drop intermittently below threshold.
            return
        self._pending.append((phi.copy(), s_t, d_t, nom_ds, nom_dd))

    def step(self, s_now: float, d_now: float) -> None:
        """Pop the oldest pending sample (if it has aged H ticks) and run RLS."""
        if not self.enabled or len(self._pending) <= self.horizon_steps:
            return
        phi, s0, d0, nom_ds, nom_dd = self._pending.popleft()
        actual_ds = s_now - s0
        actual_dd = d_now - d0
        # Residual targets
        r_s = actual_ds - nom_ds
        r_d = actual_dd - nom_dd

        # Errors for diagnostics (before update)
        pred_s = float(phi @ self.theta_s)
        pred_d = float(phi @ self.theta_d)
        self._err_nom_s.append(abs(actual_ds - nom_ds))
        self._err_nom_d.append(abs(actual_dd - nom_dd))
        self._err_res_s.append(abs(actual_ds - (nom_ds + pred_s)))
        self._err_res_d.append(abs(actual_dd - (nom_dd + pred_d)))

        # RLS recursion (shared P)
        Pphi = self.P @ phi
        denom = self.lam + float(phi @ Pphi)
        if denom < 1e-9:
            return
        K = Pphi / denom
        self.theta_s += K * (r_s - float(phi @ self.theta_s))
        self.theta_d += K * (r_d - float(phi @ self.theta_d))
        self.P = (self.P - np.outer(K, Pphi)) / self.lam
        self.samples_trained += 1

    def predict(self, phi: np.ndarray) -> Tuple[float, float]:
        if not self.enabled:
            self.last_pred_s = 0.0
            self.last_pred_d = 0.0
            return 0.0, 0.0
        self.last_pred_s = float(phi @ self.theta_s)
        self.last_pred_d = float(phi @ self.theta_d)
        return self.last_pred_s, self.last_pred_d

    def mean_error(self) -> Tuple[float, float, float, float]:
        def _mean(d): return float(np.mean(d)) if d else 0.0

        return (_mean(self._err_nom_s), _mean(self._err_nom_d),
                _mean(self._err_res_s), _mean(self._err_res_d))
