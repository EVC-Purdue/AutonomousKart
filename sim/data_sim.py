"""Runtime: identified-bicycle simulator with the same step() interface as
sim_bicycle.BicycleModel. MLP residual is added in Task 12."""
from __future__ import annotations

import json
import math
from collections import deque
from typing import Deque, Tuple

import numpy as np

from sim.identify_bicycle import BicycleParams
from sim.load_bags import NUM_STEER_HIST, NUM_THROTTLE_HIST
from sim.noise_model import NoiseModel


class DataSim:
    def __init__(
        self,
        params_path: str,
        noise_path: str,
        mlp_path: str | None = None,
        norm_path: str | None = None,
        clamp_path: str | None = None,
        residual_emp_path: str | None = None,
        noise_mode: str = "none",
        rng_seed: int = 0,
    ):
        with open(params_path) as f:
            self.params = BicycleParams(**json.load(f)["params"])
        self.noise = NoiseModel.load(noise_path)
        if noise_mode not in ("none", "gaussian", "bootstrap"):
            raise ValueError(f"unknown noise_mode {noise_mode!r}")
        self.noise_mode = noise_mode
        self.rng = np.random.default_rng(rng_seed)

        self._mlp = None
        self._norm = None
        self._clamp = None
        self._emp = None
        if mlp_path is not None and norm_path is not None and clamp_path is not None:
            self._load_mlp(mlp_path, norm_path, clamp_path)
        if residual_emp_path is not None:
            self._emp = np.load(residual_emp_path)

        self.x = self.y = self.yaw = self.v = 0.0
        self._delta_actual_rad = 0.0
        self._cmd_throttle_hist: Deque[float] = deque(maxlen=NUM_THROTTLE_HIST)
        self._cmd_steer_hist: Deque[float] = deque(maxlen=NUM_STEER_HIST)
        for _ in range(NUM_THROTTLE_HIST):
            self._cmd_throttle_hist.append(0.0)
        for _ in range(NUM_STEER_HIST):
            self._cmd_steer_hist.append(0.0)

    @property
    def speed(self) -> float:
        """Alias for v — matches BicycleModel.speed attribute used by run_sim."""
        return self.v

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.v = 0.0
        self._delta_actual_rad = 0.0

    def _load_mlp(self, mlp_path, norm_path, clamp_path):
        import torch
        import json
        from sim.clamp import Clamp
        from sim.residual_mlp import Normaliser, ResidualMLP
        self._mlp = ResidualMLP()
        self._mlp.load_state_dict(torch.load(mlp_path, map_location="cpu"))
        self._mlp.train(False)  # equivalent to net.eval() — sets inference mode
        with open(norm_path) as f:
            self._norm = Normaliser.from_json(json.load(f))
        self._clamp = Clamp.load(clamp_path)
        self._torch = torch

    def _draw_noise(self) -> Tuple[float, float]:
        if self.noise_mode == "none":
            return 0.0, 0.0
        if self.noise_mode == "gaussian" or self._emp is None:
            return (
                self.rng.normal(0.0, self.noise.sigma_v),
                self.rng.normal(0.0, self.noise.sigma_psidot),
            )
        v_edges = self._emp["v_edges"]
        cs_edges = self._emp["cs_edges"]
        last_steer_abs = abs(self._cmd_steer_hist[-1])
        vb = int(np.clip(np.digitize(self.v, v_edges) - 1, 0, v_edges.size - 2))
        cb = int(np.clip(np.digitize(last_steer_abs, cs_edges) - 1, 0, cs_edges.size - 2))
        mask = (self._emp["v_bin"] == vb) & (self._emp["cs_bin"] == cb)
        if mask.sum() < 32:
            return (
                self.rng.normal(0.0, self.noise.sigma_v),
                self.rng.normal(0.0, self.noise.sigma_psidot),
            )
        idx = self.rng.integers(0, mask.sum())
        return (
            float(self._emp["dv_res"][mask][idx]),
            float(self._emp["psi_res"][mask][idx]),
        )

    def step(self, target_mps: float, steer_deg: float, dt: float) -> Tuple[float, float, float, float]:
        p = self.params
        target_v = max(0.0, min(p.v_max_mps, float(target_mps)))

        tau = p.accel_tau if target_v >= self.v else p.brake_tau
        alpha = dt / (tau + dt)
        dv_struct = alpha * (target_v - self.v) / max(dt, 1e-6)

        delta_cmd_rad = math.radians(p.steer_gain * float(steer_deg))
        slop = math.radians(max(0.0, p.steer_slop_deg))
        if delta_cmd_rad > self._delta_actual_rad + slop:
            target_delta = delta_cmd_rad - slop
        elif delta_cmd_rad < self._delta_actual_rad - slop:
            target_delta = delta_cmd_rad + slop
        else:
            target_delta = self._delta_actual_rad
        rate_lim = math.radians(p.steer_rate_max_degps) * dt
        step_delta = max(-rate_lim, min(rate_lim, target_delta - self._delta_actual_rad))
        self._delta_actual_rad += step_delta
        delta = self._delta_actual_rad

        if p.wheelbase_m > 1e-6:
            psi_dot_struct = self.v * math.tan(delta) / p.wheelbase_m
        else:
            psi_dot_struct = 0.0
        psi_dot_struct -= p.slip_angle_at_v * self.v * (1.0 if delta >= 0 else -1.0) * delta

        if self._mlp is not None:
            from sim.residual_mlp import build_features
            feat = build_features(
                np.array([self.v]),
                np.array([psi_dot_struct]),
                np.array([float(target_mps)]),
                np.array([float(steer_deg)]),
                np.asarray(self._cmd_throttle_hist)[None, :],
                np.asarray(self._cmd_steer_hist)[None, :],
            )
            feat_n = self._norm.apply(feat)
            alpha = self._clamp.alpha(feat_n[0])
            with self._torch.no_grad():
                pred = self._mlp(self._torch.from_numpy(feat_n.astype(np.float32))).numpy()[0]
            d_dv = float(alpha * pred[0])
            d_psi = float(alpha * pred[1])
        else:
            d_dv = 0.0
            d_psi = 0.0

        # Noise — see _draw_noise.
        n_dv, n_psi = self._draw_noise()

        self.v += (dv_struct + d_dv + n_dv) * dt
        self.v = max(0.0, min(p.v_max_mps, self.v))
        psi_dot = psi_dot_struct + d_psi + n_psi
        self.yaw += psi_dot * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        self._cmd_throttle_hist.append(float(target_mps))
        self._cmd_steer_hist.append(float(steer_deg))

        return self.x, self.y, self.yaw, self.v
