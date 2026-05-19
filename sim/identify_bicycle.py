"""Identify kinematic-bicycle + actuator parameters from aligned bag data.

See docs/superpowers/specs/2026-05-18-data-driven-sim-design.md section 5.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Tuple

import numpy as np


@dataclass
class BicycleParams:
    accel_tau: float = 0.5
    brake_tau: float = 0.3
    throttle_delay_s: float = 0.0
    steer_delay_s: float = 0.0
    steer_gain: float = 1.0      # cmd_steer_deg * steer_gain -> wheel angle deg
    steer_slop_deg: float = 0.0
    wheelbase_m: float = 1.05
    steer_rate_max_degps: float = 180.0
    slip_angle_at_v: float = 0.0  # rad/(m/s); negative reduces yaw rate at high v
    v_max_mps: float = 12.0


def _delay_shift(cmd: np.ndarray, delay_s: float, dt: float) -> np.ndarray:
    """Shift cmd backward in time by delay_s (positive delay = use older cmd)."""
    k = max(0, int(round(delay_s / dt)))
    if k == 0:
        return cmd
    out = np.empty_like(cmd)
    out[:k] = cmd[0]
    out[k:] = cmd[:-k]
    return out


def predict_derivatives(
    p: BicycleParams,
    v: np.ndarray,
    cmd_throttle_pct: np.ndarray,
    cmd_steer_deg: np.ndarray,
    dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorised one-step bicycle prediction. Returns (dv_dt_model, psi_dot_model)."""
    throttle = _delay_shift(cmd_throttle_pct, p.throttle_delay_s, dt)
    steer = _delay_shift(cmd_steer_deg, p.steer_delay_s, dt)

    target_v = np.clip(throttle / 100.0, 0.0, 1.0) * p.v_max_mps
    tau = np.where(target_v >= v, p.accel_tau, p.brake_tau)
    dv_dt_model = (target_v - v) / np.maximum(tau, 1e-3)

    slop = np.deg2rad(max(0.0, p.steer_slop_deg))
    delta_cmd_rad = np.deg2rad(p.steer_gain * steer)
    delta = np.empty_like(delta_cmd_rad)
    actual = 0.0
    rate_lim = np.deg2rad(p.steer_rate_max_degps) * dt
    for i in range(delta_cmd_rad.size):
        c = delta_cmd_rad[i]
        if c > actual + slop:
            target = c - slop
        elif c < actual - slop:
            target = c + slop
        else:
            target = actual
        delta_step = np.clip(target - actual, -rate_lim, rate_lim)
        actual += delta_step
        delta[i] = actual

    psi_dot_kin = v * np.tan(delta) / max(p.wheelbase_m, 1e-3)
    psi_dot_model = psi_dot_kin - p.slip_angle_at_v * v * np.sign(delta) * delta
    return dv_dt_model, psi_dot_model


def fit_params(
    v: np.ndarray,
    cmd_throttle: np.ndarray,
    cmd_steer: np.ndarray,
    dv_dt_obs: np.ndarray,
    psi_dot_obs: np.ndarray,
    dt: float,
    prior: BicycleParams,
    speed_floor_mps: float = 0.5,
) -> Tuple[BicycleParams, dict]:
    """Fit bicycle params by minimising weighted squared residuals on
    (dv_dt, psi_dot). Returns (fitted_params, info_dict)."""
    from scipy.optimize import least_squares

    mask = v > speed_floor_mps

    order = ("accel_tau", "brake_tau", "throttle_delay_s", "steer_delay_s",
             "steer_gain", "steer_slop_deg", "wheelbase_m",
             "steer_rate_max_degps", "slip_angle_at_v")
    x0 = np.array([getattr(prior, k) for k in order], dtype=np.float64)
    lo = np.array([0.05, 0.05, 0.0, 0.0, 0.1, 0.0, 0.95 * prior.wheelbase_m, 30.0, -0.5])
    hi = np.array([5.0,  5.0,  0.5, 0.5, 5.0, 8.0, 1.05 * prior.wheelbase_m, 500.0, 0.5])

    sigma_v_rough = max(np.std(dv_dt_obs[mask]), 0.5)
    sigma_p_rough = max(np.std(psi_dot_obs[mask]), 0.1)

    def residuals(x):
        p = BicycleParams(**{k: float(val) for k, val in zip(order, x)},
                          v_max_mps=prior.v_max_mps)
        dv_m, psi_m = predict_derivatives(p, v, cmd_throttle, cmd_steer, dt)
        r_dv = (dv_dt_obs - dv_m)[mask] / sigma_v_rough
        r_psi = (psi_dot_obs - psi_m)[mask] / sigma_p_rough
        return np.concatenate([r_dv, r_psi])

    sol = least_squares(residuals, x0, bounds=(lo, hi), method="trf", max_nfev=500)
    fitted = BicycleParams(**{k: float(val) for k, val in zip(order, sol.x)},
                           v_max_mps=prior.v_max_mps)
    info = {
        "cost": float(sol.cost), "nfev": int(sol.nfev), "success": bool(sol.success),
        "sigma_v_rough": float(sigma_v_rough), "sigma_p_rough": float(sigma_p_rough),
        "param_order": list(order), "x_final": sol.x.tolist(),
    }
    return fitted, info


def aggregate_bags(npz_paths, autonomous_only: bool = True, speed_floor: float = 0.5):
    """Concatenate aligned bags into a single training array. Returns
    (v, cmd_throttle, cmd_steer, dv_dt, psi_dot, dt)."""
    from sim.load_bags import AlignedBag
    vs, cts, css, dvs, pss = [], [], [], [], []
    dt_est = None
    for path in npz_paths:
        bag = AlignedBag.from_npz(path)
        dt_bag = float(np.median(np.diff(bag.t)))
        if dt_est is None:
            dt_est = dt_bag
        elif abs(dt_bag - dt_est) > 1e-6:
            raise ValueError(f"dt mismatch between bags: {dt_est} vs {dt_bag}")
        mask = np.ones(bag.t.size, dtype=bool)
        if autonomous_only:
            mask &= bag.autonomous
        mask &= bag.v > speed_floor
        vs.append(bag.v[mask])
        cts.append(bag.cmd_throttle[mask])
        css.append(bag.cmd_steer[mask])
        dvs.append(bag.dv_dt[mask])
        pss.append(bag.psi_dot[mask])
    return (
        np.concatenate(vs), np.concatenate(cts), np.concatenate(css),
        np.concatenate(dvs), np.concatenate(pss), dt_est,
    )


def plot_diagnostics(npz_paths, params_path: str, out_dir: str):
    import json
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from sim.load_bags import AlignedBag

    with open(params_path) as f:
        payload = json.load(f)
    p = BicycleParams(**payload["params"])
    os.makedirs(out_dir, exist_ok=True)

    for npz_path in npz_paths:
        bag = AlignedBag.from_npz(npz_path)
        mask = bag.autonomous & (bag.v > 0.5)
        if mask.sum() < 100:
            continue
        dt = float(np.median(np.diff(bag.t)))
        dv_m, psi_m = predict_derivatives(p, bag.v, bag.cmd_throttle, bag.cmd_steer, dt)

        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        t_plot = bag.t[mask]
        axes[0].plot(t_plot, bag.dv_dt[mask], color="#333", lw=0.7, label="obs dv/dt")
        axes[0].plot(t_plot, dv_m[mask], color="#c33", lw=0.7, label="model dv/dt")
        axes[0].set_ylabel("dv/dt (m/s^2)")
        axes[0].legend(loc="upper right", frameon=False)
        axes[1].plot(t_plot, bag.psi_dot[mask], color="#333", lw=0.7, label="obs psi_dot")
        axes[1].plot(t_plot, psi_m[mask], color="#37c", lw=0.7, label="model psi_dot")
        axes[1].set_ylabel("psi_dot (rad/s)")
        axes[1].set_xlabel("t (s)")
        axes[1].legend(loc="upper right", frameon=False)
        tag = os.path.splitext(os.path.basename(npz_path))[0]
        fig.suptitle(f"ID fit overlay — {tag}")
        fig.tight_layout()
        out_png = os.path.join(out_dir, f"id_overlay_{tag}.png")
        fig.savefig(out_png, dpi=120)
        plt.close(fig)
        print(f"wrote {out_png}")


def main():
    import argparse
    import json
    import os
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_fit = sub.add_parser("fit")
    p_fit.add_argument("--train", nargs="+", required=True)
    p_fit.add_argument("--out", required=True)
    p_fit.add_argument("--v-max", type=float, default=12.0)
    p_fit.add_argument("--wheelbase", type=float, default=1.05)

    p_plot = sub.add_parser("plot")
    p_plot.add_argument("--bags", nargs="+", required=True)
    p_plot.add_argument("--params", required=True)
    p_plot.add_argument("--out-dir", required=True)

    args = ap.parse_args()
    if args.cmd == "fit":
        v, ct, cs, dv, ps, dt = aggregate_bags(args.train)
        print(f"training on {v.size} ticks at dt={dt:.4f}s")
        prior = BicycleParams(v_max_mps=args.v_max, wheelbase_m=args.wheelbase)
        fit, info = fit_params(v, ct, cs, dv, ps, dt, prior=prior)
        out_payload = {"params": asdict(fit), "fit_info": info,
                       "training_bags": list(args.train)}
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(out_payload, f, indent=2)
        print(f"wrote {args.out}")
        for k, v_ in asdict(fit).items():
            print(f"  {k}: {v_:.4f}")
    elif args.cmd == "plot":
        plot_diagnostics(args.bags, args.params, args.out_dir)


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
