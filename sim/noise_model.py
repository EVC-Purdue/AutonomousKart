"""Empirical noise / error model for the data-driven sim. Section 5 of the spec."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class NoiseModel:
    sigma_v: float
    sigma_psidot: float
    sigma_steer_cmd: float
    sigma_throttle_cmd: float

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "NoiseModel":
        with open(path) as f:
            return cls(**json.load(f))


def estimate_noise(
    v_residual: np.ndarray,
    psi_dot_residual: np.ndarray,
    cmd_throttle: np.ndarray,
    cmd_steer: np.ndarray,
    is_straight: np.ndarray,
    is_steady_throttle: np.ndarray,
) -> NoiseModel:
    sigma_v = float(np.std(v_residual))
    sigma_psidot = float(np.std(psi_dot_residual))
    sigma_steer_cmd = _jitter_std(cmd_steer, mask=is_straight, window=15)
    sigma_throttle_cmd = _jitter_std(cmd_throttle, mask=is_steady_throttle, window=15)
    return NoiseModel(sigma_v=sigma_v, sigma_psidot=sigma_psidot,
                      sigma_steer_cmd=sigma_steer_cmd,
                      sigma_throttle_cmd=sigma_throttle_cmd)


def _jitter_std(x: np.ndarray, mask: np.ndarray, window: int) -> float:
    if mask.sum() < window * 2:
        return float(np.std(x))
    kernel = np.ones(window) / window
    smoothed = np.convolve(x, kernel, mode="same")
    return float(np.std((x - smoothed)[mask]))


def compute_pre_mlp_artifacts(
    train_npz_paths,
    params_path: str,
    out_noise_json: str,
    out_residual_emp: str,
) -> None:
    """Predict bicycle derivatives on the training bags, compute residuals,
    write the noise model and the empirical residual pool used for bootstrap."""
    import os
    from sim.identify_bicycle import BicycleParams, predict_derivatives
    from sim.load_bags import AlignedBag

    with open(params_path) as f:
        p = BicycleParams(**json.load(f)["params"])

    v_all, dv_res_all, psi_res_all = [], [], []
    ct_all, cs_all = [], []
    is_straight_all, is_steady_all = [], []
    for path in train_npz_paths:
        bag = AlignedBag.from_npz(path)
        mask = bag.autonomous & (bag.v > 0.5)
        if mask.sum() < 100:
            continue
        dt = float(np.median(np.diff(bag.t)))
        dv_m, psi_m = predict_derivatives(p, bag.v, bag.cmd_throttle, bag.cmd_steer, dt)
        dv_res_all.append(bag.dv_dt[mask] - dv_m[mask])
        psi_res_all.append(bag.psi_dot[mask] - psi_m[mask])
        ct_all.append(bag.cmd_throttle[mask])
        cs_all.append(bag.cmd_steer[mask])
        v_all.append(bag.v[mask])
        is_straight_all.append(np.abs(bag.cmd_steer[mask]) < 2.0)
        d_throttle = np.abs(np.diff(bag.cmd_throttle[mask], prepend=bag.cmd_throttle[mask][0]))
        is_steady_all.append(d_throttle < 2.0)

    dv_res = np.concatenate(dv_res_all)
    psi_res = np.concatenate(psi_res_all)
    ct = np.concatenate(ct_all)
    cs = np.concatenate(cs_all)
    v_arr = np.concatenate(v_all)
    is_straight = np.concatenate(is_straight_all)
    is_steady = np.concatenate(is_steady_all)
    nm = estimate_noise(dv_res, psi_res, ct, cs, is_straight, is_steady)
    nm.save(out_noise_json)
    print(f"wrote {out_noise_json}: {asdict(nm)}")

    v_edges = np.linspace(0.0, 15.0, 6)
    cs_edges = np.linspace(0.0, 30.0, 4)
    v_bin = np.clip(np.digitize(v_arr, v_edges) - 1, 0, v_edges.size - 2)
    cs_bin = np.clip(np.digitize(np.abs(cs), cs_edges) - 1, 0, cs_edges.size - 2)
    np.savez_compressed(
        out_residual_emp,
        dv_res=dv_res, psi_res=psi_res,
        v_bin=v_bin, cs_bin=cs_bin,
        v_edges=v_edges, cs_edges=cs_edges,
    )
    print(f"wrote {out_residual_emp}: {dv_res.size} samples")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-noise", required=True)
    ap.add_argument("--out-residual-emp", required=True)
    args = ap.parse_args()
    compute_pre_mlp_artifacts(args.train, args.params, args.out_noise, args.out_residual_emp)


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
