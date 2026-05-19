"""Off-distribution Mahalanobis clamp for the residual MLP."""
from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np


@dataclass
class Clamp:
    mean: np.ndarray
    cov_inv: np.ndarray
    d_warn: float
    d_max: float

    def _alpha_from_d(self, d: float) -> float:
        if d <= self.d_warn:
            return 1.0
        if d >= self.d_max:
            return 0.0
        return 1.0 - (d - self.d_warn) / (self.d_max - self.d_warn)

    def alpha(self, feat_normalised: np.ndarray) -> float:
        delta = feat_normalised - self.mean
        d = float(np.sqrt(delta @ self.cov_inv @ delta))
        return self._alpha_from_d(d)

    def save(self, path: str) -> None:
        payload = {
            "mean": self.mean.tolist(),
            "cov_inv": self.cov_inv.tolist(),
            "d_warn": self.d_warn,
            "d_max": self.d_max,
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)

    @classmethod
    def load(cls, path: str) -> "Clamp":
        with open(path) as f:
            d = json.load(f)
        return cls(
            mean=np.asarray(d["mean"], dtype=np.float64),
            cov_inv=np.asarray(d["cov_inv"], dtype=np.float64),
            d_warn=float(d["d_warn"]),
            d_max=float(d["d_max"]),
        )


def fit_clamp(feats_normalised: np.ndarray, d_warn_pct=95, d_max_pct=99.5) -> Clamp:
    mean = feats_normalised.mean(0)
    cov = np.cov(feats_normalised.T)
    cov += np.eye(cov.shape[0]) * 1e-6
    cov_inv = np.linalg.inv(cov)
    delta = feats_normalised - mean
    d = np.sqrt(np.einsum("ni,ij,nj->n", delta, cov_inv, delta))
    d_warn = float(np.percentile(d, d_warn_pct))
    d_max = float(np.percentile(d, d_max_pct))
    return Clamp(mean=mean, cov_inv=cov_inv, d_warn=d_warn, d_max=d_max)


def fit_and_write(
    train_npz_paths,
    params_path: str,
    mlp_path: str,
    norm_path: str,
    out_clamp_path: str,
    out_noise_post_path: str,
    out_residual_emp_post: str,
) -> None:
    import json as _json
    import torch
    from sim.identify_bicycle import BicycleParams, predict_derivatives
    from sim.load_bags import AlignedBag
    from sim.noise_model import NoiseModel, estimate_noise
    from sim.residual_mlp import (
        FEATURE_DIM,
        Normaliser,
        ResidualMLP,
        build_features,
    )

    with open(params_path) as f:
        params = BicycleParams(**_json.load(f)["params"])
    with open(norm_path) as fn:
        norm = Normaliser.from_json(_json.load(fn))
    net = ResidualMLP()
    net.load_state_dict(torch.load(mlp_path, map_location="cpu"))
    net.eval()

    feats_all, dv_res_post, psi_res_post = [], [], []
    ct_all, cs_all, v_all = [], [], []
    is_straight_all, is_steady_all = [], []
    for path in train_npz_paths:
        bag = AlignedBag.from_npz(path)
        mask = bag.autonomous & (bag.v > 0.5)
        if mask.sum() < 100:
            continue
        dt = float(np.median(np.diff(bag.t)))
        dv_m, psi_m = predict_derivatives(params, bag.v, bag.cmd_throttle, bag.cmd_steer, dt)
        feats = build_features(bag.v, bag.psi_dot, bag.cmd_throttle, bag.cmd_steer,
                               bag.cmd_throttle_hist, bag.cmd_steer_hist)
        feats_n = norm.apply(feats)
        with torch.no_grad():
            res = net(torch.from_numpy(feats_n.astype(np.float32))).numpy()
        dv_res_post.append((bag.dv_dt - dv_m - res[:, 0])[mask])
        psi_res_post.append((bag.psi_dot - psi_m - res[:, 1])[mask])
        feats_all.append(feats_n[mask])
        ct_all.append(bag.cmd_throttle[mask])
        cs_all.append(bag.cmd_steer[mask])
        v_all.append(bag.v[mask])
        is_straight_all.append(np.abs(bag.cmd_steer[mask]) < 2.0)
        d_throttle = np.abs(np.diff(bag.cmd_throttle[mask], prepend=bag.cmd_throttle[mask][0]))
        is_steady_all.append(d_throttle < 2.0)

    feats_concat = np.concatenate(feats_all).astype(np.float32)
    clamp = fit_clamp(feats_concat)
    clamp.save(out_clamp_path)
    print(f"wrote {out_clamp_path}: d_warn={clamp.d_warn:.2f}, d_max={clamp.d_max:.2f}")

    dv_res = np.concatenate(dv_res_post)
    psi_res = np.concatenate(psi_res_post)
    nm = estimate_noise(
        dv_res, psi_res,
        np.concatenate(ct_all), np.concatenate(cs_all),
        np.concatenate(is_straight_all), np.concatenate(is_steady_all),
    )
    nm.save(out_noise_post_path)
    print(f"wrote {out_noise_post_path}: sigma_v={nm.sigma_v:.3f}, sigma_psidot={nm.sigma_psidot:.3f}")

    v_arr = np.concatenate(v_all)
    cs = np.concatenate(cs_all)
    v_edges = np.linspace(0.0, 15.0, 6)
    cs_edges = np.linspace(0.0, 30.0, 4)
    v_bin = np.clip(np.digitize(v_arr, v_edges) - 1, 0, v_edges.size - 2)
    cs_bin = np.clip(np.digitize(np.abs(cs), cs_edges) - 1, 0, cs_edges.size - 2)
    np.savez_compressed(out_residual_emp_post,
                        dv_res=dv_res, psi_res=psi_res,
                        v_bin=v_bin, cs_bin=cs_bin,
                        v_edges=v_edges, cs_edges=cs_edges)
    print(f"wrote {out_residual_emp_post}: {dv_res.size} samples")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--mlp", required=True)
    ap.add_argument("--norm", required=True)
    ap.add_argument("--out-clamp", required=True)
    ap.add_argument("--out-noise-post", required=True)
    ap.add_argument("--out-residual-emp-post", required=True)
    args = ap.parse_args()
    fit_and_write(args.train, args.params, args.mlp, args.norm,
                  args.out_clamp, args.out_noise_post, args.out_residual_emp_post)


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
