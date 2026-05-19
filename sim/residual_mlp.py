"""Residual MLP for the data-driven sim. Section 6 of the spec."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
import torch.nn as nn

FEATURE_DIM = 11  # v + psi_dot + cmd_throttle + cmd_steer + hist[3] + hist[4]


def build_features(
    v: np.ndarray,
    psi_dot: np.ndarray,
    cmd_throttle: np.ndarray,
    cmd_steer: np.ndarray,
    cmd_throttle_hist: np.ndarray,
    cmd_steer_hist: np.ndarray,
) -> np.ndarray:
    """Stack per-tick features in the layout the MLP expects."""
    return np.concatenate(
        [
            v[:, None], psi_dot[:, None],
            cmd_throttle[:, None], cmd_steer[:, None],
            cmd_throttle_hist, cmd_steer_hist,
        ],
        axis=1,
    ).astype(np.float32)


@dataclass
class Normaliser:
    mean: np.ndarray
    std: np.ndarray

    def apply(self, x: np.ndarray) -> np.ndarray:
        return (x - self.mean) / np.maximum(self.std, 1e-6)

    def to_json(self) -> dict:
        return {"mean": self.mean.tolist(), "std": self.std.tolist()}

    @classmethod
    def from_json(cls, d: dict) -> "Normaliser":
        return cls(mean=np.asarray(d["mean"], dtype=np.float32),
                   std=np.asarray(d["std"], dtype=np.float32))


class ResidualMLP(nn.Module):
    def __init__(self, hidden: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(FEATURE_DIM, hidden), nn.Tanh(),
            nn.Linear(hidden, hidden), nn.Tanh(),
            nn.Linear(hidden, 2),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def _precompute_steer_actual(params, cmd_steer: np.ndarray, dt: float) -> np.ndarray:
    """Run steer dynamics forward and return the accumulated delta (rad) at each tick.

    This matches the internal `actual` variable in predict_derivatives, allowing
    rollout to start with the correct steer state at any anchor index without
    re-running the full prefix each time.
    """
    p = params
    slop = np.deg2rad(max(0.0, p.steer_slop_deg))
    rate_lim = np.deg2rad(p.steer_rate_max_degps) * dt
    n = cmd_steer.size
    actual_arr = np.empty(n, dtype=np.float64)
    actual = 0.0
    for i in range(n):
        c = np.deg2rad(p.steer_gain * float(cmd_steer[i]))
        if c > actual + slop:
            target = c - slop
        elif c < actual - slop:
            target = c + slop
        else:
            target = actual
        actual += float(np.clip(target - actual, -rate_lim, rate_lim))
        actual_arr[i] = actual
    return actual_arr


def _bicycle_step_stateful(
    params, v: float, cmd_throttle: float, cmd_steer: float,
    dt: float, steer_actual: float
) -> tuple:
    """One-step bicycle prediction given pre-accumulated steer_actual (rad).
    Returns (dv_dt, psi_dot, new_steer_actual)."""
    p = params
    slop = np.deg2rad(max(0.0, p.steer_slop_deg))
    rate_lim = np.deg2rad(p.steer_rate_max_degps) * dt

    target_v = np.clip(cmd_throttle / 100.0, 0.0, 1.0) * p.v_max_mps
    tau = p.accel_tau if target_v >= v else p.brake_tau
    dv_dt = (target_v - v) / max(tau, 1e-3)

    c = np.deg2rad(p.steer_gain * cmd_steer)
    if c > steer_actual + slop:
        t = c - slop
    elif c < steer_actual - slop:
        t = c + slop
    else:
        t = steer_actual
    new_actual = steer_actual + float(np.clip(t - steer_actual, -rate_lim, rate_lim))

    psi_dot_kin = v * np.tan(new_actual) / max(p.wheelbase_m, 1e-3)
    psi_dot = psi_dot_kin - p.slip_angle_at_v * v * np.sign(new_actual) * new_actual
    return dv_dt, psi_dot, new_actual


def _rollout_loss(
    net: ResidualMLP, norm: Normaliser,
    bag_dict: dict, anchors: np.ndarray, horizon: int, dt: float,
    sigma_v: float, sigma_psidot: float,
    bicycle_predict,
    device: str = "cpu",
) -> torch.Tensor:
    """For each anchor index, roll the sim forward `horizon` ticks feeding
    the bag's actual command sequence and penalise (v, psi_dot) drift from
    the bag at every intermediate step. Gradients flow only through the MLP
    output at each step (state integration is detached).

    `bicycle_predict` is a (params, steer_actual_arr) tuple where steer_actual_arr
    is the precomputed per-tick steer delta (rad) from _precompute_steer_actual.
    """
    params, steer_actual_arr = bicycle_predict
    mean_t = torch.from_numpy(norm.mean).to(device)
    std_t = torch.from_numpy(np.maximum(norm.std, 1e-6)).to(device)

    losses = []
    for a in anchors:
        if a + horizon >= bag_dict["v"].size:
            continue
        v = float(bag_dict["v"][a])
        psi_dot = float(bag_dict["psi_dot"][a])
        # Steer state at anchor: use precomputed value from tick before anchor
        steer_actual = float(steer_actual_arr[a - 1]) if a > 0 else 0.0
        rt_loss = torch.zeros((), device=device)
        for h in range(horizon):
            i = a + h
            dv_struct, psi_struct, steer_actual = _bicycle_step_stateful(
                params, v,
                float(bag_dict["cmd_throttle"][i]),
                float(bag_dict["cmd_steer"][i]),
                dt, steer_actual,
            )
            feat = build_features(
                np.array([v]), np.array([psi_dot]),
                bag_dict["cmd_throttle"][i:i + 1],
                bag_dict["cmd_steer"][i:i + 1],
                bag_dict["cmd_throttle_hist"][i:i + 1],
                bag_dict["cmd_steer_hist"][i:i + 1],
            )
            x = (torch.from_numpy(feat).to(device) - mean_t) / std_t
            res = net(x).squeeze(0)
            # dv_pred and psi_pred are tensors carrying gradients through res
            dv_pred = dv_struct + res[0]
            psi_pred = psi_struct + res[1]
            target_v = float(bag_dict["v"][i + 1])
            target_psi = float(bag_dict["psi_dot"][i + 1])
            # Loss terms: v_next = v + dv_pred*dt; psi_next = psi_pred
            # Gradients flow through dv_pred and psi_pred to the MLP
            v_next = v + dv_pred * dt
            rt_loss = rt_loss + ((v_next - target_v) ** 2) / max(sigma_v, 1e-3) ** 2
            rt_loss = rt_loss + ((psi_pred - target_psi) ** 2) / max(sigma_psidot, 1e-3) ** 2
            # State update: detach so next step's features are based on predicted scalars
            v = float(v_next.detach())
            psi_dot = float(psi_pred.detach())
        losses.append(rt_loss / horizon)
    if not losses:
        return torch.zeros((), device=device)
    return torch.stack(losses).mean()


def _bicycle_predict_fn(params):
    from sim.identify_bicycle import predict_derivatives
    def fn(v, ct, cs, dt):
        return predict_derivatives(params, v, ct, cs, dt)
    return fn


def _stack_training_data(train_npz_paths, params, autonomous_only=True, speed_floor=0.5):
    from sim.load_bags import AlignedBag
    feats_all, tgt_all = [], []
    bag_dicts = []
    dt_last = None
    for path in train_npz_paths:
        bag = AlignedBag.from_npz(path)
        mask = (bag.autonomous if autonomous_only else np.ones_like(bag.autonomous)) & (bag.v > speed_floor)
        if mask.sum() < 100:
            continue
        dt = float(np.median(np.diff(bag.t)))
        dt_last = dt
        dv_m, psi_m = _bicycle_predict_fn(params)(bag.v, bag.cmd_throttle, bag.cmd_steer, dt)
        feats = build_features(bag.v, bag.psi_dot, bag.cmd_throttle, bag.cmd_steer,
                               bag.cmd_throttle_hist, bag.cmd_steer_hist)
        target = np.stack([bag.dv_dt - dv_m, bag.psi_dot - psi_m], axis=1).astype(np.float32)
        feats_all.append(feats[mask])
        tgt_all.append(target[mask])
        bag_dicts.append({k: getattr(bag, k) for k in bag.field_names()})
    return np.concatenate(feats_all), np.concatenate(tgt_all), bag_dicts, dt_last


def _valid_anchor_indices(bag_dict: dict, horizon: int, speed_floor: float = 0.5) -> np.ndarray:
    """Return indices safe to use as rollout anchors: v > speed_floor with horizon room."""
    v = bag_dict["v"]
    n = v.size
    idx = np.where((v > speed_floor) & (np.arange(n) + horizon < n))[0]
    return idx


def main():
    import argparse
    import json
    import os
    from sim.identify_bicycle import BicycleParams
    from sim.noise_model import NoiseModel

    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--val", nargs="+", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--noise", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--rollout-weight", type=float, default=0.3)
    ap.add_argument("--rollout-horizon", type=int, default=30)
    ap.add_argument("--rollout-anchors-per-epoch", type=int, default=200)
    args = ap.parse_args()

    with open(args.params) as f:
        params = BicycleParams(**json.load(f)["params"])
    noise = NoiseModel.load(args.noise)

    feats_tr, tgt_tr, bag_dicts_tr, dt = _stack_training_data(args.train, params)
    feats_va, tgt_va, bag_dicts_va, _ = _stack_training_data(args.val, params)
    print(f"train ticks={feats_tr.shape[0]}  val ticks={feats_va.shape[0]}")

    norm = Normaliser(mean=feats_tr.mean(0).astype(np.float32),
                      std=feats_tr.std(0).astype(np.float32))
    x_tr = torch.from_numpy(norm.apply(feats_tr))
    y_tr = torch.from_numpy(tgt_tr)
    x_va = torch.from_numpy(norm.apply(feats_va))
    y_va = torch.from_numpy(tgt_va)

    net = ResidualMLP()
    opt = torch.optim.Adam(net.parameters(), lr=3e-3)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    w = torch.tensor([1.0 / max(noise.sigma_v, 1e-3) ** 2,
                      1.0 / max(noise.sigma_psidot, 1e-3) ** 2], dtype=torch.float32)

    rng = np.random.default_rng(0)
    best_val = float("inf")
    history = {"train_single": [], "val_single": [], "val_rollout": []}

    # Precompute steer state at every tick for O(1) anchor warm-up
    steer_actual_tr = _precompute_steer_actual(params, bag_dicts_tr[0]["cmd_steer"], dt)
    steer_actual_va = _precompute_steer_actual(params, bag_dicts_va[0]["cmd_steer"], dt)
    bicycle_tr = (params, steer_actual_tr)
    bicycle_va = (params, steer_actual_va)

    valid_tr = _valid_anchor_indices(bag_dicts_tr[0], args.rollout_horizon)
    valid_va = _valid_anchor_indices(bag_dicts_va[0], args.rollout_horizon)
    print(f"valid rollout anchors: train={valid_tr.size}  val={valid_va.size}")

    # Fixed validation anchors — same set every epoch so val_rollout is a
    # consistent metric for early-stopping. Use 4× the per-epoch training
    # anchors for a tighter estimate.
    val_rng = np.random.default_rng(12345)
    val_anchor_count = max(args.rollout_anchors_per_epoch * 4, 400)
    fixed_val_anchors = val_rng.choice(valid_va, size=val_anchor_count, replace=True)

    for epoch in range(args.epochs):
        net.train()
        idx = torch.randperm(x_tr.shape[0])
        epoch_single = 0.0
        for s in range(0, x_tr.shape[0], 4096):
            b = idx[s:s + 4096]
            single = ((net(x_tr[b]) - y_tr[b]) ** 2 * w).mean()
            anchors = rng.choice(valid_tr, size=args.rollout_anchors_per_epoch, replace=True)
            rollout = _rollout_loss(net, norm, bag_dicts_tr[0], anchors,
                                    args.rollout_horizon, dt,
                                    noise.sigma_v, noise.sigma_psidot,
                                    bicycle_tr)
            loss = single + args.rollout_weight * rollout
            opt.zero_grad()
            loss.backward()
            opt.step()
            epoch_single += single.item() * b.numel()
        epoch_single /= x_tr.shape[0]
        sched.step()

        net.eval()
        with torch.no_grad():
            val_single = ((net(x_va) - y_va) ** 2 * w).mean().item()
            val_rollout = _rollout_loss(net, norm, bag_dicts_va[0], fixed_val_anchors,
                                        args.rollout_horizon, dt,
                                        noise.sigma_v, noise.sigma_psidot,
                                        bicycle_va).item()
        history["train_single"].append(epoch_single)
        history["val_single"].append(val_single)
        history["val_rollout"].append(val_rollout)
        print(f"epoch {epoch:3d}  train_single={epoch_single:.4f}  "
              f"val_single={val_single:.4f}  val_rollout={val_rollout:.4f}")

        if val_single < best_val:
            best_val = val_single
            os.makedirs(args.out_dir, exist_ok=True)
            torch.save(net.state_dict(), os.path.join(args.out_dir, "mlp.pt"))
            with open(os.path.join(args.out_dir, "norm.json"), "w") as f:
                json.dump(norm.to_json(), f, indent=2)
    with open(os.path.join(args.out_dir, "training_history.json"), "w") as f:
        json.dump(history, f, indent=2)
    print(f"best val_single = {best_val:.4f}")


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
