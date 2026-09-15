"""Train the learned plant on multi-step rollout error.

One-step error is the metric that let the previous plant pass review while
being 1.64 m wrong at two seconds, so the loss here rolls the model forward
against the recorded commands and scores where it ends up. The horizon grows
on a curriculum because a randomly initialised model diverges at 60 steps.
"""
from __future__ import annotations

import json
import math
from typing import List

import numpy as np

from sim.plant_dataset import (FEATURE_NAMES, HISTORY, Segment, TARGET_NAMES,
                               Whitener, mirror, mirror_rollout,
                               targets_from_reference, windows_from_segment)
from sim.plant_models import PlantEnsemble, PlantMLP, as_stepper
from sim.plant_reference import FusedReference, PLANT_HZ
from sim.plant_rollout import V_MAX, V_MIN, rollout_errors

DT = 1.0 / PLANT_HZ

# Floors on the three `_pose_norm` terms. A term near zero would otherwise
# amplify the corresponding rollout_loss error by orders of magnitude instead
# of scaling it to a comparable size -- e.g. a training split that is mostly
# steady-state cruise can leave the accumulated speed-change norm near zero
# even though the whitener's one-step `dv` spread stays healthy, so the two
# can't be relied on to floor each other. Each floor is a physically small
# value chosen to sit orders of magnitude below realistic accumulated error
# (at 6 m/s over a one-second horizon: pos ~36, head ~0.09, speed ~1), so it
# only engages on genuinely degenerate input.
POS_NORM_FLOOR = 0.05 ** 2                  # 5 cm, squared
HEAD_NORM_FLOOR = math.radians(0.5) ** 2    # 0.5 degree, squared
SPEED_NORM_FLOOR = 0.05 ** 2                # 5 cm/s, squared


def pooled_windows(segments: List[Segment], splits=("train",)):
    feats, tg = [], []
    for seg in segments:
        if seg.split not in splits:
            continue
        f, t = windows_from_segment(seg)
        feats.append(f)
        tg.append(t)
    return np.concatenate(feats), np.concatenate(tg)


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _segment_targets(seg: Segment) -> np.ndarray:
    """Per-tick (alpha, a_long) for a whole segment, reusing
    targets_from_reference rather than reimplementing the derivative."""
    ref = FusedReference(t=seg.t, x=seg.x, y=seg.y, psi=seg.psi, v=seg.v,
                         omega=seg.omega, accel=np.zeros_like(seg.v))
    return targets_from_reference(ref)


def _pose_norm(segments: List[Segment], horizon: int, splits=("train",)) -> dict:
    """Typical squared size of the reference position, heading and speed
    change accumulated over `horizon` steps, in the start tick's frame.

    rollout_loss sums a position term (metres^2), a heading term (radians^2)
    and a speed term ((m/s)^2) that would otherwise sit at wildly different
    scales -- position would drown heading. Dividing each by the typical
    squared size it takes over a full-length rollout puts all three on a
    comparable footing, the same role `Whitener` plays for the one-step
    features and targets.

    This is a second moment (mean of squares about zero), not a variance
    (mean of squares about the mean): a kart holding a constant speed and
    turn rate covers nearly the same displacement from every start tick, so
    its variance across start ticks is ~0 even though the displacement
    itself is not small, and normalising by that near-zero variance would
    blow the position term up rather than scale it down.

    Body-frame integration of a segment's own (ds, dn, dpsi) targets
    reconstructs the plain relative displacement between two ticks, so this
    is computed directly off the segment's (x, y, psi, v) rather than by
    re-integrating targets step by step: rotating (x[k+h]-x[k], y[k+h]-y[k])
    by -psi[k] equals the local-frame position after h steps of integrating
    that segment's own (ds, dn) targets starting from tick k.
    """
    pos_x, pos_y, head, speed = [], [], [], []
    for seg in segments:
        if seg.split not in splits:
            continue
        n = len(seg.t)
        if n <= horizon:
            continue
        psi0 = seg.psi[:n - horizon]
        dx = seg.x[horizon:] - seg.x[:n - horizon]
        dy = seg.y[horizon:] - seg.y[:n - horizon]
        pos_x.append(np.cos(psi0) * dx + np.sin(psi0) * dy)
        pos_y.append(-np.sin(psi0) * dx + np.cos(psi0) * dy)
        head.append(_wrap(seg.psi[horizon:] - seg.psi[:n - horizon]))
        speed.append(seg.v[horizon:] - seg.v[:n - horizon])
    pos_x = np.concatenate(pos_x)
    pos_y = np.concatenate(pos_y)
    head = np.concatenate(head)
    speed = np.concatenate(speed)
    # A channel with (near) no spread -- e.g. constant-speed fixtures, where
    # every dv is exactly 0 -- would otherwise divide the corresponding loss
    # term by ~0 and blow up training; see the module-level *_NORM_FLOOR
    # comment for why a physically-sized floor is used instead of an
    # arbitrarily small one.
    return {
        "pos": max(float(np.mean(pos_x ** 2) + np.mean(pos_y ** 2)), POS_NORM_FLOOR),
        "head": max(float(np.mean(head ** 2)), HEAD_NORM_FLOOR),
        "speed": max(float(np.mean(speed ** 2)), SPEED_NORM_FLOOR),
    }


def rollout_loss(member, window0, cmd_seq, target_seq, horizon, fw: Whitener, tw: Whitener, norm: dict, torch):
    """Pose error accumulated over `horizon` steps, in the start-tick frame.

    window0:    (B, HISTORY, 4) raw features, last row is the start tick
    cmd_seq:    (B, horizon + 1, 2) recorded (delta_cmd, throttle_sp); entry k is
                the command at the tick reached after k steps, so entry 0 matches
                window0[:, -1, 2:4]
    target_seq: (B, horizon, 4) recorded (ds, dn, dpsi, dv) per step
    norm:       dict with keys "pos", "head", "speed", the per-term scale divisors

    Two poses are integrated from the origin of the start tick's frame: one
    driven by the model's own predictions, one by the recording. Comparing
    accumulated poses instead of one-step (v, psi_dot) gives ds and dn a
    gradient -- the superseded (v, omega)-only loss left them untrained, and
    ds dominates the position error the acceptance metric measures.

    The window fed back into `member` follows the same invariant
    `sim/plant_rollout.py` uses: after a step, the new last row is the tick
    just advanced to, so it carries cmd_seq[:, step + 1], not cmd_seq[:, step].
    The predicted speed is clamped with `plant_rollout.clamp_speed`'s bounds,
    the same ones evaluation and the runtime apply.
    """
    window = window0.clone().to(torch.float32)
    cmd_seq = cmd_seq.to(torch.float32)
    target_seq = target_seq.to(torch.float32)
    B = window.shape[0]
    fmean = torch.as_tensor(fw.mean, dtype=torch.float32)
    fstd = torch.as_tensor(fw.std, dtype=torch.float32)
    tmean = torch.as_tensor(tw.mean, dtype=torch.float32)
    tstd = torch.as_tensor(tw.std, dtype=torch.float32)

    v0 = window[:, -1, 0]
    w0 = window[:, -1, 1]
    x_pred = torch.zeros(B); y_pred = torch.zeros(B); psi_pred = torch.zeros(B)
    v_pred = v0.clone(); w_pred = w0.clone()
    x_ref = torch.zeros(B); y_ref = torch.zeros(B); psi_ref = torch.zeros(B)
    v_ref = v0.clone(); w_ref = w0.clone()

    loss = 0.0
    for step in range(horizon):
        z = (window - fmean) / fstd
        pred = member(z) * tstd + tmean
        alpha_p, a_p = pred[:, 0], pred[:, 1]

        # Same definitional integration as plant_rollout and learned_plant:
        # position and heading advance on the OLD v and psi_dot, then the
        # accelerations update them.
        x_pred = x_pred + torch.cos(psi_pred) * v_pred * DT
        y_pred = y_pred + torch.sin(psi_pred) * v_pred * DT
        psi_pred = psi_pred + w_pred * DT
        v_pred = torch.clamp(v_pred + a_p * DT, V_MIN, V_MAX)
        w_pred = w_pred + alpha_p * DT

        truth = target_seq[:, step, :]
        alpha_t, a_t = truth[:, 0], truth[:, 1]
        x_ref = x_ref + torch.cos(psi_ref) * v_ref * DT
        y_ref = y_ref + torch.sin(psi_ref) * v_ref * DT
        psi_ref = psi_ref + w_ref * DT
        v_ref = torch.clamp(v_ref + a_t * DT, V_MIN, V_MAX)
        w_ref = w_ref + alpha_t * DT

        pos_err = (x_pred - x_ref) ** 2 + (y_pred - y_ref) ** 2
        head_err = (psi_pred - psi_ref) ** 2
        speed_err = (v_pred - v_ref) ** 2
        loss = loss + (pos_err / norm["pos"] + head_err / norm["head"]
                       + speed_err / norm["speed"]).mean()

        cmd_next = cmd_seq[:, step + 1, :]
        new_row = torch.stack([v_pred, w_pred, cmd_next[:, 0], cmd_next[:, 1]], dim=1)
        window = torch.cat([window[:, 1:, :], new_row[:, None, :]], dim=1)

    return loss / horizon


# How hard a straightening window is oversampled at most, and the bands that
# define one. A kart turning with its wheel near centre is straightening, and
# that regime is what the sim needs after a departure.
STRAIGHTEN_MAX_WEIGHT = 8.0
STRAIGHTEN_WHEEL_DEG = 8.0
STRAIGHTEN_OMEGA = 0.3


def straightening_weight(window0: np.ndarray) -> np.ndarray:
    """Sampling weight per start window, >= 1, largest where the kart is
    turning with its wheel near centre.

    Wheel angle and yaw rate are strongly correlated in normal driving, so a
    model fitted on the bulk explains yaw with the wheel and learns almost no
    restoring term: the trained plant damped at -0.93 alpha per rad/s where the
    data says -5.83. Those windows are rare, so they are drawn more often
    rather than reweighted in the loss, which keeps the loss comparable across
    stages.
    """
    w = np.asarray(window0, dtype=float)
    omega = np.abs(w[:, -1, 1])
    wheel = np.abs(w[:, -1, 2])
    turning = np.clip(omega / STRAIGHTEN_OMEGA, 0.0, 1.0)
    centred = np.clip(1.0 - wheel / STRAIGHTEN_WHEEL_DEG, 0.0, 1.0)
    return 1.0 + (STRAIGHTEN_MAX_WEIGHT - 1.0) * turning * centred


def _build_rollout_arrays(segments: List[Segment], horizon: int, splits=("train",)):
    """(window0, cmd_seq, target_seq) for every valid start tick, mirrored.

    Built once per curriculum stage -- `train_member` reshuffles indices out
    of this per epoch rather than rebuilding it `epochs_per_stage` times.

    The returned set is twice the number of start ticks: the second half is
    the left-right reflection of the first, in the same order. Spec section 12
    names mirror augmentation as the primary mitigation for 8 min of data
    against 1988 parameters, and it has to reach these tensors, not only the
    whitener fit, or the network trains on half the data the linear baseline
    gets.
    """
    window0s, cmd_seqs, target_seqs = [], [], []
    for seg in segments:
        if seg.split not in splits:
            continue
        per_tick = np.column_stack([seg.v, seg.omega, seg.delta_cmd,
                                    seg.throttle_sp]).astype(np.float32)
        tg_full = _segment_targets(seg).astype(np.float32)
        n = len(seg.t)
        m = n - HISTORY - horizon + 1
        for k in range(max(0, m)):
            start = HISTORY - 1 + k
            window0s.append(per_tick[k:k + HISTORY])
            cmd_seqs.append(per_tick[start:start + horizon + 1, 2:4])
            target_seqs.append(tg_full[start:start + horizon])
    if not window0s:
        return (np.empty((0, HISTORY, 4), dtype=np.float32),
                np.empty((0, horizon + 1, 2), dtype=np.float32),
                np.empty((0, horizon, len(TARGET_NAMES)), dtype=np.float32))
    arrays = np.stack(window0s), np.stack(cmd_seqs), np.stack(target_seqs)
    flipped = mirror_rollout(*arrays)
    return tuple(np.concatenate([a, m]).astype(np.float32)
                 for a, m in zip(arrays, flipped))


def _bootstrap(arrays, rng):
    """Resample start ticks with replacement, one draw shared by all three arrays.

    Spec section 8 asks the ensemble for bootstrap resamples as well as seeds.
    Members that differ only by initialisation and batch order see identical
    data, and the spread the robustness work reads off them then understates
    the uncertainty. The one index draw is applied to every array so a window
    keeps the commands and targets that belong to it.
    """
    n = len(arrays[0])
    sel = rng.integers(0, n, size=n)
    return tuple(a[sel] for a in arrays)


def _epoch_batches(arrays, rng, batch=256, weights=None):
    """Shuffled batches for one epoch.

    `weights` draws with replacement in proportion to them instead, which is
    how the straightening regime gets seen often enough to be learned. The
    epoch still covers the same number of draws, so the loss stays comparable
    across stages and epochs.
    """
    window0, cmd_seq, target_seq = arrays
    n = len(window0)
    if weights is None:
        idx = rng.permutation(n)
    else:
        p = np.asarray(weights, dtype=float)
        idx = rng.choice(n, size=n, replace=True, p=p / p.sum())
    for i in range(0, len(idx), batch):
        sel = idx[i:i + batch]
        yield window0[sel], cmd_seq[sel], target_seq[sel]


def train_member(train_segments, val_segments, seed=0,
                 curriculum=(1, 5, 20, 60), epochs_per_stage=40, hidden=32,
                 val_every=5, bootstrap=False):
    """One ensemble member, early-stopped on held-out 60-step rollout error.

    Validation runs every `val_every` epochs and at the end of every stage,
    not once per stage: 1988 parameters on 8 min of correlated data overfit
    inside a stage, and scoring only at stage boundaries leaves the last
    `epochs_per_stage` epochs at the longest horizon -- where overfitting
    lands -- unwatched. The first score is taken before any training, so
    `history["val_rollout"][0]` is the untrained baseline every later score
    is read against.
    """
    import torch
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)

    feats, tg = pooled_windows(train_segments, splits=("train",))
    mf, mt = mirror(feats, tg)
    feats_all = np.concatenate([feats, mf])
    tg_all = np.concatenate([tg, mt])
    fw = Whitener().fit(feats_all)
    tw = Whitener().fit(tg_all)
    norm = _pose_norm(train_segments, max(curriculum), splits=("train",))

    member = PlantMLP(hidden=hidden)
    # lr=1e-3 (the value tried first) reliably overshot once the rollout loss
    # -- correctly scaled by norm -- got small: weight decay's fixed pull then
    # dominates a shrinking gradient and pushes val rollout error back up
    # between curriculum stages. lr=3e-4 converges without that regression.
    opt = torch.optim.Adam(member.parameters(), lr=3e-4, weight_decay=1e-4)
    history = {"val_rollout": [], "train_loss": [], "horizon": [], "epoch": []}
    best = [float("inf"), None]
    stage = [0, 0, float("nan")]   # horizon, epoch, mean train loss this epoch

    def checkpoint():
        member.eval()
        out = rollout_errors(as_stepper(_TorchAdapter(member), fw, tw),
                             val_segments, horizons=(60,), splits=("val",))
        score = out[60]["pos_median"]
        history["val_rollout"].append(score)
        history["train_loss"].append(stage[2])
        history["horizon"].append(stage[0])
        history["epoch"].append(stage[1])
        if score < best[0]:
            best[0] = score
            best[1] = {k: v.clone() for k, v in member.state_dict().items()}
        member.train()

    checkpoint()
    for horizon in curriculum:
        stage[0] = horizon
        arrays = _build_rollout_arrays(train_segments, horizon, splits=("train",))
        if bootstrap:
            arrays = _bootstrap(arrays, rng)
        weights = straightening_weight(arrays[0])
        for epoch in range(epochs_per_stage):
            stage[1] = epoch + 1
            epoch_loss, n_batches = 0.0, 0
            for window0, cmd_seq, target_seq in _epoch_batches(
                    arrays, rng, weights=weights):
                if len(window0) == 0:
                    continue
                opt.zero_grad()
                loss = rollout_loss(member, torch.as_tensor(window0), torch.as_tensor(cmd_seq),
                                    torch.as_tensor(target_seq), horizon, fw, tw, norm, torch)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(member.parameters(), 1.0)
                opt.step()
                epoch_loss += float(loss.detach())
                n_batches += 1
            stage[2] = epoch_loss / n_batches if n_batches else float("nan")
            if (epoch + 1) % val_every == 0 or epoch + 1 == epochs_per_stage:
                checkpoint()
    # Early stopping: keep the checkpoint that scored best on held-out rollout,
    # never the last one, since more epochs can make things worse.
    if best[1] is not None:
        member.load_state_dict(best[1])
    member.eval()
    return member, history


class _TorchAdapter:
    """Gives a torch module the .predict(numpy) shape `as_stepper` expects."""

    def __init__(self, module):
        self.module = module

    def predict(self, features):
        import torch
        with torch.no_grad():
            return self.module(torch.as_tensor(np.asarray(features), dtype=torch.float32)).numpy()


def train_ensemble(train_segments, val_segments, n=5, bootstrap=True,
                   on_member=None, **kwargs):
    """Train `n` members. Returns (ensemble, [history per member]).

    The histories are the only window onto what training did, so they are
    returned rather than discarded; `on_member` is called with (seed, history)
    as each finishes so a long run reports progress instead of going silent.
    """
    members, histories = [], []
    for s in range(n):
        member, history = train_member(train_segments, val_segments, seed=s,
                                       bootstrap=bootstrap, **kwargs)
        members.append(member)
        histories.append(history)
        if on_member is not None:
            on_member(s, history)
    return PlantEnsemble(members), histories


def coverage_hull(feats: np.ndarray) -> dict:
    """Per-feature [min, max] over the training windows.

    Keyed off FEATURE_NAMES so this dict and `LearnedPlant._check_hull`, which
    reads it back, cannot drift into disagreeing about which column is which.
    """
    return {name: [float(feats[..., i].min()), float(feats[..., i].max())]
            for i, name in enumerate(FEATURE_NAMES)}


def main() -> None:
    import argparse
    from sim.plant_dataset import build_segments
    from sim.sensor_noise import SensorNoiseModel

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--sensors", default="sim/model/sensors.json")
    ap.add_argument("--out", default="sim/model/plant_nn.pt")
    ap.add_argument("--meta", default="sim/model/plant_nn.json")
    ap.add_argument("--members", type=int, default=5)
    # 120 because the command's partial R^2 on the state change is still
    # climbing there: 0.188 -> 0.196 for yaw and 0.092 -> 0.146 for speed
    # between 60 and 120 ticks. Stopping at 60 left a member 41% worse at a
    # 2 s rollout and 2.6x less responsive to steering.
    ap.add_argument("--curriculum", type=int, nargs="+",
                    default=[1, 5, 20, 60, 120])
    ap.add_argument("--linear-out", default="sim/model/plant_linear.json",
                    help="where to write the linear ARX plant fitted on the "
                         "same split; it ships by default, see from_linear")
    args = ap.parse_args()

    noise = SensorNoiseModel.load(args.sensors)
    segments = build_segments(args.runs, noise)
    train = [s for s in segments if s.split == "train"]
    val = [s for s in segments if s.split == "val"]
    def report(seed, history):
        v = history["val_rollout"]
        print(f"  member {seed}: untrained {v[0]:.3f} m -> best {min(v):.3f} m "
              f"at checkpoint {int(np.argmin(v))} of {len(v) - 1}", flush=True)

    print(f"train {len(train)} stints, val {len(val)} stints, "
          f"{args.members} members, curriculum {tuple(args.curriculum)}",
          flush=True)
    ens, histories = train_ensemble(train, val, n=args.members,
                                    curriculum=tuple(args.curriculum),
                                    on_member=report)
    ens.save(args.out)

    feats, tg = pooled_windows(segments, splits=("train",))
    mf, mt = mirror(feats, tg)
    # The hull, like the whitener, describes the data the members actually
    # saw, which `_build_rollout_arrays` mirrors -- so it is symmetric in
    # psi_dot and delta_cmd, and a left-hand query is inside it exactly when
    # its right-hand reflection is.
    feats_all = np.concatenate([feats, mf])
    fw = Whitener().fit(feats_all)
    tw = Whitener().fit(np.concatenate([tg, mt]))
    with open(args.meta, "w") as f:
        json.dump({
            "window": HISTORY, "plant_hz": PLANT_HZ,
            "features": list(FEATURE_NAMES), "targets": list(TARGET_NAMES),
            "feature_whitener": fw.to_dict(), "target_whitener": tw.to_dict(),
            "runs": args.runs, "members": args.members,
            "curriculum": list(args.curriculum),
            "coverage": coverage_hull(feats_all),
            "history": [  # per member: the only record of what training did
                {k: [None if v != v else round(float(v), 6) for v in h[k]]
                 for k in ("val_rollout", "train_loss", "horizon", "epoch")}
                for h in histories],
        }, f, indent=2)
    # The linear plant is fitted on exactly the data and whitening the network
    # saw, so the two artifacts are comparable by construction.
    from sim.plant_models import LinearARX
    lin = LinearARX().fit(fw.apply(feats_all), tw.apply(np.concatenate([tg, mt])))
    with open(args.linear_out, "w") as f:
        json.dump({
            "window": HISTORY, "plant_hz": PLANT_HZ,
            "features": list(FEATURE_NAMES), "targets": list(TARGET_NAMES),
            "feature_whitener": fw.to_dict(), "target_whitener": tw.to_dict(),
            "coverage": coverage_hull(feats_all), "runs": args.runs,
            "n_params": lin.n_params, "linear": lin.to_dict(),
        }, f, indent=2)
    print(f"wrote {args.out}, {args.meta} and {args.linear_out}")


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
