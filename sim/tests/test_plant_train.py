import math

import numpy as np
import pytest

from sim.plant_dataset import HISTORY, Segment, Whitener, targets_from_reference
from sim.plant_reference import PLANT_HZ, FusedReference


def _segment(n=600, v=5.0, omega=0.0, split="train"):
    t = np.arange(n) / PLANT_HZ
    psi = omega * t
    return Segment(t=t, x=np.cumsum(np.cos(psi) * v / PLANT_HZ),
                   y=np.cumsum(np.sin(psi) * v / PLANT_HZ), psi=psi,
                   v=np.full(n, v), omega=np.full(n, omega),
                   delta_cmd=np.full(n, omega * 30.0),
                   throttle_sp=np.full(n, v), split=split)


def _rollout_arrays(seg, horizon, k0=None):
    """(window0, cmd_seq, target_seq) for one or every valid start tick.

    Built independently of sim.plant_train._build_rollout_arrays so a bug in
    that helper can't hide a matching bug in rollout_loss.
    """
    per_tick = np.column_stack([seg.v, seg.omega, seg.delta_cmd,
                                seg.throttle_sp]).astype(np.float32)
    ref = FusedReference(t=seg.t, x=seg.x, y=seg.y, psi=seg.psi, v=seg.v,
                         omega=seg.omega, accel=np.zeros_like(seg.v))
    tg_full = targets_from_reference(ref).astype(np.float32)
    n = len(seg.t)
    m = n - HISTORY - horizon + 1
    ks = range(m) if k0 is None else [k0]
    window0s, cmd_seqs, target_seqs = [], [], []
    for k in ks:
        start = HISTORY - 1 + k
        window0s.append(per_tick[k:k + HISTORY])
        cmd_seqs.append(per_tick[start:start + horizon + 1, 2:4])
        target_seqs.append(tg_full[start:start + horizon])
    return np.stack(window0s), np.stack(cmd_seqs), np.stack(target_seqs)


def test_training_reduces_rollout_error_on_a_learnable_segment():
    from sim.plant_train import train_member
    train = [_segment(omega=0.0), _segment(omega=0.2), _segment(omega=-0.2)]
    val = [_segment(omega=0.1, split="val")]
    member, history = train_member(train, val, seed=0,
                                   curriculum=(1, 5), epochs_per_stage=15)
    assert history["val_rollout"][-1] < history["val_rollout"][0]


def test_rollout_loss_grows_with_horizon_for_a_wrong_member():
    torch = pytest.importorskip("torch")
    from sim.plant_train import rollout_loss

    fw = Whitener(np.zeros(4), np.ones(4))
    tw = Whitener(np.zeros(2), np.ones(2))
    norm = {"pos": 1.0, "head": 1.0, "speed": 1.0}

    class Wrong(torch.nn.Module):
        """Invents a steady angular acceleration the kart never had, so the
        predicted heading peels away from the reference and the position
        error compounds with every step."""
        def forward(self, x):
            out = torch.zeros(x.shape[0], 2)
            out[:, 0] = 2.0
            return out

    seg = _segment(n=200, v=5.0, omega=0.3)
    w0_short, cmd_short, tgt_short = _rollout_arrays(seg, 1)
    w0_long, cmd_long, tgt_long = _rollout_arrays(seg, 20)
    short = rollout_loss(Wrong(), torch.as_tensor(w0_short), torch.as_tensor(cmd_short),
                         torch.as_tensor(tgt_short), 1, fw, tw, norm, torch)
    long_ = rollout_loss(Wrong(), torch.as_tensor(w0_long), torch.as_tensor(cmd_long),
                         torch.as_tensor(tgt_long), 20, fw, tw, norm, torch)
    assert float(long_) > float(short)


def test_rollout_loss_punishes_a_member_that_ignores_the_steering_command():
    """The defect that made the first two trained plants useless.

    `ds` and `dpsi` were v*dt and psi_dot*dt, exact identities of the model's
    own inputs, so a member could score well by copying its input and never
    represent a command at all. Measured on the result: 0.00000 rad/s per
    degree of steering for the linear fit, 0.00081 for the network, against
    0.02514 for a bicycle model. With accelerations as targets, a member that
    predicts zero acceleration -- pure persistence -- must score badly
    whenever the reference is actually accelerating.
    """
    torch = pytest.importorskip("torch")
    from sim.plant_train import rollout_loss

    fw = Whitener(np.zeros(4), np.ones(4))
    tw = Whitener(np.zeros(2), np.ones(2))
    norm = {"pos": 1.0, "head": 1.0, "speed": 1.0}
    horizon = 20

    class Persistence(torch.nn.Module):
        """Zero acceleration: carries the current state forward forever."""
        def forward(self, x):
            return torch.zeros(x.shape[0], 2)

    # A segment whose yaw rate is genuinely changing, which is exactly what a
    # steering command does and exactly what persistence cannot represent.
    alpha_true = 3.0
    n = 200
    t = np.arange(n) / PLANT_HZ
    dt = 1.0 / PLANT_HZ
    omega = alpha_true * t
    psi = np.concatenate([[0.0], np.cumsum(omega[:-1] * dt)])
    v = np.full(n, 5.0)
    x = np.concatenate([[0.0], np.cumsum(v[:-1] * np.cos(psi[:-1]) * dt)])
    y = np.concatenate([[0.0], np.cumsum(v[:-1] * np.sin(psi[:-1]) * dt)])
    seg = Segment(t=t, x=x, y=y, psi=psi, v=v, omega=omega,
                  delta_cmd=np.full(n, 10.0), throttle_sp=v.copy(), split="train")
    class Truth(torch.nn.Module):
        """Predicts the angular acceleration the segment was built with."""
        def forward(self, x):
            out = torch.zeros(x.shape[0], 2)
            out[:, 0] = alpha_true
            return out

    w0, cmd, tgt = _rollout_arrays(seg, horizon)
    args = (torch.as_tensor(w0), torch.as_tensor(cmd), torch.as_tensor(tgt),
            horizon, fw, tw, norm, torch)
    truth = float(rollout_loss(Truth(), *args))
    persistence = float(rollout_loss(Persistence(), *args))
    # Scale-free, because the absolute loss depends on the normalisers: what
    # matters is that following the command beats ignoring it by a wide
    # margin. Under the old targets both members scored the same.
    assert truth < 1e-9
    assert persistence > 1e4 * truth + 1e-5


def test_rollout_window_update_uses_the_tick_just_advanced_tos_command():
    # The window fed to `member` after step k must carry cmd_seq[:, k + 1],
    # the command at the tick the rollout has just advanced to -- not
    # cmd_seq[:, k], the command that was active one step earlier. This is
    # the exact off-by-one that review found in plant_rollout.py.
    torch = pytest.importorskip("torch")
    from sim.plant_train import rollout_loss

    fw = Whitener(np.zeros(4), np.ones(4))
    tw = Whitener(np.zeros(2), np.ones(2))
    norm = {"pos": 1.0, "head": 1.0, "speed": 1.0}
    horizon = 5

    seg = _segment(n=200, v=5.0, omega=0.0)
    # Distinct, differently-scaled command each tick so a one-tick lag or a
    # channel swap changes the value a call sees.
    seg.delta_cmd = np.arange(len(seg.t), dtype=float)
    seg.throttle_sp = np.arange(len(seg.t), dtype=float) * 2.0 + 1.0
    w0, cmd, tgt = _rollout_arrays(seg, horizon, k0=0)

    seen = []

    class Recorder(torch.nn.Module):
        def forward(self, x):
            seen.append(x[:, -1, 2:4].clone())
            return torch.zeros(x.shape[0], 2)

    rollout_loss(Recorder(), torch.as_tensor(w0), torch.as_tensor(cmd),
                torch.as_tensor(tgt), horizon, fw, tw, norm, torch)

    assert len(seen) == horizon
    for step in range(horizon):
        assert torch.allclose(seen[step][0], torch.as_tensor(cmd[0, step, :]))


def test_pooled_windows_only_includes_the_requested_split():
    from sim.plant_train import pooled_windows
    train = [_segment(n=100, split="train"), _segment(n=100, split="train")]
    val = [_segment(n=100, split="val")]
    feats_train, tg_train = pooled_windows(train + val, splits=("train",))
    feats_all, tg_all = pooled_windows(train + val, splits=("train", "val"))
    per_seg = 100 - HISTORY
    assert len(feats_train) == 2 * per_seg
    assert len(tg_train) == 2 * per_seg
    assert len(feats_all) == 3 * per_seg


def _varying_segment(n, split="test"):
    """Every channel distinct per tick, so a one-tick shift anywhere shows.

    `delta_cmd` is the wheel angle the model reads, so it is built by slewing
    a raw command exactly as build_segments does, and the raw command travels
    alongside it -- the runtime is handed that and slews for itself.
    """
    from sim.plant_dataset import slew_limit
    t = np.arange(n) / PLANT_HZ
    psi = 0.1 * np.arange(n)
    v = 5.0 + 0.1 * np.arange(n)
    raw = 3.0 + np.arange(n, dtype=float)
    return Segment(t=t, x=np.cumsum(np.cos(psi) * v / PLANT_HZ),
                   y=np.cumsum(np.sin(psi) * v / PLANT_HZ), psi=psi, v=v,
                   omega=0.2 + 0.01 * np.arange(n),
                   delta_cmd=slew_limit(raw, PLANT_HZ, start=float(raw[0])),
                   throttle_sp=7.0 + 0.5 * np.arange(n), split=split,
                   cmd_raw=raw)


def test_training_evaluation_and_runtime_feed_the_model_identical_windows():
    """The one place the window invariant is pinned across all three sites.

    `rollout_loss` (training), `rollout_errors` (acceptance) and
    `LearnedPlant.step` (runtime) each advance a window independently. If any
    two disagree on which tick a row represents, the model trains on one
    alignment and is scored or driven on another, and every unit test still
    passes. Here one deterministic model is rolled through all three and the
    exact sequence of windows it is handed must match.
    """
    torch = pytest.importorskip("torch")

    from sim.learned_plant import LearnedPlant
    from sim.plant_models import as_stepper
    from sim.plant_rollout import rollout_errors
    from sim.plant_train import rollout_loss

    horizon = 6
    seg = _varying_segment(HISTORY + horizon + 2)
    k0 = HISTORY - 1
    identity_f = Whitener(np.zeros(4), np.ones(4))
    identity_t = Whitener(np.zeros(2), np.ones(2))
    weights = np.tile([1e-3, 2e-3, 3e-4, 4e-4], HISTORY)

    def increments(flat):
        s = float(flat @ weights)
        return np.array([1e-3 + 1e-5 * s, 0.05 + 1e-4 * s])

    class RecordingNumpy:
        def __init__(self):
            self.windows = []

        def predict(self, z):
            self.windows.append(np.asarray(z[0], dtype=float))
            return increments(np.asarray(z[0], dtype=float).reshape(-1))[None, :]

    class RecordingTorch(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.windows = []

        def forward(self, z):
            self.windows.append(z[0].detach().numpy().astype(float))
            w = torch.as_tensor(weights, dtype=torch.float32)
            s = (z.reshape(z.shape[0], -1) * w).sum(1)
            return torch.stack([1e-3 + 1e-5 * s, 0.05 + 1e-4 * s], dim=1)

    evaluation = RecordingNumpy()
    rollout_errors(as_stepper(evaluation, identity_f, identity_t), [seg],
                   horizons=(horizon,), splits=("test",), stride=10_000)

    window0, cmd_seq, target_seq = _rollout_arrays(seg, horizon, k0=0)
    training = RecordingTorch()
    rollout_loss(training, torch.as_tensor(window0), torch.as_tensor(cmd_seq),
                 torch.as_tensor(target_seq), horizon, identity_f, identity_t,
                 {"pos": 1.0, "head": 1.0, "speed": 1.0}, torch)

    runtime_model = RecordingNumpy()
    plant = LearnedPlant(runtime_model, {
        "plant_hz": PLANT_HZ, "coverage": {},
        "feature_whitener": identity_f.to_dict(),
        "target_whitener": identity_t.to_dict()})
    # psi_dot is plant state, so the handoff has to seed it along with the
    # window; leaving it at the reset default would desynchronise the runtime
    # from the other two after the first step.
    plant.reset(float(seg.x[k0]), float(seg.y[k0]), float(seg.psi[k0]),
                v=float(seg.v[k0]),
                history=[(seg.v[j], seg.omega[j], seg.delta_cmd[j],
                          seg.throttle_sp[j])
                         for j in range(k0 - HISTORY + 1, k0 + 1)])
    for step in range(horizon):
        plant.step(float(seg.throttle_sp[k0 + step]),
                   float(seg.cmd_raw[k0 + step]), 1.0 / PLANT_HZ)

    assert len(evaluation.windows) == horizon
    assert len(training.windows) == horizon
    assert len(runtime_model.windows) == horizon
    for step in range(horizon):
        expected_cmd = seg.delta_cmd[k0 + step]
        assert evaluation.windows[step][-1, 2] == pytest.approx(expected_cmd)
        np.testing.assert_allclose(training.windows[step],
                                   evaluation.windows[step], atol=1e-4)
        np.testing.assert_allclose(runtime_model.windows[step],
                                   evaluation.windows[step], atol=1e-9)


def _integrate_targets(targets, v, psi_dot):
    """(x, y, psi) after integrating (alpha, a_long) from the origin."""
    x = y = psi = 0.0
    dt = 1.0 / PLANT_HZ
    for alpha, a_long in targets:
        x += math.cos(psi) * v * dt
        y += math.sin(psi) * v * dt
        psi += psi_dot * dt
        v += a_long * dt
        psi_dot += alpha * dt
    return x, y, psi


def test_build_rollout_arrays_mirror_augments_the_training_tensors():
    """Mirror augmentation has to reach the tensors the network trains on.

    Fitting the whitener on a mirrored pool while training on the unmirrored
    one leaves the network on half the data `plant_report.fit_linear` gives
    the baseline it has to beat, and spec section 12's headline mitigation
    for the sample size absent from the model it protects. Asserting only
    "the set doubled" would pass against a copy-paste duplication, and
    asserting only the signs would pass against an unmirrored second half of
    zeros, so both are checked, and then the reflected sample is integrated
    to confirm its trajectory really is the mirror image of the original.
    """
    from sim.plant_train import _build_rollout_arrays

    horizon = 4
    seg = _varying_segment(HISTORY + 20, split="train")
    w0, cmd, tgt = _build_rollout_arrays([seg], horizon)
    plain_w0, plain_cmd, plain_tgt = _rollout_arrays(seg, horizon)

    n = len(plain_w0)
    assert n > 0
    assert (len(w0), len(cmd), len(tgt)) == (2 * n, 2 * n, 2 * n)
    np.testing.assert_allclose(w0[:n], plain_w0)
    np.testing.assert_allclose(cmd[:n], plain_cmd)
    np.testing.assert_allclose(tgt[:n], plain_tgt)

    np.testing.assert_allclose(w0[n:], plain_w0 * [1.0, -1.0, -1.0, 1.0])
    np.testing.assert_allclose(cmd[n:], plain_cmd * [-1.0, 1.0])
    np.testing.assert_allclose(tgt[n:], plain_tgt * [-1.0, 1.0])
    # The fixture turns, so the reflection is not the identity.
    assert not np.allclose(w0[n:], w0[:n])

    for k in (0, n - 1):
        x, y, psi = _integrate_targets(tgt[k], w0[k][-1, 0], w0[k][-1, 1])
        mx, my, mpsi = _integrate_targets(tgt[n + k], w0[n + k][-1, 0],
                                          w0[n + k][-1, 1])
        assert abs(y) > 1e-3 and abs(psi) > 1e-3
        assert mx == pytest.approx(x, abs=1e-6)
        assert my == pytest.approx(-y, abs=1e-6)
        assert mpsi == pytest.approx(-psi, abs=1e-6)


def test_validation_runs_inside_a_stage_not_only_at_its_end():
    """Scoring once per curriculum stage leaves the last `epochs_per_stage`
    epochs of the longest horizon unwatched, which is exactly where 1988
    parameters on correlated data overfit. Validation now runs every
    `val_every` epochs and at the stage's final epoch, plus once before any
    training so the history opens on an untrained baseline."""
    from sim.plant_train import train_member
    train = [_segment(n=120, omega=0.0), _segment(n=120, omega=0.2)]
    val = [_segment(n=200, omega=0.1, split="val")]
    _, history = train_member(train, val, seed=0, curriculum=(1,),
                              epochs_per_stage=5, val_every=2)
    # untrained baseline, then epochs 2, 4 and the stage's last epoch 5.
    assert len(history["val_rollout"]) == 4


def test_bootstrap_resamples_with_replacement_and_keeps_rows_aligned():
    """Spec section 8's bootstrap. Drawing a separate resample per array
    would break the pairing between a window, the commands that drive it and
    the targets it is scored against, and a permutation would leave every
    member on identical data -- both are checked, not just the length."""
    from sim.plant_train import _bootstrap
    n = 200
    tag = np.arange(n, dtype=np.float32)
    arrays = (np.tile(tag[:, None, None], (1, HISTORY, 4)),
              np.tile(tag[:, None, None], (1, 3, 2)),
              np.tile(tag[:, None, None], (1, 2, 4)))
    bw, bc, bt = _bootstrap(arrays, np.random.default_rng(0))

    assert len(bw) == len(bc) == len(bt) == n
    assert np.array_equal(bw[:, 0, 0], bc[:, 0, 0])
    assert np.array_equal(bw[:, 0, 0], bt[:, 0, 0])
    drawn = bw[:, 0, 0]
    assert set(np.unique(drawn)).issubset(set(tag.tolist()))
    assert len(np.unique(drawn)) < n


def test_train_ensemble_gives_every_member_its_own_resample(monkeypatch):
    import sim.plant_train as pt
    seen = []

    def fake_train_member(train_segments, val_segments, **kwargs):
        seen.append(kwargs)
        return object(), {"val_rollout": [1.0]}

    monkeypatch.setattr(pt, "train_member", fake_train_member)
    pt.train_ensemble([], [], n=3)
    assert len(seen) == 3
    assert all(k["bootstrap"] for k in seen)
    assert sorted(k["seed"] for k in seen) == [0, 1, 2]


def test_rollout_loss_clamps_speed_the_same_way():
    """Training has to integrate the plant the runtime runs, ceiling and
    floor included, or the loss is computed on speeds the sim can never
    reach."""
    torch = pytest.importorskip("torch")
    from sim.plant_rollout import V_MAX, V_MIN
    from sim.plant_train import rollout_loss

    fw = Whitener(np.zeros(4), np.ones(4))
    tw = Whitener(np.zeros(2), np.ones(2))
    norm = {"pos": 1.0, "head": 1.0, "speed": 1.0}
    horizon = 20
    seg = _segment(n=100, v=5.0, omega=0.0)
    w0, cmd, tgt = _rollout_arrays(seg, horizon, k0=0)

    class ConstantAccel(torch.nn.Module):
        def __init__(self, a_long):
            super().__init__()
            self.a_long = a_long
            self.speeds = []

        def forward(self, x):
            self.speeds.append(float(x[0, -1, 0]))
            out = torch.zeros(x.shape[0], 2)
            out[:, 1] = self.a_long
            return out

    # 60 m/s^2 for a 20-step (0.33 s) horizon walks the speed into both rails.
    braking = ConstantAccel(-60.0)
    rollout_loss(braking, torch.as_tensor(w0), torch.as_tensor(cmd),
                 torch.as_tensor(tgt), horizon, fw, tw, norm, torch)
    assert min(braking.speeds) == pytest.approx(V_MIN)

    runaway = ConstantAccel(60.0)
    rollout_loss(runaway, torch.as_tensor(w0), torch.as_tensor(cmd),
                 torch.as_tensor(tgt), horizon, fw, tw, norm, torch)
    assert max(runaway.speeds) == pytest.approx(V_MAX)


def test_coverage_hull_is_keyed_off_the_declared_feature_order():
    """The hull dict and `LearnedPlant._check_hull` read the same tuple, so
    one cannot drift into labelling a column with another channel's name."""
    from sim.plant_dataset import FEATURE_NAMES
    from sim.plant_train import coverage_hull
    feats = np.zeros((10, HISTORY, 4))
    for i in range(4):
        feats[..., i] = np.linspace(10.0 * i, 10.0 * i + 3.0, 10)[:, None]
    hull = coverage_hull(feats)
    assert list(hull) == list(FEATURE_NAMES)
    for i, name in enumerate(FEATURE_NAMES):
        assert hull[name] == pytest.approx([10.0 * i, 10.0 * i + 3.0])


def test_straightening_windows_are_oversampled():
    """The regime the sim needs after a departure, and the one the data hides.

    Wheel angle and yaw rate are strongly correlated in normal driving -- you
    steer into a turn -- so a model fitted on the bulk explains yaw with the
    wheel and sees little of the restoring term, which is what governs whether
    a kart straightens once the planner centres the wheel. The regime is rare:
    only 66 windows in 586 s hold the wheel inside 8 deg for a quarter second
    while the kart is turning at more than 0.3 rad/s. Over those windows the
    yaw rate decays to 0.485 of its value in 0.25 s (tau 0.35 s) and the
    trained plant decays to 0.581 (tau 0.46 s), so the model is around 30%
    under-damped. Drawing those windows more often costs nothing measurable in
    rollout error and is the only handle the data offers on that term.
    """
    from sim.plant_train import straightening_weight

    # turning hard with the wheel centred: the restoring regime
    turning = np.zeros((HISTORY, 4)); turning[:, 1] = 0.8; turning[:, 2] = 1.0
    # turning with the wheel into the corner: the common case
    committed = np.zeros((HISTORY, 4)); committed[:, 1] = 0.8; committed[:, 2] = 25.0
    # straight running
    straight = np.zeros((HISTORY, 4)); straight[:, 1] = 0.0; straight[:, 2] = 0.0

    w = straightening_weight(np.stack([turning, committed, straight]))
    assert w.shape == (3,)
    assert w[0] > 3.0 * w[1], "the restoring regime must be drawn far more often"
    assert w[0] > 3.0 * w[2]
    assert np.all(w >= 1.0), "no window is ever dropped"
    assert abs(float(np.mean(w)) - 1.0) > 0.0   # weights are not degenerate


def test_epoch_batches_draw_the_straightening_regime_more_often():
    """The weight has to reach the batches, not just exist."""
    from sim.plant_train import _epoch_batches, straightening_weight

    n = 400
    w0 = np.zeros((n, HISTORY, 4), dtype=np.float32)
    w0[:, :, 1] = 0.8                      # everything is turning
    w0[: n // 2, :, 2] = 0.5               # first half: wheel centred
    w0[n // 2:, :, 2] = 25.0               # second half: wheel committed
    cmd = np.zeros((n, 2, 2), dtype=np.float32)
    tgt = np.zeros((n, 1, 2), dtype=np.float32)
    cmd[:, 0, 0] = np.arange(n)            # a tag that survives the draw

    rng = np.random.default_rng(0)
    seen = []
    for a, b, _c in _epoch_batches((w0, cmd, tgt), rng,
                                   weights=straightening_weight(w0)):
        seen.extend(int(v) for v in b[:, 0, 0])
    seen = np.asarray(seen)
    assert len(seen) == n, "an epoch still covers n draws"
    centred = float((seen < n // 2).mean())
    assert centred > 0.7, f"straightening windows only {centred:.0%} of the epoch"

    # with no weights the split stays even, so the change is opt-in
    seen2 = []
    for a, b, _c in _epoch_batches((w0, cmd, tgt), np.random.default_rng(0)):
        seen2.extend(int(v) for v in b[:, 0, 0])
    assert 0.4 < float((np.asarray(seen2) < n // 2).mean()) < 0.6
