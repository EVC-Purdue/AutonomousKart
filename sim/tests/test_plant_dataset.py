import numpy as np
import pytest

from sim.plant_dataset import (
    FEATURE_NAMES, HISTORY, SPLIT_MODULUS, TARGET_NAMES, Segment, Whitener,
    assign_splits, mirror, targets_from_reference, windows_from_segment,
)
from sim.plant_reference import FusedReference, PLANT_HZ


def _straight_segment(n=200, v=5.0):
    t = np.arange(n) / PLANT_HZ
    return Segment(t=t, x=v * t, y=np.zeros(n), psi=np.zeros(n),
                   v=np.full(n, v), omega=np.zeros(n),
                   delta_cmd=np.zeros(n), throttle_sp=np.full(n, v),
                   split="train")


def _varying_speed_segment(n=50):
    """omega ramps quadratically so alpha -- the first target -- steps by a
    known, distinct amount at every tick, making each window's target uniquely
    identifiable instead of being the same at every index."""
    t = np.arange(n) / PLANT_HZ
    dt = 1.0 / PLANT_HZ
    # omega[k+1] - omega[k] = (k+1)*dt  =>  alpha[k] = k+1
    omega = np.concatenate([[0.0], np.cumsum(np.arange(1, n) * dt)])
    v = np.full(n, 4.0)
    x = np.concatenate([[0.0], np.cumsum(v[:-1] * dt)])
    return Segment(t=t, x=x, y=np.zeros(n), psi=np.zeros(n),
                   v=v, omega=omega,
                   delta_cmd=np.zeros(n), throttle_sp=v.copy(),
                   split="train")


def test_targets_are_the_accelerations_commands_drive():
    """ds and dpsi were dropped because they are v*dt and psi_dot*dt, exact
    identities of the model's own inputs: a model scored on them learns to
    copy and never represents what a command does. The targets are the
    derivatives instead."""
    n = 100
    t = np.arange(n) / PLANT_HZ
    # Yaw rate ramping at 2 rad/s^2, speed ramping at 3 m/s^2.
    omega = 2.0 * t
    v = 1.0 + 3.0 * t
    ref = FusedReference(t=t, x=np.zeros(n), y=np.zeros(n), psi=np.zeros(n),
                         v=v, omega=omega, accel=np.zeros(n))
    tg = targets_from_reference(ref)
    assert tg.shape == (n - 1, 2)
    assert np.allclose(tg[:, 0], 2.0, atol=1e-6)    # angular acceleration
    assert np.allclose(tg[:, 1], 3.0, atol=1e-6)    # longitudinal acceleration


def test_targets_are_not_a_copy_of_any_input_channel():
    """The defect that made the first two trained plants useless: if a target
    is an algebraic identity of an input, the model scores well by copying and
    never learns the command response. Neither target may be."""
    n = 200
    t = np.arange(n) / PLANT_HZ
    rng = np.random.default_rng(0)
    omega = np.cumsum(rng.normal(0, 0.05, n))
    v = 4.0 + np.cumsum(rng.normal(0, 0.01, n))
    ref = FusedReference(t=t, x=np.zeros(n), y=np.zeros(n), psi=np.zeros(n),
                         v=v, omega=omega, accel=np.zeros(n))
    tg = targets_from_reference(ref)
    dt = 1.0 / PLANT_HZ
    for i, chan in enumerate((omega[:-1], v[:-1])):
        r2 = 1.0 - ((tg[:, i] - chan / dt) ** 2).mean() / tg[:, i].var()
        assert r2 < 0.5, f"target {i} is nearly a scaled copy of an input"


def test_windows_have_the_right_shapes_and_alignment():
    seg = _straight_segment(n=50)
    feats, tg = windows_from_segment(seg)
    assert feats.shape == (50 - HISTORY, HISTORY, len(FEATURE_NAMES))
    assert tg.shape == (50 - HISTORY, len(TARGET_NAMES))
    # The last row of a window is the tick the target steps away from.
    assert feats[0, -1, 0] == pytest.approx(seg.v[HISTORY - 1])

    # The fixture above holds v constant across the whole segment, so a
    # window at any position looks identical to any other -- pairing window
    # k with tg[HISTORY-2+k] (one tick early) instead of tg[HISTORY-1+k]
    # (correct) would still pass every assertion above. Use a segment where
    # v -- and therefore ds -- steps by 1 every tick, so each window's
    # target is uniquely identifiable and a one-index shift changes the
    # asserted value.
    seg2 = _varying_speed_segment(n=50)
    feats2, tg2 = windows_from_segment(seg2)
    m = 50 - HISTORY
    # alpha[j] is (omega[j+1]-omega[j])/dt, so a segment whose omega steps by
    # a known amount each tick gives every window a distinct, predictable
    # target -- checked at two positions, so a one-index shift changes it.
    # window k ends at tick k+HISTORY-1, and alpha[j] == j+1 by construction.
    assert tg2[0, 0] == pytest.approx(float(HISTORY))
    assert tg2[m - 1, 0] == pytest.approx(float(HISTORY + m - 1))


def test_mirror_negates_the_lateral_channels_only():
    feats = np.zeros((1, HISTORY, 4))
    feats[0, :, 0] = 5.0      # v
    feats[0, :, 1] = 0.3      # psi_dot
    feats[0, :, 2] = 10.0     # delta_cmd
    feats[0, :, 3] = 6.0      # throttle_sp
    tg = np.array([[0.05, 0.01]])
    mf, mt = mirror(feats, tg)
    assert np.allclose(mf[0, :, 0], 5.0)
    assert np.allclose(mf[0, :, 1], -0.3)
    assert np.allclose(mf[0, :, 2], -10.0)
    assert np.allclose(mf[0, :, 3], 6.0)
    assert mt[0, 0] == pytest.approx(-0.05)   # alpha flips
    assert mt[0, 1] == pytest.approx(0.01)    # a_long does not


def test_assign_splits_never_leaves_val_or_test_empty():
    rng = np.random.default_rng(0)
    durations = rng.uniform(4.0, 150.0, size=24)
    tags = assign_splits(list(durations))
    n_val = sum(t == "val" for t in tags)
    n_test = sum(t == "test" for t in tags)
    n_train = sum(t == "train" for t in tags)
    assert n_val > 0
    assert n_test > 0
    # Two of every SPLIT_MODULUS ranks are held out; whatever that fraction
    # is, train should hold the rest of it, not a number tied to one rule.
    expected_train_frac = 1.0 - 2.0 / SPLIT_MODULUS
    assert abs(n_train / len(tags) - expected_train_frac) < 0.1


def test_whitener_round_trips():
    rng = np.random.default_rng(0)
    a = rng.normal(3.0, 2.0, size=(500, 4))
    w = Whitener().fit(a)
    z = w.apply(a)
    assert abs(z.mean()) < 1e-9
    assert abs(z.std() - 1.0) < 1e-6
    assert np.allclose(w.invert(z), a)
    assert Whitener.from_dict(w.to_dict()).apply(a).tolist() == z.tolist()


def _fake_streams(dur=8.0, v=6.0, yaw=None, yaw_var=None, cmd_start=0.0,
                  n_cmd=None):
    """One straight-line run at `v` m/s, in the shape `read_streams` returns.

    Fast enough to drive `build_segments` end to end in a unit test: 10 Hz
    RTK-quality GPS, 100 Hz gyro and wheel, 20 Hz commands.
    """
    gt = np.arange(0.0, dur, 0.1)
    n = len(gt)
    ht = np.arange(0.0, dur, 0.01)
    ct = np.arange(cmd_start, dur, 0.05)
    if n_cmd is not None:
        ct = ct[:n_cmd]
    return {
        "gps_t": gt, "gps_x": v * gt, "gps_y": np.zeros(n),
        "gps_var_x": np.full(n, 4e-4), "gps_var_y": np.full(n, 4e-4),
        "gps_yaw": np.zeros(n) if yaw is None else yaw,
        "gps_yaw_var": np.full(n, 1e-4) if yaw_var is None else yaw_var,
        "gps_speed": np.full(n, v), "gps_speed_var": np.full(n, 1e-4),
        "imu_t": ht, "imu_gyro": np.zeros((len(ht), 3)),
        "imu_accel": np.zeros((len(ht), 3)),
        "wheel_t": ht, "wheel_v": np.full(len(ht), v),
        "cmd_t": ct, "cmd_steer": np.full(len(ct), 2.0),
        "cmd_throttle": np.full(len(ct), v),
    }


def _build(monkeypatch, streams):
    import sim.sensor_noise as sn
    from sim.plant_dataset import build_segments
    monkeypatch.setattr(sn, "read_streams", lambda run_dir: streams)
    return build_segments(["run"], sn.SensorNoiseModel.load("sim/model/sensors.json"))


def test_build_segments_ignores_gps_headings_the_receiver_disowned(monkeypatch):
    """`gps_node` sets the heading variance to 1e6 when the VTG gates fail,
    and `sensor_noise.collect` masks those out when fitting the same channel.
    Folding one into the training truth at the fitted variance drags the
    reference heading toward a course the receiver disowned. Half the fixes
    here carry a 1.5 rad heading at that variance while the kart drives dead
    straight: gated, the fused heading stays on the truth; ungated it is
    pulled to ~0.1 rad, 6 degrees of invented yaw."""
    n = len(np.arange(0.0, 8.0, 0.1))
    yaw = np.zeros(n)
    yaw[::2] = 1.5
    yaw_var = np.full(n, 1e-4)
    yaw_var[::2] = 1e6

    segments = _build(monkeypatch, _fake_streams(yaw=yaw, yaw_var=yaw_var))
    assert len(segments) == 1
    assert float(np.max(np.abs(segments[0].psi))) < 5e-3


def test_build_segments_starts_at_the_first_recorded_command(monkeypatch):
    """A tick before the first command has no command to hold. Back-filling
    the first one puts a command the kart had not been given into the window;
    zero-filling invents a hard brake at speed. The tick is dropped."""
    streams = _fake_streams(dur=10.0, cmd_start=2.0)
    segments = _build(monkeypatch, streams)
    assert len(segments) == 1
    assert segments[0].t[0] >= streams["cmd_t"][0]
    assert segments[0].t[0] < streams["cmd_t"][0] + 0.05
    assert np.all(segments[0].delta_cmd == 2.0)


def test_build_segments_drops_a_run_with_no_commands(monkeypatch):
    """A manual-driving bag carries no /cmd_drive. `_zoh` used to index an
    empty array and take `build_segments` down with it; there is also nothing
    to learn from ticks with no command, so the run is skipped."""
    assert _build(monkeypatch, _fake_streams(n_cmd=0)) == []


def test_zoh_holds_zero_before_the_first_sample_and_on_an_empty_stream():
    from sim.plant_dataset import _zoh
    grid = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
    out = _zoh(np.array([1.0, 1.5]), np.array([7.0, 9.0]), grid)
    np.testing.assert_allclose(out, [0.0, 0.0, 7.0, 9.0, 9.0])
    np.testing.assert_allclose(_zoh(np.array([]), np.array([]), grid), np.zeros(5))


def test_plausibility_is_judged_against_the_fixes_fuse_actually_used():
    """A corrupt fix that `fuse` drops must not then fail the reference for
    being 889 km from it: the gate would reject a perfect reference on the
    strength of the one sample the filter was right to ignore."""
    from sim.plant_dataset import reference_is_plausible
    from sim.plant_reference import FusedReference

    n = 60
    t = np.arange(n) / 10.0
    ref = FusedReference(t=t, x=6.0 * t, y=np.zeros(n), psi=np.zeros(n),
                         v=np.full(n, 6.0), omega=np.zeros(n),
                         accel=np.zeros(n))
    gx, gy = 6.0 * t, np.zeros(n)
    rtk_ok = np.ones(n, dtype=bool)
    assert reference_is_plausible(ref, t, gx, gy, rtk_ok)

    gy = gy.copy()
    gy[n // 2] = 889537.265
    assert reference_is_plausible(ref, t, gx, gy, rtk_ok)


def test_slew_limit_turns_a_command_into_a_wheel_angle():
    """The plant's steering feature is where the wheel is, not what was asked.

    MPC demanded 91-127 deg/s (p90) on the 2026-09-13 bags while the heavy
    kart's wheel slews at 33.7 deg/s, so the command and the wheel are
    different signals for seconds at a time. The model reads HISTORY = 5 ticks
    = 83 ms of history, far too little to infer a wheel that needs ~2 s to
    traverse a full swing, so the actuator is integrated for it.
    """
    from sim.plant_dataset import STEER_RATE_MAX_DEGPS, slew_limit

    hz = 60.0
    n = 120
    cmd = np.full(n, 40.0)
    wheel = slew_limit(cmd, hz, start=0.0)
    step = STEER_RATE_MAX_DEGPS / hz
    assert wheel[0] == pytest.approx(0.0)
    assert wheel[1] == pytest.approx(step)
    # never moves faster than the actuator can
    assert np.max(np.abs(np.diff(wheel))) <= step + 1e-9
    # and does arrive, given enough time
    assert wheel[-1] == pytest.approx(40.0, abs=1e-6)

    # a command that reverses faster than the wheel can follow is tracked, not jumped
    flip = np.concatenate([np.full(30, 30.0), np.full(30, -30.0)])
    w = slew_limit(flip, hz, start=30.0)
    assert np.max(np.abs(np.diff(w))) <= step + 1e-9
    assert w[30] > 0.0          # still on the old side one tick after the flip

    # mirror symmetry, which the training augmentation relies on
    np.testing.assert_allclose(slew_limit(-flip, hz, start=-30.0), -w, atol=1e-9)
