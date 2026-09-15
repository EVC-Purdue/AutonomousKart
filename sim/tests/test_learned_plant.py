import json

import numpy as np
import pytest

from sim.plant_dataset import HISTORY, Whitener


def _fake_files(tmp_path, ds=0.1, dpsi=0.0):
    """An ensemble of one member whose weights are irrelevant: the test
    monkeypatches predict, so only the plumbing is under test."""
    meta = {
        "history": HISTORY, "plant_hz": 60.0,
        "features": ["v", "psi_dot", "delta_cmd", "throttle_sp"],
        "targets": ["alpha", "a_long"],
        "feature_whitener": Whitener(np.zeros(4), np.ones(4)).to_dict(),
        "target_whitener": Whitener(np.zeros(2), np.ones(2)).to_dict(),
        "coverage": {"v": [0.0, 12.0], "psi_dot": [-2.0, 2.0],
                     "delta_cmd": [-60.0, 60.0], "throttle_sp": [0.0, 12.0]},
    }
    p = tmp_path / "plant_nn.json"
    p.write_text(json.dumps(meta))
    return str(p)


def test_step_integrates_increments_into_world_coordinates(tmp_path):
    # Catches a broken body->world rotation: a swapped sin/cos, a flipped
    # sign, or a step that skips the frame rotation and adds (ds, dn)
    # straight into (x, y).
    from sim.learned_plant import LearnedPlant
    meta = _fake_files(tmp_path)
    plant = LearnedPlant.from_meta(meta, ensemble=None)
    plant._predict = lambda window: np.zeros(2)
    # Position advances on the state's own speed: heading is +y, so one tick at
    # 6 m/s moves 0.1 m along y and nothing along x.
    plant.reset(0.0, 0.0, np.pi / 2, v=6.0)
    plant.step(6.0, 0.0, 1.0 / 60.0)
    assert plant.x == pytest.approx(0.0, abs=1e-9)
    assert plant.y == pytest.approx(0.1, abs=1e-9)


def test_heading_increment_rotates_the_kart(tmp_path):
    # Catches a broken heading accumulation: yaw not accumulated across
    # steps (overwritten instead of summed), or dpsi dropped entirely.
    from sim.learned_plant import LearnedPlant
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    # A constant angular acceleration ramps psi_dot, and heading integrates it:
    # after n ticks yaw = alpha * dt^2 * (0+1+...+(n-1)).
    alpha = 60.0
    plant._predict = lambda window: np.array([alpha, 0.0])
    plant.reset(0.0, 0.0, 0.0, v=5.0)
    n = 4
    for _ in range(n):
        plant.step(5.0, 0.0, 1.0 / 60.0)
    dt = 1.0 / 60.0
    assert plant.yaw == pytest.approx(alpha * dt * dt * (n * (n - 1) / 2), abs=1e-9)
    assert plant.psi_dot == pytest.approx(alpha * dt * n, abs=1e-9)


def test_out_of_hull_is_counted(tmp_path):
    # Catches a missing or misplaced _check_hull call -- e.g. checking the
    # hull before the new command is written into the tick, so an
    # out-of-coverage steer_deg is never seen.
    from sim.learned_plant import LearnedPlant
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    plant._predict = lambda window: np.zeros(2)
    plant.reset(0.0, 0.0, 0.0)
    # The steering feature is the wheel angle, and the wheel slews at
    # STEER_RATE_MAX_DEGPS, so an out-of-coverage angle takes time to reach.
    for _ in range(120):
        plant.step(5.0, 500.0, 1.0 / 60.0)
    assert plant.out_of_hull >= 1


def test_step_substeps_when_dt_exceeds_the_plant_period(tmp_path):
    # Catches a wrong n_sub computation -- e.g. int() truncation instead of
    # round(), or no sub-stepping at all (always exactly one _predict call).
    from sim.learned_plant import LearnedPlant
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    calls = []
    plant._predict = lambda window: (calls.append(1), np.zeros(2))[1]
    plant.reset(0.0, 0.0, 0.0)
    plant.step(5.0, 0.0, 3.0 / 60.0)
    assert len(calls) == 3


def test_step_pairs_window_history_with_the_tick_it_was_predicted_at(tmp_path):
    """Pins the invariant shared by sim/plant_rollout.py's rollout_errors and
    sim/plant_train.py's rollout_loss: at the moment _predict is called, the
    window's last row carries the command that is about to be applied, and
    once a row has been consumed by _predict it becomes a fixed piece of
    history -- it must reappear unchanged, never mutated again.

    The first assertion block (window[-1] pairing) alone does not catch the
    bug the brief's step() had: because it appends the new tick before
    calling _predict, window[-1] already carries the right command on every
    call, both under the brief's version and the corrected one. The actual
    bug is that the brief's step() then overwrites that same, already-used
    slot in place after predicting, instead of appending a fresh row --
    corrupting the just-consumed tick's v value for every future window that
    includes it. The second block (append-only history) is what catches
    that: under the brief's version, windows[t][-2] != windows[t-1][-1]
    starting at t=1.
    """
    from sim.learned_plant import LearnedPlant
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    windows = []

    def record(window):
        windows.append(window.copy())
        return np.array([0.0, 60.0])  # a_long=60: v changes by 1.0 every call

    plant._predict = record
    plant.reset(0.0, 0.0, 0.0)

    commands = [(1.0, 10.0), (2.0, 20.0), (3.0, 30.0),
                (4.0, 40.0), (5.0, 50.0), (6.0, 60.0)]
    for target_mps, steer_deg in commands:
        plant.step(target_mps, steer_deg, 1.0 / 60.0)

    assert len(windows) == len(commands)
    # window[-1, 2] is where the wheel was when the tick was predicted, which
    # is the command slewed through the actuator, not the command itself.
    from sim.plant_dataset import slew_limit
    wheel_seq = slew_limit([c[1] for c in commands], 60.0, start=0.0)
    for t, (window, (target_mps, steer_deg)) in enumerate(zip(windows, commands)):
        assert window[-1, 2] == pytest.approx(wheel_seq[t])
        assert window[-1, 3] == pytest.approx(target_mps)

    for t in range(1, len(windows)):
        assert np.array_equal(windows[t][-2], windows[t - 1][-1]), (
            f"call {t}: window[-2]={windows[t][-2]} != "
            f"call {t - 1}'s window[-1]={windows[t - 1][-1]} -- history was "
            "mutated after being consumed by _predict"
        )


def test_reset_seeds_the_whole_window_at_the_handoff_speed(tmp_path):
    """The model's entire input is the last HISTORY ticks, so a reset that
    zeroes them hands a kart doing 6.2 m/s a window that says it is stopped.
    Asserting `plant.v` alone would pass against that bug -- the old driver
    assigned `.v` after reset -- so the assertion is on the window the model
    is actually handed."""
    from sim.learned_plant import LearnedPlant
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    seen = []
    plant._predict = lambda window: (seen.append(window.copy()), np.zeros(2))[1]

    plant.reset(1.0, 2.0, 0.0, v=6.2)
    plant.step(6.0, 3.0, 1.0 / 60.0)

    assert plant.speed == pytest.approx(6.2)
    window = seen[0]
    assert window.shape == (HISTORY, 4)
    np.testing.assert_allclose(window[:, 0], 6.2)          # v on every row
    np.testing.assert_allclose(window[:, 1], 0.0)          # psi_dot
    np.testing.assert_allclose(window[:-1, 2], 0.0)        # wheel before handoff
    np.testing.assert_allclose(window[:-1, 3], 6.2)        # throttle setpoint
    # The tick under way carries the throttle about to be applied, and the
    # wheel angle the actuator currently holds -- a 3 deg request cannot have
    # arrived yet, since reset centred the wheel.
    assert window[-1, 2] == pytest.approx(0.0)
    assert window[-1, 3] == pytest.approx(6.0)


def test_reset_takes_an_explicit_history(tmp_path):
    from sim.learned_plant import LearnedPlant
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    seen = []
    plant._predict = lambda window: (seen.append(window.copy()), np.zeros(2))[1]

    rows = [(4.0 + i, 0.1 * i, 2.0 * i, 5.0 + i) for i in range(HISTORY)]
    plant.reset(0.0, 0.0, 0.0, history=rows)
    plant.step(9.0, 8.0, 1.0 / 60.0)

    np.testing.assert_allclose(seen[0][:-1], np.asarray(rows)[:-1])
    np.testing.assert_allclose(seen[0][-1, :2], np.asarray(rows)[-1, :2])

    with pytest.raises(ValueError):
        plant.reset(0.0, 0.0, 0.0, history=rows[:2])


def test_a_standing_start_announces_itself_as_extrapolation(tmp_path):
    """v = 0 sits outside the training hull -- segments are cut on motion
    above 1.5 m/s -- so the default reset, which is what a standing start
    gets, has to be counted rather than silently predicted."""
    from sim.learned_plant import LearnedPlant
    meta = _fake_files(tmp_path)
    plant = LearnedPlant.from_meta(meta, ensemble=None)
    plant.coverage = {"v": [1.5, 12.0], "psi_dot": [-2.0, 2.0],
                      "delta_cmd": [-60.0, 60.0], "throttle_sp": [0.0, 12.0]}
    plant._predict = lambda window: np.zeros(2)

    plant.reset(0.0, 0.0, 0.0)
    plant.step(4.0, 0.0, 1.0 / 60.0)
    assert plant.out_of_hull == 1
    assert plant.queries == 1

    plant.reset(0.0, 0.0, 0.0, v=6.0)
    plant.step(6.0, 0.0, 1.0 / 60.0)
    assert plant.out_of_hull == 1      # in the hull this time
    assert plant.queries == 2


def test_check_hull_reads_the_features_in_the_declared_order(tmp_path):
    """A hardcoded name tuple that drifts from FEATURE_NAMES applies one
    channel's bounds to another. Each feature is pushed out of range on its
    own, with the other three comfortably inside, so any mis-ordering leaves
    at least one case unflagged."""
    from sim.learned_plant import LearnedPlant
    from sim.plant_dataset import FEATURE_NAMES
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)
    plant.coverage = {"v": [0.0, 10.0], "psi_dot": [-1.0, 1.0],
                      "delta_cmd": [-20.0, 20.0], "throttle_sp": [0.0, 8.0]}
    inside = [5.0, 0.0, 0.0, 4.0]
    outside = {"v": 50.0, "psi_dot": 9.0, "delta_cmd": 90.0, "throttle_sp": 40.0}

    assert plant._check_hull(inside) is False
    for i, name in enumerate(FEATURE_NAMES):
        tick = list(inside)
        tick[i] = outside[name]
        assert plant._check_hull(tick) is True, f"{name} out of range not flagged"


def test_speed_is_clamped_to_the_shared_bounds(tmp_path):
    from sim.learned_plant import LearnedPlant
    from sim.plant_rollout import V_MAX
    plant = LearnedPlant.from_meta(_fake_files(tmp_path), ensemble=None)

    plant._predict = lambda window: np.array([0.0, -300.0])
    plant.reset(0.0, 0.0, 0.0, v=2.0)
    plant.step(0.0, 0.0, 1.0 / 60.0)
    assert plant.v == pytest.approx(0.0)

    plant._predict = lambda window: np.array([0.0, 300.0])
    plant.reset(0.0, 0.0, 0.0, v=2.0)
    for _ in range(10):
        plant.step(12.0, 0.0, 1.0 / 60.0)
    assert plant.v == pytest.approx(V_MAX)


def test_from_linear_round_trips_through_the_same_runtime(tmp_path):
    """The linear plant ships through LearnedPlant, so it must load and step."""
    import json
    import numpy as np
    from sim.learned_plant import LearnedPlant
    from sim.plant_dataset import HISTORY, Whitener
    from sim.plant_models import LinearARX

    rng = np.random.default_rng(0)
    feats = rng.normal(0.0, 1.0, size=(400, HISTORY, 4))
    tg = feats.reshape(400, -1) @ rng.normal(0.0, 0.1, size=(HISTORY * 4, 2))
    fw, tw = Whitener().fit(feats), Whitener().fit(tg)
    lin = LinearARX().fit(fw.apply(feats), tw.apply(tg))
    meta = {
        "window": HISTORY, "plant_hz": 60.0,
        "features": ["v", "psi_dot", "delta_cmd", "throttle_sp"],
        "targets": ["alpha", "a_long"],
        "feature_whitener": fw.to_dict(), "target_whitener": tw.to_dict(),
        "coverage": {"v": [0.0, 12.0], "psi_dot": [-4.0, 4.0],
                     "delta_cmd": [-60.0, 60.0], "throttle_sp": [0.0, 12.0]},
        "linear": lin.to_dict(),
    }
    path = tmp_path / "plant_linear.json"
    path.write_text(json.dumps(meta))

    plant = LearnedPlant.from_linear(str(path))
    plant.reset(0.0, 0.0, 0.0, v=4.0)
    plant.step(5.0, 2.0, 1.0 / 60.0)
    # Prediction must match the model driven directly through the same whitening,
    # or the shipped plant is not the thing that was cross-validated.
    window = np.asarray(plant._hist, dtype=float)
    assert np.allclose(plant._predict(window),
                       tw.invert(lin.predict(fw.apply(window[None, ...])))[0])
    assert np.isfinite([plant.x, plant.y, plant.yaw, plant.v]).all()


def test_runtime_slews_the_wheel_like_the_dataset_does():
    """Training feeds the model a rate-limited wheel angle, so the runtime has
    to as well, or the model is driven by a signal it never saw. A step
    command must reach the model as a ramp, at the actuator's rate."""
    from sim.plant_dataset import STEER_RATE_MAX_DEGPS, Whitener
    from sim.learned_plant import LearnedPlant

    seen = []
    plant = LearnedPlant(None, {
        "plant_hz": 60.0, "coverage": {},
        "feature_whitener": Whitener(np.zeros(4), np.ones(4)).to_dict(),
        "target_whitener": Whitener(np.zeros(2), np.ones(2)).to_dict()})
    plant._predict = lambda w: (seen.append(float(w[-1, 2])), np.array([0.0, 0.0]))[1]
    plant.reset(0.0, 0.0, 0.0, v=5.0)
    for _ in range(6):
        plant.step(5.0, 40.0, 1.0 / 60.0)

    step = STEER_RATE_MAX_DEGPS / 60.0
    assert seen[0] == pytest.approx(0.0, abs=1e-9)
    assert np.max(np.diff(seen)) <= step + 1e-9
    assert seen[-1] < 40.0          # a step command has not arrived yet
    assert seen[-1] == pytest.approx(5 * step, rel=1e-6)
