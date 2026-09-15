import math
import os

import numpy as np
import pytest
import yaml

REAL_RUN_DIR = "new_mpc_bags/20260913/run_20260913_211133_auto"
REAL_LINE_CSV = "data/racing_line/line6.csv"

# Minimal params_live.yaml block build_planner needs. num_samples/horizon_steps
# kept small so the discriminating test below stays fast.
_FIXTURE_PARAMS = {
    "/pathfinder_node": {
        "ros__parameters": {
            "v_max_mps": 12.0,
            "wheelbase_m": 1.05,
            "steer_max_deg": 60.0,
            "steer_rate_max_degps": 180.0,
            "a_max_mps2": 2.0,
            "a_min_mps2": -3.0,
            "a_lat_max_mps2": 4.0,
            "mpc": {
                "horizon_steps": 12,
                "dt_s": 0.05,
                "num_samples": 48,
                "target_speed_mps": 6.0,
                "track_half_width_m": 3.0,
                "residual": {"mode": "shadow", "cache_enabled": False},
            },
        },
    },
}


def _write_fixture_run(tmp_path):
    """A run_dir with just enough params_live.yaml for build_planner, plus a
    dead-straight racing line (y=0, x from -5 to 100 in 1 m steps)."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    with open(run_dir / "params_live.yaml", "w") as f:
        yaml.safe_dump(_FIXTURE_PARAMS, f)

    line_csv = tmp_path / "line_straight.csv"
    with open(line_csv, "w") as f:
        for i, x in enumerate(range(-5, 101)):
            f.write(f"{float(i)},{float(x)},0.0,0.0,0.0,5.0\n")

    return str(run_dir), str(line_csv)


class _StraightPlant:
    """Test double with the DataSim/LearnedPlant interface: ignores the
    planner's steering entirely and drives dead straight, holding whatever
    yaw `reset()` gave it, at whatever speed `reset()` was handed -- it takes
    its speed from the reset argument and nowhere else, so a driver that
    leaves `v` at the default and assigns `.v` afterwards is exposed (the
    seeded window a LearnedPlant reads would stay at zero, and this plant
    never moves)."""

    def __init__(self):
        self.x = self.y = self.yaw = self.v = 0.0

    @property
    def speed(self):
        return self.v

    def reset(self, x, y, yaw, v=0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.v = float(v)

    def step(self, target_mps, steer_deg, dt):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        return self.x, self.y, self.yaw, self.v


def test_statistics_are_computed_from_a_trace():
    from sim.plant_closed_loop import trace_statistics
    t = np.arange(0.0, 2.0, 1.0 / 60.0)
    cte = np.sin(t)
    steer = 10.0 * np.sin(2.0 * t)
    out = trace_statistics(t, cte, steer)
    assert out["mean_abs_cte"] == pytest.approx(np.abs(cte).mean(), rel=1e-9)
    assert out["max_abs_cte"] == pytest.approx(np.abs(cte).max(), rel=1e-9)
    assert out["sd_steer"] == pytest.approx(steer.std(), rel=1e-9)
    assert out["p90_steer_rate"] > 0.0


def test_real_stats_carry_both_engagements():
    from sim.plant_closed_loop import REAL_STATS
    assert set(REAL_STATS) == {1, 2}
    assert REAL_STATS[1]["mean_abs_cte"] == pytest.approx(0.43)
    assert REAL_STATS[2]["sd_steer"] == pytest.approx(20.9)


def test_stability_flags_a_diverging_plant():
    from sim.plant_closed_loop import is_stable
    t = np.arange(0.0, 5.0, 1.0 / 60.0)
    assert is_stable(np.zeros_like(t))
    assert not is_stable(np.linspace(0.0, 50.0, len(t)))


def test_stability_check_runs_the_datasim_backend_end_to_end(tmp_path):
    """Plumbing test for `stability_check`, not a stability measurement.

    `sim_backend` used to be a lie: `stability_check` took a `plant_factory`
    parameter it never touched, hardcoding `sim_backend="learned"` instead --
    a backend with no trained weights in this checkout, so the function had
    zero behavioural coverage. `sim_backend` is now the thing that actually
    selects the backend, and "datasim" exists today, so this exercises the
    real code path (rows parsed from `line_csv`, `simulate()` invoked per
    seed, `is_stable`/`aborted` folded into the per-seed bool) end to end
    with a duration and seed count small enough to stay a fast unit test.
    """
    from sim.plant_closed_loop import stability_check

    _, line_csv = _write_fixture_run(tmp_path)
    out = stability_check(line_csv, sim_backend="datasim", seconds=2.0, seeds=1)
    assert isinstance(out, list)
    assert len(out) == 1
    assert isinstance(out[0], bool)


def test_build_planner_reads_kart_constants_from_params_live_yaml(tmp_path):
    from sim.plant_closed_loop import build_planner
    run_dir, line_csv = _write_fixture_run(tmp_path)
    planner = build_planner(run_dir, line_csv)
    assert planner.v_max == pytest.approx(12.0)
    assert planner.wheelbase == pytest.approx(1.05)
    assert planner.K == 48  # num_samples from the fixture yaml, not a default


def test_handoff_predicts_cte_from_a_frozen_lateral_offset(tmp_path):
    """The discriminating case: a plant that ignores steering and drives dead
    straight, handed off with a lateral offset from a dead-straight racing
    line, must show that EXACT offset as its cross-track error for the whole
    closed-loop trace -- not 0 (warmup pose leaking into the reset), not the
    offset from an adjacent tick (an off-by-one on the handoff index), and
    not "the plant never moved" (a missing post-reset speed assignment).

    The lateral offset is planted as a single-tick spike in the synthetic
    recorded trace, exactly at the tick `_closed_loop_trace` should select as
    the handoff: every other recorded tick, before AND after, sits on the
    line at y=0. An off-by-one in either direction lands on a y=0 tick and
    the predicted 2.0 m offset silently becomes 0.
    """
    from sim.plant_closed_loop import _closed_loop_trace, build_planner, trace_statistics

    run_dir, line_csv = _write_fixture_run(tmp_path)
    planner = build_planner(run_dir, line_csv)

    dt_rec = 1.0 / 60.0
    n = 360  # 6 s of recorded ticks
    t = np.arange(n) * dt_rec
    v_rec = 4.0
    x = v_rec * t
    y = np.zeros(n)
    yaw = np.zeros(n)
    v = np.full(n, v_rec)

    start_s, warmup_s, end_s = 0.0, 0.503, t[-1]
    handoff_t = start_s + warmup_s
    i0 = int(np.searchsorted(t, handoff_t, side="left"))
    y[i0] = 2.0  # isolated spike -- the only tick off the line

    spawned = []

    def plant_factory():
        p = _StraightPlant()
        spawned.append(p)
        return p

    ts, ctes, steers = _closed_loop_trace(
        planner, t, x, y, yaw, v, (start_s, end_s), plant_factory, warmup_s)

    assert ts.size > 60  # several seconds of closed-loop ticks were collected
    np.testing.assert_allclose(ctes, 2.0, atol=1e-6)

    out = trace_statistics(ts, ctes, steers)
    assert out["mean_abs_cte"] == pytest.approx(2.0, abs=1e-6)
    assert out["max_abs_cte"] == pytest.approx(2.0, abs=1e-6)

    # Speed reached the plant through reset(): it must have actually driven
    # forward, not sat frozen at x0 with a default speed of zero.
    assert spawned[-1].x > x[i0] + 1.0


@pytest.mark.skipif(
    not os.path.isdir(REAL_RUN_DIR) or not os.path.isfile(REAL_LINE_CSV),
    reason="real bag/racing-line data not present in this checkout",
)
def test_engagement_stats_against_the_real_bag_with_data_sim():
    """Integration smoke test: build_planner + the real /mpc/status stream +
    DataSim end to end, on the actual run the ENGAGEMENTS windows describe.
    DataSim is not expected to match REAL_STATS (that gap is the acceptance
    gate this harness exists to measure) -- this only checks the pipeline
    runs on real data and returns sane, bounded numbers."""
    from sim.data_sim import DataSim
    from sim.plant_closed_loop import ENGAGEMENTS, engagement_stats, is_stable

    out = engagement_stats(REAL_RUN_DIR, ENGAGEMENTS[1], DataSim, REAL_LINE_CSV)
    assert set(out) == {"mean_abs_cte", "max_abs_cte", "sd_steer", "p90_steer_rate"}
    assert 0.0 <= out["mean_abs_cte"] <= out["max_abs_cte"]
    assert is_stable(np.array([out["max_abs_cte"]]))
    assert out["sd_steer"] > 0.0
    assert out["p90_steer_rate"] >= 0.0


def test_handoff_seeds_a_learned_plants_window_with_the_recorded_speed(tmp_path):
    """The discriminating case for a learned plant: at the handoff the kart is
    doing 4 m/s, and the model's only knowledge of that is the window it is
    handed. Under the old driver -- `reset(x, y, yaw)` then `plant.v = ...` --
    the window's speed column was four rows of zeros plus a fifth carrying a
    command, so the model was told the kart was stopped while the driver's
    `plant.speed` said otherwise. Assert on the window, not on `plant.speed`,
    which was right even then."""
    from sim.learned_plant import LearnedPlant
    from sim.plant_closed_loop import _closed_loop_trace, build_planner
    from sim.plant_dataset import HISTORY, Whitener

    run_dir, line_csv = _write_fixture_run(tmp_path)
    planner = build_planner(run_dir, line_csv)

    n = 240
    t = np.arange(n) / 60.0
    v_rec = 4.0
    x = v_rec * t
    y = np.zeros(n)
    yaw = np.zeros(n)
    v = np.full(n, v_rec)

    identity_f = Whitener(np.zeros(4), np.ones(4)).to_dict()
    identity_t = Whitener(np.zeros(2), np.ones(2)).to_dict()
    windows = []

    def plant_factory():
        plant = LearnedPlant(None, {"plant_hz": 60.0, "coverage": {},
                                    "feature_whitener": identity_f,
                                    "target_whitener": identity_t})
        # Steady straight cruise: no angular and no longitudinal acceleration.
        plant._predict = lambda w: (windows.append(w.copy()),
                                    np.array([0.0, 0.0]))[1]
        return plant

    _closed_loop_trace(planner, t, x, y, yaw, v, (0.0, t[-1]),
                       plant_factory, warmup_s=0.5)

    assert windows
    first = windows[0]
    assert first.shape == (HISTORY, 4)
    np.testing.assert_allclose(first[:, 0], v_rec)
    np.testing.assert_allclose(first[:-1, 3], v_rec)


def test_closed_loop_table_reports_every_candidate_against_the_real_row(monkeypatch):
    """The spec's second acceptance table, as a runnable artifact: one row per
    engagement per candidate, each beside the statistics the real kart
    produced over that window. Every stub returns a distinct set of numbers,
    so a table that scored one candidate under every name, or dropped the
    real row, or put a value in the wrong column, fails here."""
    import sim.plant_closed_loop as pcl

    def fake_engagement_stats(run_dir, window, factory, line_csv, warmup_s=0.5):
        return factory()

    monkeypatch.setattr(pcl, "engagement_stats", fake_engagement_stats)

    def stats(base):
        return dict(zip(pcl._STAT_COLUMNS,
                        (base + 0.1, base + 0.2, base + 0.3, base + 0.4)))

    factories = {"graybox": lambda: stats(1.0), "learned_nn": lambda: stats(2.0)}
    text = pcl.closed_loop_table("run", "line.csv", factories)

    rows = {}
    for line in text.splitlines()[1:]:
        parts = line.split()
        rows[(int(parts[0]), parts[1])] = [float(v) for v in parts[2:]]

    assert set(rows) == {(e, n) for e in pcl.ENGAGEMENTS
                         for n in ("real", "graybox", "learned_nn")}
    for engagement in pcl.ENGAGEMENTS:
        assert rows[(engagement, "real")] == pytest.approx(
            [pcl.REAL_STATS[engagement][c] for c in pcl._STAT_COLUMNS])
        assert rows[(engagement, "graybox")] == pytest.approx([1.1, 1.2, 1.3, 1.4])
        assert rows[(engagement, "learned_nn")] == pytest.approx([2.1, 2.2, 2.3, 2.4])


def test_build_factories_omits_the_network_until_its_weights_exist(tmp_path):
    from sim.plant_closed_loop import build_factories
    from sim.data_sim import DataSim
    factories = build_factories(str(tmp_path / "missing.pt"), str(tmp_path / "missing.json"))
    assert set(factories) == {"graybox"}
    assert factories["graybox"] is DataSim

    weights = tmp_path / "plant_nn.pt"
    weights.write_text("fake")
    factories = build_factories(str(weights), str(tmp_path / "plant_nn.json"))
    assert set(factories) == {"graybox", "learned_nn"}


def test_simulate_can_close_the_loop_with_pure_pursuit():
    """Pure pursuit is the controller that actually ran all season, so a plant
    that only ever meets MPC has never been tested against the thing the kart
    really used. `simulate` hardcoded MPCPlanner."""
    from sim.closed_loop import DEFAULT_PP, simulate

    line = [(float(i) * 1.0, float(i) * 1.0, 0.0, 6.0) for i in range(120)]
    result = simulate(line, DEFAULT_PP, planner_kind="pure_pursuit",
                      sim_backend="bicycle", n_laps=1, max_steps=600)
    assert not result.aborted
    assert result.xs.size > 100
    assert float(np.max(np.abs(result.ds))) < 2.0


def test_build_planner_falls_back_to_the_params_snapshot(tmp_path):
    """run_20260913_195301_auto recorded a whole session of MPC, including two
    off-track excursions, with a 0-byte params_live.yaml. The snapshot under
    params/ spells the node key without its leading slash, flattens the mpc
    block into dotted keys, and carries the kart constants under the `/**`
    wildcard, so reading it is not just a matter of trying another path."""
    from sim.plant_closed_loop import build_planner

    run = tmp_path / "run"
    (run / "params").mkdir(parents=True)
    (run / "params_live.yaml").write_text("")
    (run / "params" / "pathfinder.yaml").write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    v_max_mps: 12.0\n"
        "    wheelbase_m: 1.05\n"
        "    steer_max_deg: 60.0\n"
        "    steer_rate_max_degps: 180.0\n"
        "    a_max_mps2: 2.0\n"
        "    a_min_mps2: -3.0\n"
        "    a_lat_max_mps2: 8.0\n"
        "pathfinder_node:\n"
        "  ros__parameters:\n"
        "    mpc.horizon_steps: 20\n"
        "    mpc.dt_s: 0.05\n"
        "    mpc.num_samples: 64\n"
        "    mpc.steer_sigma_deg: 4.0\n"
        "    mpc.target_speed_mps: 6.0\n")
    line = tmp_path / "line.csv"
    line.write_text("s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps\n" + "\n".join(
        f"{i*1.0},{i*1.0},0.0,0.0,0.0,6.0" for i in range(60)))

    planner = build_planner(str(run), str(line))
    assert planner.kart.steer_rate_max_degps == 180.0
    assert planner.kart.wheelbase_m == 1.05
    assert int(planner.params["horizon_steps"]) == 20
    assert float(planner.params["steer_sigma_deg"]) == 4.0


def test_handoff_seeds_the_recorded_yaw_rate_not_just_the_speed(tmp_path):
    """The other half of the handoff, and the one that hid the MPC failures.

    `reset(x, y, yaw, v=...)` with no history leaves psi_dot at 0, so a kart
    handed over mid-corner at 0.88 rad/s is told it is going straight. The
    plant then has to build the turn from scratch, the planner sees an easier
    problem than the kart had, and three recorded off-track excursions came
    back on-track in sim. Assert on the window's psi_dot column and on the
    plant's own psi_dot, both of which were zero.
    """
    from sim.learned_plant import LearnedPlant
    from sim.plant_closed_loop import _closed_loop_trace, build_planner
    from sim.plant_dataset import HISTORY, Whitener

    run_dir, line_csv = _write_fixture_run(tmp_path)
    planner = build_planner(run_dir, line_csv)

    n = 240
    t = np.arange(n) / 60.0
    v_rec, omega_rec = 4.0, 0.8
    yaw = omega_rec * t
    x = np.cumsum(np.cos(yaw) * v_rec / 60.0)
    y = np.cumsum(np.sin(yaw) * v_rec / 60.0)
    v = np.full(n, v_rec)

    identity_f = Whitener(np.zeros(4), np.ones(4)).to_dict()
    identity_t = Whitener(np.zeros(2), np.ones(2)).to_dict()
    windows, plants = [], []

    def plant_factory():
        plant = LearnedPlant(None, {"plant_hz": 60.0, "coverage": {},
                                    "feature_whitener": identity_f,
                                    "target_whitener": identity_t})
        plant._predict = lambda w: (windows.append(w.copy()), np.array([0.0, 0.0]))[1]
        plants.append(plant)
        return plant

    _closed_loop_trace(planner, t, x, y, yaw, v, (0.0, t[-1]),
                       plant_factory, warmup_s=0.5)

    assert windows
    first = windows[0]
    assert first.shape == (HISTORY, 4)
    np.testing.assert_allclose(first[:, 1], omega_rec, atol=0.05)
    assert plants[0].psi_dot == pytest.approx(omega_rec, abs=0.05)


def test_handoff_seeds_the_wheel_angle_from_the_recorded_commands():
    """A wheel that slews at 33.7 deg/s does not centre itself at the handoff.

    Seeding the steering column with 0.0 hands the plant a centred wheel while
    the kart's was at -30 deg mid-corner, and at 33.7 deg/s it takes most of a
    second to get back there -- the whole length of the clips these are scored
    on. The wheel is integrated from the recorded command stream instead.
    """
    from sim.plant_closed_loop import _handoff_history
    from sim.plant_dataset import HISTORY, STEER_RATE_MAX_DEGPS

    n = 200
    t = np.arange(n) / 60.0
    yaw = np.zeros(n)
    v = np.full(n, 5.0)
    steer = np.full(n, -30.0)          # held hard over for seconds
    i0 = 150

    rows = _handoff_history(t, np.zeros(n), np.zeros(n), yaw, v, i0, steer=steer)
    assert len(rows) == HISTORY
    wheel = [r[2] for r in rows]
    # the wheel has had time to arrive, so it is over, not centred
    assert wheel[-1] == pytest.approx(-30.0, abs=0.6)
    assert max(abs(a - b) for a, b in zip(wheel[1:], wheel[:-1])) <= \
        STEER_RATE_MAX_DEGPS / 60.0 + 1e-9

    # with no command stream it degrades to a centred wheel rather than failing
    rows_none = _handoff_history(t, np.zeros(n), np.zeros(n), yaw, v, i0)
    assert all(r[2] == 0.0 for r in rows_none)


def test_planner_index_is_seeded_globally_before_the_warm_up():
    """A fresh planner starts its Frenet index at 0 and only searches a local
    window (proj_window_back 5, proj_window_fwd 37), so on a closed line it
    can lock half a lap from the kart and never recover. Measured against
    run_20260913_203929: the real planner sat at index 14-21 while a rebuilt
    one sat at 226-233 of 425, steering toward the wrong part of the track and
    commanding +25 deg where the kart was given -18.
    """
    from sim.plant_closed_loop import seed_planner_index, build_planner

    run = _write_fixture_run.__wrapped__ if hasattr(_write_fixture_run, "__wrapped__") \
        else None
    import tempfile, pathlib
    tmp = pathlib.Path(tempfile.mkdtemp())
    run_dir, line_csv = _write_fixture_run(tmp)
    planner = build_planner(run_dir, line_csv)
    lx = planner._static_arrays["x"]
    ly = planner._static_arrays["y"]
    assert len(lx) > 40

    target = len(lx) // 2
    planner.closest_idx = 0
    seed_planner_index(planner, float(lx[target]), float(ly[target]))
    assert abs(planner.closest_idx - target) <= 1
    assert abs(planner.static_closest_idx - target) <= 1
