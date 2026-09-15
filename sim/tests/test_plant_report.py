import numpy as np
import pytest

from sim.plant_dataset import HISTORY, Segment
from sim.plant_reference import PLANT_HZ


def _straight(n=300, v=5.0, split="test"):
    t = np.arange(n) / PLANT_HZ
    return Segment(t=t, x=v * t, y=np.zeros(n), psi=np.zeros(n), v=np.full(n, v),
                   omega=np.zeros(n), delta_cmd=np.zeros(n),
                   throttle_sp=np.full(n, v), split=split)


def _turning(n=600, v=5.0, omega=0.0, split="train"):
    # Non-degenerate feature matrix (varying v/omega/cmd) so LinearARX's
    # lstsq has something to fit -- a pure straight-line fixture like
    # _straight would leave every feature column constant.
    t = np.arange(n) / PLANT_HZ
    psi = omega * t
    return Segment(t=t, x=np.cumsum(np.cos(psi) * v / PLANT_HZ),
                   y=np.cumsum(np.sin(psi) * v / PLANT_HZ), psi=psi,
                   v=np.full(n, v), omega=np.full(n, omega),
                   delta_cmd=np.full(n, omega * 30.0),
                   throttle_sp=np.full(n, v), split=split)


def _parse_rows(text):
    """{(name, steps): pos_median} from a rollout_table's printed body."""
    rows = {}
    for line in text.splitlines()[1:]:
        parts = line.split()
        if len(parts) < 8:
            continue
        name, steps = parts[0], int(parts[1])
        rows[(name, steps)] = float(parts[3])
    return rows


def test_rollout_table_names_every_candidate_and_horizon():
    from sim.plant_report import rollout_table
    dt = 1.0 / PLANT_HZ
    perfect = lambda w, d, t: (0.0, 0.0)
    frozen = lambda w, d, t: (0.0, 0.0)
    text = rollout_table({"perfect": perfect, "frozen": frozen}, [_straight()])
    assert "perfect" in text and "frozen" in text
    for h in (5, 10, 20, 60, 120):
        assert str(h) in text


def test_rollout_table_scores_each_named_stepper_independently():
    # A test that only greps for substrings would still pass against a
    # rollout_table that ignored named_steppers entirely (e.g. always
    # scoring the first stepper under every name) or that scored every
    # candidate identically (e.g. a table of zeros). Parse the pos_median
    # column and check the two candidates land on genuinely different,
    # individually-correct numbers.
    from sim.plant_report import rollout_table
    dt = 1.0 / PLANT_HZ
    seg = _straight()
    perfect = lambda w, d, t: (0.0, 0.0)
    biased = lambda w, d, t: (0.0, -0.5)
    text = rollout_table({"perfect": perfect, "biased": biased}, [seg])
    rows = _parse_rows(text)

    # A constant deceleration a falls behind by |a| * dt^2 * n(n-1)/2.
    a = 0.5
    for h in (5, 10, 20, 60, 120):
        assert rows[("perfect", h)] == pytest.approx(0.0, abs=1e-6)
        # abs tolerance: the table prints three decimals, so small values are
        # quantised by the formatting, not by the computation.
        assert rows[("biased", h)] == pytest.approx(
            a * dt * dt * h * (h - 1) / 2, abs=1e-3)
        assert rows[("perfect", h)] != pytest.approx(rows[("biased", h)])


def test_load_learned_returns_none_when_weights_are_absent():
    # Pins the "must not crash when sim/model/plant_nn.pt is absent" path.
    from sim.plant_report import load_learned
    assert load_learned("sim/model/does_not_exist.pt", "sim/model/does_not_exist.json") is None


def test_load_learned_wraps_the_plant_when_weights_are_present(tmp_path, monkeypatch):
    from sim.plant_report import load_learned

    weights = tmp_path / "plant_nn.pt"
    weights.write_text("fake")
    meta = tmp_path / "plant_nn.json"
    meta.write_text("{}")

    class FakePlant:
        def __init__(self):
            self.checked = []

        def _check_hull(self, tick):
            self.checked.append(np.asarray(tick).copy())
            return False

        def _predict(self, window):
            return np.array([1.0, 2.0])

    fake = FakePlant()
    monkeypatch.setattr("sim.plant_report.LearnedPlant.from_files",
                        lambda w, m: fake)
    stepper = load_learned(str(weights), str(meta))
    assert stepper is not None
    window = np.arange(HISTORY * 4, dtype=float).reshape(HISTORY, 4)
    assert stepper(window, 0.0, 0.0) == (1.0, 2.0)
    # The stepper must run the same hull accounting `LearnedPlant.step` does,
    # on the tick it is about to predict from.
    assert len(fake.checked) == 1
    np.testing.assert_allclose(fake.checked[0], window[-1])
    assert stepper.plant is fake


def test_build_candidates_omits_learned_nn_when_weights_are_absent(tmp_path):
    from sim.plant_report import build_candidates
    segments = [_turning(omega=0.0), _turning(omega=0.2), _turning(omega=-0.2)]
    candidates, have_nn = build_candidates(
        segments, str(tmp_path / "missing.pt"), str(tmp_path / "missing.json"))
    assert "linear_arx" in candidates
    assert "learned_nn" not in candidates
    assert have_nn is False


def test_fit_linear_returns_a_working_stepper():
    from sim.plant_report import fit_linear
    train = [_turning(omega=0.0), _turning(omega=0.2), _turning(omega=-0.2)]
    stepper = fit_linear(train)
    out = stepper(np.zeros((HISTORY, 4)), 0.0, 0.0)
    assert len(out) == 2
    assert all(np.isfinite(v) for v in out)


def test_print_report_reports_linear_only_and_says_so_plainly(tmp_path, capsys):
    # The CLI's core requirement: don't crash and don't fabricate a table
    # for a model that was never trained -- say so and move on.
    from sim.plant_report import print_report
    segments = [_turning(omega=0.0), _turning(omega=0.2), _turning(omega=-0.2, split="test")]
    missing_weights = str(tmp_path / "plant_nn.pt")
    print_report(segments, missing_weights, str(tmp_path / "plant_nn.json"))
    out = capsys.readouterr().out
    assert "not found" in out
    assert "September test windows" in out
    # The message may name "learned_nn" while explaining it's missing; the
    # table itself must not, since that would mean a fabricated row for a
    # model that was never trained.
    table = out.split("September test windows ===")[1]
    assert "linear_arx" in table
    assert "learned_nn" not in table


def test_print_report_aux_table_scores_every_segment_regardless_of_original_split(tmp_path, capsys):
    from sim.plant_report import print_report
    segments = [_turning(omega=0.0), _turning(omega=0.2), _turning(omega=-0.2, split="test")]
    aux = [_turning(omega=0.05, split="train"), _turning(omega=-0.05, split="val")]
    print_report(segments, str(tmp_path / "missing.pt"), str(tmp_path / "missing.json"),
                 aux_segments=aux)
    out = capsys.readouterr().out
    assert "auxiliary" in out
    assert "gates nothing" in out or "gate" in out
    # print_report is documented to force every aux segment's split to
    # "test" before scoring, so the whole auxiliary set gets evaluated.
    assert all(s.split == "test" for s in aux)

    # If only the segment that started as "test" were scored (the stale
    # held_out_frac=1.0 behaviour the brief warned about), the aux table's
    # sample count would match a single-segment run. Confirm it's larger.
    single = [aux[0]]
    single[0].split = "test"
    from sim.plant_report import rollout_table, fit_linear
    stepper = fit_linear(segments)
    single_n = int(rollout_table({"linear_arx": stepper}, single).splitlines()[1].split()[-1])
    both_n = int(rollout_table({"linear_arx": stepper}, aux).splitlines()[1].split()[-1])
    assert both_n == 2 * single_n


def _window(v=5.0, psi_dot=0.0, delta_cmd=0.0, throttle_sp=5.0):
    return np.tile([v, psi_dot, delta_cmd, throttle_sp], (HISTORY, 1)).astype(float)


def test_graybox_stepper_returns_accelerations():
    """The gray-box has to answer the same question as the other two
    candidates: given this window and this command, what does the kart do in
    one tick, in its own frame."""
    from sim.plant_report import graybox_stepper
    dt = 1.0 / PLANT_HZ
    step = graybox_stepper(dt)

    alpha, a_long = step(_window(v=5.0, throttle_sp=5.0), 0.0, 5.0)
    # Straight, already at the commanded speed: no acceleration either way.
    assert alpha == pytest.approx(0.0, abs=1e-6)
    assert abs(a_long) < 1e-4

    # Steering one way accelerates yaw one way, and the mirrored command
    # mirrors it. This is the property the learned plants failed.
    left = step(_window(v=5.0), 10.0, 5.0)
    right = step(_window(v=5.0), -10.0, 5.0)
    assert left[0] > 0.0
    assert right[0] == pytest.approx(-left[0], rel=1e-9)


def test_graybox_stepper_is_stateless_and_reads_the_windows_yaw_rate():
    """One DataSim is reused across calls, so anything it keeps -- pose,
    speed, the wheel angle -- has to be re-seeded every time or the rollout
    scores an instrument that drifts. The wheel angle is the gray-box's only
    memory, and it is reconstructed from the window, so two windows that
    differ only in psi_dot must give different answers."""
    from sim.plant_report import graybox_stepper
    step = graybox_stepper()

    once = step(_window(v=6.0), 12.0, 6.0)
    step(_window(v=6.0), -40.0, 2.0)          # a very different call between
    twice = step(_window(v=6.0), 12.0, 6.0)
    assert twice == pytest.approx(once)

    # Zero steer from a turning state must decelerate the yaw back toward
    # straight, so its angular acceleration is the more negative of the two.
    turning = step(_window(v=6.0, psi_dot=0.5), 0.0, 6.0)
    straight = step(_window(v=6.0, psi_dot=0.0), 0.0, 6.0)
    assert turning[0] < straight[0] - 1e-6


def test_build_candidates_always_carries_the_graybox_baseline(tmp_path):
    from sim.plant_report import build_candidates
    segments = [_turning(omega=0.0), _turning(omega=0.2), _turning(omega=-0.2)]
    candidates, have_nn = build_candidates(
        segments, str(tmp_path / "missing.pt"), str(tmp_path / "missing.json"))
    assert set(candidates) == {"graybox", "linear_arx"}
    assert have_nn is False


def test_print_report_announces_the_extrapolation_rate(tmp_path, capsys, monkeypatch):
    """`out_of_hull` was incremented and never read by anybody. The table now
    says how much of it was scored outside the training data."""
    from sim.learned_plant import LearnedPlant
    from sim.plant_dataset import Whitener
    from sim.plant_report import print_report

    weights = tmp_path / "plant_nn.pt"
    weights.write_text("fake")
    identity_f = Whitener(np.zeros(4), np.ones(4)).to_dict()
    identity_t = Whitener(np.zeros(2), np.ones(2)).to_dict()
    plant = LearnedPlant(None, {"plant_hz": PLANT_HZ,
                                "feature_whitener": identity_f,
                                "target_whitener": identity_t,
                                # Every scored window sits outside this.
                                "coverage": {"v": [100.0, 200.0]}})
    plant._predict = lambda window: np.zeros(2)
    monkeypatch.setattr("sim.plant_report.LearnedPlant.from_files",
                        lambda w, m: plant)

    segments = [_turning(omega=0.0), _turning(omega=0.2), _turning(omega=-0.2, split="test")]
    print_report(segments, str(weights), str(tmp_path / "plant_nn.json"))
    out = capsys.readouterr().out
    assert "outside the training hull" in out
    assert "learned_nn: " in out
    line = [ln for ln in out.splitlines() if "outside the training hull" in ln][0]
    n_out, n_total = int(line.split()[1]), int(line.split()[3])
    assert n_out == n_total > 0
    assert "(100.0%)" in line


def test_command_sensitivity_separates_a_plant_that_steers_from_one_that_does_not():
    """The acceptance check rollout error could not make.

    Both plants trained against the old `dpsi`/`ds` targets scored well at
    every horizon while carrying essentially no command sensitivity: 0.00000
    rad/s per degree for the linear fit and 0.00081 for the network, against
    0.02514 for the gray-box. A plant that ignores its steering command is
    useless to a planner no matter what the rollout table says, so the number
    is measured directly.
    """
    from sim.plant_report import command_sensitivity

    seg = _turning(n=300, v=5.0, omega=0.2, split="test")

    def deaf(window, delta_cmd, throttle_sp):
        return 0.5, 0.1

    def responsive(window, delta_cmd, throttle_sp):
        return 0.03 * window[-1, 2], 0.4 * window[-1, 3]

    steps = 20
    held = steps / PLANT_HZ  # each tick's response accumulates over the probe

    assert command_sensitivity(deaf, [seg], steps=steps)["steer"] == pytest.approx(0.0)
    live = command_sensitivity(responsive, [seg], steps=steps)
    assert live["steer"] == pytest.approx(0.03 * held, rel=1e-6)
    assert live["throttle"] == pytest.approx(0.4 * held, rel=1e-6)
