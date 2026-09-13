"""The apex repro must keep reproducing the bag it is named after.

These assertions are about the harness, not about whether the MPC is fixed.
They pin `sim/fixtures/apex_params_20260519.json` (frozen) against the logged
commands in `sim/fixtures/apex_20260519_214619.npz`, so they stay true no
matter what is done to the live pathfinder.yaml. If MPCPlanner's solver
changes, these fail — which is the signal that the repro no longer matches the
kart and the fixture needs regenerating.

They deliberately assert on the SEED ENSEMBLE, never on a named seed. The
solver's output at this corner is close to noise (the first steering action
explains under 4% of the cost variance among candidates), so "seed 3 matches
the kart" is luck, not a property — it stops being true the moment anything
perturbs the RNG stream.
"""
import numpy as np
import pytest

from sim import apex_repro


@pytest.fixture(scope="module")
def baseline():
    return apex_repro.run(baseline=True, seeds=tuple(range(8)))


def _logged_apex(report):
    r = report.results[0]
    return float(np.mean(r.logged[r.apex]))


def test_ensemble_brackets_the_logged_command(baseline):
    """The kart's own run must look like a draw from the replay, not an
    outlier. On 05/19 it averaged +1.90 deg through the corner."""
    cmds = np.array([r.apex_command_deg for r in baseline.results])
    logged = _logged_apex(baseline)
    assert cmds.min() <= logged <= cmds.max(), (
        f"logged {logged:+.2f} outside the ensemble "
        f"[{cmds.min():+.2f}, {cmds.max():+.2f}]"
    )
    z = (logged - np.median(cmds)) / np.std(cmds)
    assert abs(z) < 2.0, f"logged sits at z={z:+.2f} of the ensemble"


def test_baseline_fails_the_gate(baseline):
    """The point of the fixture: the 05/19 params do not reach their own
    optimum through the corner, and do not do so repeatably."""
    assert not baseline.passed
    assert baseline.mean_abs_shortfall_deg > 15.0
    assert baseline.seed_spread_deg > 10.0


def test_optimum_and_pure_pursuit_agree_on_the_corner(baseline):
    """Two independent references for 'turn now'. If these diverge, the fixture
    or the line no longer line up and the score means nothing."""
    r = baseline.results[0]
    m = r.apex
    assert np.nanmean(r.optimum[m]) > 15.0
    assert np.nanmean(r.pursuit[m]) > 15.0


def test_the_corner_is_where_it_breaks(baseline):
    """Before the corner the solver sits near its optimum; the blow-up is
    local to the apex rather than a harness-wide offset."""
    lead = np.mean([r.lead_in_tracking_deg for r in baseline.results])
    assert lead < 10.0
    assert baseline.mean_abs_shortfall_deg > 3.0 * lead


def test_probe_does_not_perturb_the_replay():
    """_solve draws from the planner RNG even at sigma=0, so the cost-minimum
    probe must save and restore RNG state. Otherwise the commands depend on how
    often we measure them."""
    fx = apex_repro.load_fixture()
    line = apex_repro.load_line()
    mpc_p, pp_p, kart = apex_repro.load_baseline()
    mpc_p = dict(mpc_p)
    mpc_p.setdefault("residual.mode", "off")
    mpc_p["residual.cache_enabled"] = False
    sweep = np.linspace(-kart.steer_max_deg, kart.steer_max_deg, 41)
    dense = apex_repro.run_one(3, mpc_p, pp_p, kart, fx, line, sweep, 1)
    sparse = apex_repro.run_one(3, mpc_p, pp_p, kart, fx, line, sweep, 10 ** 9)
    assert dense.apex_command_deg == pytest.approx(sparse.apex_command_deg, abs=1e-9)
