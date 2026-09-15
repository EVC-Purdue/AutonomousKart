import numpy as np
import pytest

from sim.plant_dataset import HISTORY, Whitener
from sim.plant_models import LinearARX, as_stepper


def _linear_data(n=4000, seed=0):
    rng = np.random.default_rng(seed)
    feats = rng.normal(0.0, 1.0, size=(n, HISTORY, 4))
    w = rng.normal(0.0, 0.5, size=(HISTORY * 4, 2))
    tg = feats.reshape(n, -1) @ w + 0.25
    return feats, tg, w


def test_linear_arx_recovers_a_linear_map():
    feats, tg, _ = _linear_data()
    m = LinearARX().fit(feats, tg)
    assert np.abs(m.predict(feats) - tg).max() < 1e-8


def test_linear_arx_recovers_distinct_per_target_intercepts():
    # The shared fixture above adds the same scalar (0.25) to every target
    # column, so a bug that collapsed the intercept to a single shared
    # scalar -- or transposed/averaged it across targets -- would still
    # pass. Use a distinct intercept per column so recovering the wrong
    # one is visible.
    rng = np.random.default_rng(1)
    n = 4000
    feats = rng.normal(0.0, 1.0, size=(n, HISTORY, 4))
    w = rng.normal(0.0, 0.5, size=(HISTORY * 4, 2))
    b = np.array([0.25, -1.5])
    tg = feats.reshape(n, -1) @ w + b
    m = LinearARX().fit(feats, tg)
    assert np.allclose(m.intercept, b, atol=1e-8)
    assert np.abs(m.predict(feats) - tg).max() < 1e-8


def test_linear_arx_has_84_parameters():
    feats, tg, _ = _linear_data(n=200)
    m = LinearARX().fit(feats, tg)
    assert m.n_params == HISTORY * 4 * 2 + 2


def test_linear_arx_round_trips():
    feats, tg, _ = _linear_data(n=200)
    m = LinearARX().fit(feats, tg)
    m2 = LinearARX.from_dict(m.to_dict())
    assert np.allclose(m2.predict(feats), m.predict(feats))


def test_as_stepper_undoes_whitening():
    feats, tg, _ = _linear_data(n=500)
    fw = Whitener().fit(feats)
    tw = Whitener().fit(tg)
    m = LinearARX().fit(fw.apply(feats), tw.apply(tg))
    step = as_stepper(m, fw, tw)
    got = np.asarray(step(feats[0], feats[0, -1, 2], feats[0, -1, 3]))
    assert np.allclose(got, tg[0], atol=1e-6)


def test_mlp_is_about_two_thousand_parameters():
    from sim.plant_models import PlantMLP
    m = PlantMLP(hidden=32)
    assert 1500 < m.n_params < 2500


def test_mlp_forward_shape():
    import torch
    from sim.plant_models import PlantMLP
    m = PlantMLP(hidden=32)
    out = m(torch.zeros(7, HISTORY, 4))
    assert tuple(out.shape) == (7, 2)


def test_ensemble_mean_matches_hand_average(tmp_path):
    import torch
    from sim.plant_models import PlantEnsemble, PlantMLP
    torch.manual_seed(0)
    ens = PlantEnsemble([PlantMLP(hidden=8) for _ in range(3)])
    feats = np.zeros((4, HISTORY, 4))
    by_hand = np.mean([
        mem(torch.as_tensor(feats, dtype=torch.float32)).detach().numpy()
        for mem in ens.members], axis=0)
    assert np.allclose(ens.predict(feats), by_hand, atol=1e-6)


def test_ensemble_mean_differs_from_any_single_member():
    # The fixture above feeds an all-zero input, and PlantMLP has no bias
    # in its LayerNorm-preceded first linear layer that depends on the
    # *identity* of the member, but distinct random weight inits still
    # give each member a distinct output on the same input. Assert the
    # ensemble mean is not simply equal to one member's output, so a
    # `predict` that quietly returns e.g. `members[0](x)` is caught even
    # if the hand-averaged fixture above were weakened later.
    import torch
    from sim.plant_models import PlantEnsemble, PlantMLP
    torch.manual_seed(0)
    ens = PlantEnsemble([PlantMLP(hidden=8) for _ in range(3)])
    feats = np.zeros((4, HISTORY, 4))
    pred = ens.predict(feats)
    for mem in ens.members:
        with torch.no_grad():
            single = mem(torch.as_tensor(feats, dtype=torch.float32)).numpy()
        assert not np.allclose(pred, single, atol=1e-6)


def test_ensemble_round_trips(tmp_path):
    from sim.plant_models import PlantEnsemble, PlantMLP
    ens = PlantEnsemble([PlantMLP(hidden=8) for _ in range(2)])
    p = tmp_path / "ens.pt"
    ens.save(str(p))
    feats = np.zeros((3, HISTORY, 4))
    assert np.allclose(PlantEnsemble.load(str(p)).predict(feats), ens.predict(feats))
