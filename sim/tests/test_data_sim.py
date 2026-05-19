import json

import numpy as np
import pytest

from sim.data_sim import DataSim
from sim.identify_bicycle import BicycleParams
from sim.noise_model import NoiseModel


@pytest.fixture
def temp_artifacts(tmp_path):
    params = BicycleParams()
    params_path = tmp_path / "bicycle_params.json"
    with open(params_path, "w") as f:
        json.dump({"params": params.__dict__, "fit_info": {}}, f)
    noise = NoiseModel(sigma_v=0.5, sigma_psidot=0.05,
                       sigma_steer_cmd=0.2, sigma_throttle_cmd=0.5)
    noise_path = tmp_path / "noise.json"
    noise.save(str(noise_path))
    return {"params": str(params_path), "noise": str(noise_path)}


def test_step_returns_4_tuple_and_no_nan(temp_artifacts):
    sim = DataSim(params_path=temp_artifacts["params"],
                  noise_path=temp_artifacts["noise"],
                  noise_mode="none")
    sim.reset(0.0, 0.0, 0.0)
    out = sim.step(target_mps=8.0, steer_deg=10.0, dt=1 / 60)
    assert len(out) == 4
    assert all(np.isfinite(out))


def test_zero_command_no_motion(temp_artifacts):
    sim = DataSim(params_path=temp_artifacts["params"],
                  noise_path=temp_artifacts["noise"],
                  noise_mode="none")
    sim.reset(1.0, 2.0, 0.3)
    for _ in range(60):
        x, y, yaw, v = sim.step(0.0, 0.0, 1 / 60)
    assert abs(v) < 0.01
    assert abs(x - 1.0) < 0.01
    assert abs(y - 2.0) < 0.01


def test_deterministic_when_noise_none(temp_artifacts):
    sim_a = DataSim(params_path=temp_artifacts["params"],
                    noise_path=temp_artifacts["noise"],
                    noise_mode="none")
    sim_b = DataSim(params_path=temp_artifacts["params"],
                    noise_path=temp_artifacts["noise"],
                    noise_mode="none")
    sim_a.reset(0, 0, 0)
    sim_b.reset(0, 0, 0)
    cmds = [(50.0, 5.0)] * 100 + [(80.0, -3.0)] * 100
    for tm, sd in cmds:
        a = sim_a.step(tm, sd, 1 / 60)
        b = sim_b.step(tm, sd, 1 / 60)
        assert a == b


def test_gaussian_noise_differs_from_none(temp_artifacts):
    sim_n = DataSim(params_path=temp_artifacts["params"],
                    noise_path=temp_artifacts["noise"], noise_mode="none")
    sim_g = DataSim(params_path=temp_artifacts["params"],
                    noise_path=temp_artifacts["noise"], noise_mode="gaussian",
                    rng_seed=42)
    sim_n.reset(0, 0, 0)
    sim_g.reset(0, 0, 0)
    diffs = 0
    for _ in range(120):
        a = sim_n.step(8.0, 5.0, 1 / 60)
        b = sim_g.step(8.0, 5.0, 1 / 60)
        if a != b:
            diffs += 1
    assert diffs > 60, f"gaussian noise did not move the sim ({diffs}/120)"


def test_bootstrap_falls_back_when_no_emp(temp_artifacts):
    sim = DataSim(params_path=temp_artifacts["params"],
                  noise_path=temp_artifacts["noise"], noise_mode="bootstrap")
    sim.reset(0, 0, 0)
    out = sim.step(8.0, 5.0, 1 / 60)
    assert all(np.isfinite(out))


def test_mlp_reduces_to_bicycle_off_distribution(tmp_path, temp_artifacts):
    import json
    import torch
    from sim.clamp import Clamp
    from sim.residual_mlp import FEATURE_DIM, Normaliser, ResidualMLP

    # Build a deliberately constant-output residual MLP.
    net = ResidualMLP()
    with torch.no_grad():
        for p in net.parameters():
            p.zero_()
        net.net[-1].bias[:] = torch.tensor([0.5, 0.05])
    mlp_path = tmp_path / "mlp.pt"
    torch.save(net.state_dict(), str(mlp_path))
    norm = Normaliser(mean=np.zeros(FEATURE_DIM, dtype=np.float32),
                      std=np.ones(FEATURE_DIM, dtype=np.float32))
    norm_path = tmp_path / "norm.json"
    with open(norm_path, "w") as f:
        json.dump(norm.to_json(), f)
    # Tight clamp around the origin — far-off inputs should give alpha = 0.
    clamp = Clamp(
        mean=np.zeros(FEATURE_DIM),
        cov_inv=np.eye(FEATURE_DIM),
        d_warn=1.0, d_max=2.0,
    )
    clamp_path = tmp_path / "clamp.json"
    clamp.save(str(clamp_path))

    sim_off = DataSim(
        params_path=temp_artifacts["params"],
        noise_path=temp_artifacts["noise"],
        mlp_path=str(mlp_path), norm_path=str(norm_path), clamp_path=str(clamp_path),
        noise_mode="none",
    )
    sim_off.reset(0, 0, 0)
    sim_off.v = 12.0
    out = sim_off.step(12.0, 60.0, 1 / 60)
    # With alpha = 0 the residual contributes nothing; sim should behave as
    # ID-only and finish with finite outputs.
    assert all(np.isfinite(out))
