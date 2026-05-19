import numpy as np

from sim.noise_model import NoiseModel, estimate_noise


def test_zero_residuals_give_small_sigma():
    n = 1000
    rng = np.random.default_rng(0)
    v_resid = rng.normal(0.0, 0.5, n)
    psi_resid = rng.normal(0.0, 0.05, n)
    cmd_throttle = rng.normal(50.0, 1.0, n)
    cmd_steer = rng.normal(0.0, 0.5, n)
    is_straight = np.abs(cmd_steer) < 1.0
    is_steady = np.abs(np.diff(cmd_throttle, prepend=cmd_throttle[0])) < 0.5
    nm = estimate_noise(v_resid, psi_resid, cmd_throttle, cmd_steer,
                        is_straight, is_steady)
    assert 0.3 < nm.sigma_v < 0.7
    assert 0.03 < nm.sigma_psidot < 0.07
    assert nm.sigma_steer_cmd > 0
    assert nm.sigma_throttle_cmd > 0


def test_serialise_roundtrip(tmp_path):
    nm = NoiseModel(sigma_v=0.4, sigma_psidot=0.05,
                    sigma_steer_cmd=0.2, sigma_throttle_cmd=0.7)
    p = tmp_path / "noise.json"
    nm.save(str(p))
    nm2 = NoiseModel.load(str(p))
    assert nm == nm2
