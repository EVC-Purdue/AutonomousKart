import numpy as np

from sim.rtk_reference import reference
from sim.sensor_noise import (
    ChannelNoise,
    ChannelSampler,
    SensorNoiseModel,
    contiguous,
    fit_channel,
    scale_to_core,
    split_white_ou,
    tail_stats,
)


def _ou(n, sigma, tau, dt, rng):
    a = np.exp(-dt / tau)
    step = sigma * np.sqrt(1.0 - a * a)
    out = np.empty(n)
    cur = rng.normal(0.0, sigma)
    for k in range(n):
        out[k] = cur
        cur = a * cur + rng.normal(0.0, step)
    return out


def test_split_recovers_a_known_white_plus_ou_mix():
    rng = np.random.default_rng(0)
    dt = 0.01
    n = 200_000
    residual = _ou(n, 0.10, 0.20, dt, rng) + rng.normal(0.0, 0.05, n)
    sigma_white, sigma_ou, tau = split_white_ou(residual, dt)
    assert 0.03 < sigma_white < 0.07
    assert 0.08 < sigma_ou < 0.12
    assert 0.15 < tau < 0.28


def test_split_reports_a_white_residual_as_white():
    rng = np.random.default_rng(1)
    sigma_white, sigma_ou, _ = split_white_ou(rng.normal(0.0, 0.2, 100_000), 0.01)
    assert 0.18 < sigma_white < 0.22
    assert sigma_ou < 0.05


def test_fit_channel_recovers_scale_and_bias():
    rng = np.random.default_rng(2)
    truth = rng.normal(0.0, 2.0, 50_000)
    measured = -1.02 * truth + 0.3 + rng.normal(0.0, 0.1, truth.size)
    fit = fit_channel(truth, measured, 0.01, 100.0)
    assert abs(fit.scale + 1.02) < 0.01
    assert abs(fit.bias - 0.3) < 0.01
    assert 0.08 < fit.sigma_core < 0.12
    assert fit.r2 > 0.99


def test_fix_bias_pins_the_intercept():
    rng = np.random.default_rng(3)
    truth = rng.uniform(3.0, 5.0, 20_000)
    measured = 1.03 * truth + rng.normal(0.0, 0.05, truth.size)
    fit = fit_channel(truth, measured, 0.01, 100.0, fix_bias=True)
    assert fit.bias == 0.0
    assert abs(fit.scale - 1.03) < 0.01


def test_tail_stats_separates_a_core_from_excursions():
    rng = np.random.default_rng(4)
    residual = rng.normal(0.0, 0.1, 20_000)
    residual[::100] += 5.0            # 1% of samples thrown far out
    core, rate, tail_rms = tail_stats(residual)
    assert 0.09 < core < 0.11         # the core survives the excursions
    assert 0.005 < rate < 0.02
    assert 4.5 < tail_rms < 5.5       # and the excursions keep their own size


def test_scale_to_core_keeps_the_ratio():
    white, ou = scale_to_core(0.3, 0.4, 0.25)
    assert abs(np.hypot(white, ou) - 0.25) < 1e-9
    assert abs(white / ou - 0.75) < 1e-9


def test_sampler_reproduces_its_own_parameters():
    channel = ChannelNoise(scale=0.0, bias=0.0, sigma_white=0.05,
                           sigma_ou=0.10, tau_s=0.20, rate_hz=100.0)
    drawn = ChannelSampler(channel, 0.01, np.random.default_rng(5)).sample(np.zeros(200_000))
    sigma_white, sigma_ou, tau = split_white_ou(drawn, 0.01)
    assert 0.03 < sigma_white < 0.07
    assert 0.08 < sigma_ou < 0.12
    assert 0.15 < tau < 0.28


def test_sampler_applies_scale_and_bias():
    channel = ChannelNoise(scale=-1.0, bias=0.5, rate_hz=100.0)
    drawn = ChannelSampler(channel, 0.01, np.random.default_rng(6)).sample(np.arange(5.0))
    assert np.allclose(drawn, -np.arange(5.0) + 0.5)


def test_model_roundtrip(tmp_path):
    model = SensorNoiseModel(gyro_z=ChannelNoise(scale=-1.01, sigma_ou=0.08, tau_s=0.17))
    model.gyro_bias_still = (0.1, 0.2, 0.3)
    path = tmp_path / "sensors.json"
    model.save(str(path))
    loaded = SensorNoiseModel.load(str(path))
    # repr, not ==, because the unset r2 fields are NaN and NaN != NaN.
    assert repr(loaded) == repr(model)
    assert loaded.gyro_z.scale == -1.01
    assert loaded.gyro_bias_still == (0.1, 0.2, 0.3)


def test_contiguous_splits_on_gaps_and_drops_short_runs():
    t = np.arange(0, 10.0, 0.1)
    mask = np.zeros(len(t), dtype=bool)
    mask[0:10] = True      # 0.9 s, too short
    mask[20:80] = True     # 5.9 s, kept
    assert contiguous(mask, t, 4.0) == [(20, 80)]


def test_reference_recovers_a_constant_turn():
    dt = 0.1
    t = np.arange(0.0, 20.0, dt)
    speed, omega = 6.0, 0.25
    heading = omega * t
    x = (speed / omega) * np.sin(heading)
    y = (speed / omega) * (1.0 - np.cos(heading))
    rng = np.random.default_rng(7)
    var = np.full(len(t), 0.03 ** 2)
    ref = reference(t, x + rng.normal(0, 0.03, len(t)),
                    y + rng.normal(0, 0.03, len(t)), var, var)
    mid = slice(20, len(t) - 20)
    assert abs(np.median(ref.speed[mid]) - speed) < 0.15
    assert abs(np.median(ref.psi_dot[mid]) - omega) < 0.02
    assert abs(np.median(ref.accel_long[mid])) < 0.15


def test_reference_rejects_a_position_glitch():
    dt = 0.1
    t = np.arange(0.0, 20.0, dt)
    x = 5.0 * t
    y = np.zeros_like(t)
    x[100] += 50.0
    var = np.full(len(t), 0.03 ** 2)
    ref = reference(t, x, y, var, var)
    assert ref.rejected[100]
    assert abs(ref.x[100] - 5.0 * t[100]) < 1.0
