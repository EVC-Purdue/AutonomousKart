import numpy as np

from sim.identify_bicycle import (
    BicycleParams,
    fit_params,
    predict_derivatives,
)


def _synth_run(p: BicycleParams, dt: float, n: int, seed: int = 0):
    rng = np.random.default_rng(seed)
    cmd_throttle = 30.0 + 30.0 * np.sin(np.linspace(0, 12 * np.pi, n)) + rng.normal(0, 2, n)
    cmd_steer = 10.0 * np.sin(np.linspace(0, 25 * np.pi, n))
    cmd_throttle = np.clip(cmd_throttle, 0.0, 100.0)
    # Use the full-array predict_derivatives (same path as fit_params residuals) so
    # the synthetic data is exactly self-consistent with the model.
    v = np.zeros(n)
    dv_dt_obs, _ = predict_derivatives(p, v, cmd_throttle, cmd_steer, dt)
    for i in range(n - 1):
        v[i + 1] = v[i] + dv_dt_obs[i] * dt
    dv_dt_obs, psi_dot_obs = predict_derivatives(p, v, cmd_throttle, cmd_steer, dt)
    return v, cmd_throttle, cmd_steer, dv_dt_obs, psi_dot_obs


def test_recovers_known_params_no_noise():
    truth = BicycleParams(
        accel_tau=0.6, brake_tau=0.3,
        throttle_delay_s=0.05, steer_delay_s=0.05,
        steer_gain=0.4, steer_slop_deg=1.0,
        wheelbase_m=1.05, steer_rate_max_degps=180.0,
        slip_angle_at_v=0.0, v_max_mps=12.0,
    )
    dt = 1.0 / 60.0
    v, ct, cs, dv, ps = _synth_run(truth, dt, n=4000, seed=0)
    fit, info = fit_params(v, ct, cs, dv, ps, dt, prior=BicycleParams(v_max_mps=12.0))
    assert info["success"]
    # Tolerances are loose because the slop+rate-limit makes the surface non-smooth.
    assert abs(fit.accel_tau - truth.accel_tau) < 0.15
    assert abs(fit.brake_tau - truth.brake_tau) < 0.15
    assert abs(fit.steer_gain - truth.steer_gain) < 0.1
    assert abs(fit.wheelbase_m - truth.wheelbase_m) < 0.1
