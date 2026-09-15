import numpy as np
from sim.plant_reference import PLANT_HZ, fuse
from sim.sensor_noise import SensorNoiseModel


def _model():
    m = SensorNoiseModel.load("sim/model/sensors.json")
    return m


def _constant_turn(dur=20.0, v=6.0, omega=0.25):
    """Ground truth: constant speed, constant yaw rate."""
    t = np.arange(0.0, dur, 1.0 / PLANT_HZ)
    psi = omega * t
    x = (v / omega) * np.sin(psi)
    y = (v / omega) * (1.0 - np.cos(psi))
    return t, x, y, psi


def test_fuse_recovers_a_constant_turn():
    t, x, y, psi = _constant_turn()
    m = _model()
    rng = np.random.default_rng(0)
    # GPS at 10 Hz with its fitted noise, gyro and wheel at 100 Hz with theirs.
    gi = np.arange(0, len(t), int(PLANT_HZ / 10))
    gps_t = t[gi]
    gps_x = x[gi] + rng.normal(0, m.gps_x.sigma_core, len(gi))
    gps_y = y[gi] + rng.normal(0, m.gps_y.sigma_core, len(gi))
    gps_var = np.full(len(gi), m.gps_x.sigma_core ** 2)
    hi = np.arange(0, len(t), int(PLANT_HZ / 100) or 1)
    gyro_t = t[hi]
    gyro_z = m.gyro_z.scale * 0.25 + rng.normal(0, m.gyro_z.sigma_core, len(hi))
    wheel_t = t[hi]
    wheel_v = m.wheel_speed.scale * 6.0 + rng.normal(0, m.wheel_speed.sigma_core, len(hi))

    ref = fuse(gps_t, gps_x, gps_y, gps_var, gps_t, psi[gi], gyro_t, gyro_z,
               wheel_t, wheel_v, m)

    mid = slice(len(ref.t) // 4, -len(ref.t) // 4)
    assert abs(np.median(ref.v[mid]) - 6.0) < 0.10
    assert abs(np.median(ref.omega[mid]) - 0.25) < 0.01
    assert np.median(np.hypot(ref.x[mid] - np.interp(ref.t[mid], t, x),
                              ref.y[mid] - np.interp(ref.t[mid], t, y))) < 0.10


def test_fuse_output_is_on_a_uniform_60hz_grid():
    t, x, y, psi = _constant_turn(dur=5.0)
    m = _model()
    gi = np.arange(0, len(t), 6)
    ref = fuse(t[gi], x[gi], y[gi], np.full(len(gi), 9e-4), t[gi], psi[gi],
               t, np.full(len(t), 0.25), t, np.full(len(t), 6.0), m)
    dt = np.diff(ref.t)
    assert np.allclose(dt, 1.0 / PLANT_HZ, atol=1e-9)


def _cumtrapz(y, dt):
    """Cumulative trapezoidal integral, zero at the first sample."""
    out = np.zeros_like(y)
    out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * dt)
    return out


def _rms(a):
    return float(np.sqrt(np.mean(np.square(a))))


def _lagged(sample_t, truth_t, truth, lag):
    """The value a sensor sample stamped ``sample_t`` actually carries when
    the channel trails truth by ``lag`` seconds: a sample stamped T describes
    truth at T - lag (see sim/plant_reference.fuse's docstring for the
    derivation from sim.sensor_noise.best_lag)."""
    return np.interp(sample_t - lag, truth_t, truth)


def _oscillating_yaw_rate(dur=6.0, v=6.0, omega_amp=0.4, omega_freq=0.5):
    """Ground truth: constant speed, sinusoidal yaw rate."""
    t = np.arange(0.0, dur, 1.0 / PLANT_HZ)
    dt = 1.0 / PLANT_HZ
    omega = omega_amp * np.sin(2.0 * np.pi * omega_freq * t)
    psi = _cumtrapz(omega, dt)
    x = _cumtrapz(v * np.cos(psi), dt)
    y = _cumtrapz(v * np.sin(psi), dt)
    return t, x, y, psi, omega


def _oscillating_speed(dur=6.0, omega=0.25, v0=6.0, v_amp=1.0, v_freq=3.0):
    """Ground truth: constant yaw rate, sinusoidal speed."""
    t = np.arange(0.0, dur, 1.0 / PLANT_HZ)
    dt = 1.0 / PLANT_HZ
    v = v0 + v_amp * np.sin(2.0 * np.pi * v_freq * t)
    psi = omega * t
    x = _cumtrapz(v * np.cos(psi), dt)
    y = _cumtrapz(v * np.sin(psi), dt)
    return t, x, y, psi, v


def test_fuse_needs_the_gyro_to_track_fast_yaw():
    """0.5 Hz, not higher: the fix makes the gyro help at every frequency
    tested up to 5 Hz (see task-1-report.md), so the frequency here is
    capped by what the kart's yaw actually does, not by what fusion can
    resolve. The measured yaw-rate autocorrelation falls below 0.5 at 1.13 s,
    so real yaw content sits well under 1 Hz, and 0.5 Hz is already aliased
    away by 10 Hz course/GPS. Threshold (0.08 rad/s) chosen by ablation
    against sim/model/sensors.json's fitted gyro noise and lag; see
    task-1-report.md for the with/without-gyro RMS that set it."""
    t, x, y, psi, omega = _oscillating_yaw_rate()
    m = _model()
    rng = np.random.default_rng(1)
    gi = np.arange(0, len(t), int(PLANT_HZ / 10))
    gps_t = t[gi]
    gps_x = x[gi] + rng.normal(0, m.gps_x.sigma_core, len(gi))
    gps_y = y[gi] + rng.normal(0, m.gps_y.sigma_core, len(gi))
    gps_var = np.full(len(gi), m.gps_x.sigma_core ** 2)
    course = m.vtg_course.scale * psi[gi] + rng.normal(0, m.vtg_course.sigma_core, len(gi))
    hi = np.arange(0, len(t), int(PLANT_HZ / 100) or 1)
    gyro_t = t[hi]
    gyro_z = (m.gyro_z.scale * _lagged(gyro_t, t, omega, m.gyro_z.lag_s)
              + rng.normal(0, m.gyro_z.sigma_core, len(hi)))
    wheel_t = t[hi]
    wheel_v = (m.wheel_speed.scale * _lagged(wheel_t, t, np.full(len(t), 6.0), m.wheel_speed.lag_s)
               + rng.normal(0, m.wheel_speed.sigma_core, len(hi)))

    ref = fuse(gps_t, gps_x, gps_y, gps_var, gps_t, course, gyro_t, gyro_z,
               wheel_t, wheel_v, m)

    mid = slice(len(ref.t) // 4, -len(ref.t) // 4)
    omega_truth_mid = np.interp(ref.t[mid], t, omega)
    assert _rms(ref.omega[mid] - omega_truth_mid) < 0.08


def test_fuse_needs_the_wheel_speed_to_track_fast_speed_changes():
    """A 3 Hz speed oscillation is aliased away by 10 Hz course/GPS; only the
    100 Hz wheel speed resolves it. Threshold (0.45 m/s) chosen by ablation
    against sim/model/sensors.json's fitted wheel noise; see task-1-report.md
    for the with/without-wheel RMS that set it."""
    t, x, y, psi, v = _oscillating_speed()
    m = _model()
    rng = np.random.default_rng(2)
    gi = np.arange(0, len(t), int(PLANT_HZ / 10))
    gps_t = t[gi]
    gps_x = x[gi] + rng.normal(0, m.gps_x.sigma_core, len(gi))
    gps_y = y[gi] + rng.normal(0, m.gps_y.sigma_core, len(gi))
    gps_var = np.full(len(gi), m.gps_x.sigma_core ** 2)
    course = m.vtg_course.scale * psi[gi] + rng.normal(0, m.vtg_course.sigma_core, len(gi))
    hi = np.arange(0, len(t), int(PLANT_HZ / 100) or 1)
    gyro_t = t[hi]
    gyro_z = (m.gyro_z.scale * _lagged(gyro_t, t, np.full(len(t), 0.25), m.gyro_z.lag_s)
              + rng.normal(0, m.gyro_z.sigma_core, len(hi)))
    wheel_t = t[hi]
    wheel_v = (m.wheel_speed.scale * _lagged(wheel_t, t, v, m.wheel_speed.lag_s)
               + rng.normal(0, m.wheel_speed.sigma_core, len(hi)))

    ref = fuse(gps_t, gps_x, gps_y, gps_var, gps_t, course, gyro_t, gyro_z,
               wheel_t, wheel_v, m)

    mid = slice(len(ref.t) // 4, -len(ref.t) // 4)
    v_truth_mid = np.interp(ref.t[mid], t, v)
    assert _rms(ref.v[mid] - v_truth_mid) < 0.45


def test_fuse_corrects_the_gyro_transport_lag():
    """A 2 Hz yaw-rate oscillation, chosen (not the 0.5 Hz used above) so a
    90 ms transport-lag misalignment is a large fraction of a period and the
    sign of the correction actually matters: at 0.5 Hz correct vs. inverted
    lag handling are indistinguishable (both ~0.056 rad/s RMS, see
    task-1-report.md), while at 2 Hz they separate by roughly 8x. The
    synthetic gyro stream carries a genuine lag baked in (sample stamped t
    holds truth from t - gyro_z.lag_s, via _lagged), so only the correct sign
    of the correction in fuse() recovers it. Threshold (0.15 rad/s) chosen by
    ablation against a sign-flipped fuse(); see task-1-report.md."""
    t, x, y, psi, omega = _oscillating_yaw_rate(omega_freq=2.0)
    m = _model()
    rng = np.random.default_rng(3)
    gi = np.arange(0, len(t), int(PLANT_HZ / 10))
    gps_t = t[gi]
    gps_x = x[gi] + rng.normal(0, m.gps_x.sigma_core, len(gi))
    gps_y = y[gi] + rng.normal(0, m.gps_y.sigma_core, len(gi))
    gps_var = np.full(len(gi), m.gps_x.sigma_core ** 2)
    course = m.vtg_course.scale * psi[gi] + rng.normal(0, m.vtg_course.sigma_core, len(gi))
    hi = np.arange(0, len(t), int(PLANT_HZ / 100) or 1)
    gyro_t = t[hi]
    gyro_z = (m.gyro_z.scale * _lagged(gyro_t, t, omega, m.gyro_z.lag_s)
              + rng.normal(0, m.gyro_z.sigma_core, len(hi)))
    wheel_t = t[hi]
    wheel_v = (m.wheel_speed.scale * _lagged(wheel_t, t, np.full(len(t), 6.0), m.wheel_speed.lag_s)
               + rng.normal(0, m.wheel_speed.sigma_core, len(hi)))

    ref = fuse(gps_t, gps_x, gps_y, gps_var, gps_t, course, gyro_t, gyro_z,
               wheel_t, wheel_v, m)

    mid = slice(len(ref.t) // 4, -len(ref.t) // 4)
    omega_truth_mid = np.interp(ref.t[mid], t, omega)
    assert _rms(ref.omega[mid] - omega_truth_mid) < 0.15


def _straight_streams(dur=8.0, v=6.0):
    """A dead-straight run: 10 Hz GPS on y = 0, 100 Hz gyro and wheel."""
    gt = np.arange(0.0, dur, 0.1)
    ht = np.arange(0.0, dur, 0.01)
    return (gt, v * gt, np.zeros(len(gt)), ht, np.zeros(len(ht)),
            ht, np.full(len(ht), v))


def test_fuse_weights_each_fix_by_its_own_reported_covariance():
    """`moving_stints` keeps non-RTK fixes inside a stint because "the
    smoother down-weights them through their reported covariance". A single
    median variance for the whole stint does not: a 2 m float fix then pulls
    as hard as a 2 cm fixed one. One fix here is 2 m off the line and says so,
    at 2 m sigma against 2 cm everywhere else."""
    m = _model()
    gt, gx, gy, gyro_t, gyro_z, wheel_t, wheel_v = _straight_streams()
    var = np.full(len(gt), 4e-4)
    k = len(gt) // 2
    gy = gy.copy()
    gy[k] = 2.0
    var[k] = 4.0
    course = np.zeros(len(gt))

    ref = fuse(gt, gx, gy, var, gt, course, gyro_t, gyro_z, wheel_t, wheel_v,
               m, gps_var_y=var)
    assert float(np.max(np.abs(ref.y))) < 0.01

    # Collapsing the same covariances to their median -- what the stint used
    # to do -- lets the discarded fix drag the reference off the line.
    flat = np.full(len(gt), float(np.median(var)))
    ref_median = fuse(gt, gx, gy, flat, gt, course, gyro_t, gyro_z, wheel_t,
                      wheel_v, m, gps_var_y=flat)
    assert float(np.max(np.abs(ref_median.y))) > 0.1


def test_fuse_reads_the_y_covariance_for_the_y_update():
    """A fix can be good in x and bad in y. `gps_var_y` was carried through
    the pipeline and never read, so the y update ran on the x variance."""
    m = _model()
    gt, gx, gy, gyro_t, gyro_z, wheel_t, wheel_v = _straight_streams()
    var_x = np.full(len(gt), 4e-4)
    var_y = np.full(len(gt), 4e-4)
    k = len(gt) // 2
    gy = gy.copy()
    gy[k] = 2.0
    var_y[k] = 4.0                       # only y is untrustworthy on this fix
    course = np.zeros(len(gt))

    ref = fuse(gt, gx, gy, var_x, gt, course, gyro_t, gyro_z, wheel_t, wheel_v,
               m, gps_var_y=var_y)
    assert float(np.max(np.abs(ref.y))) < 0.01


def test_reachable_fixes_drops_an_isolated_spike_and_keeps_honest_motion():
    """Kept when a neighbour can be reached at a physical speed, and an
    endpoint is judged against the one neighbour it has."""
    from sim.plant_reference import reachable_fixes

    t = np.arange(0.0, 1.0, 0.1)
    x, y = 6.0 * t, np.zeros(len(t))
    assert reachable_fixes(t, x, y).all()

    x[5] = 889537.0
    keep = reachable_fixes(t, x, y)
    assert not keep[5]
    assert keep[np.arange(len(t)) != 5].all()

    x = 6.0 * t
    x[0] = -50_000.0
    assert not reachable_fixes(t, x, y)[0]


def test_fuse_survives_one_corrupt_fix_that_claims_rtk_accuracy():
    """The defect that cost two September stints.

    Three of the 33131 September fixes land hundreds of kilometres away and
    report the same 3 cm sigma every good RTK fix reports, so weighting by
    the reported covariance -- which is what makes the smoother trustworthy
    everywhere else -- gives one of them near-total authority. On
    run_20260913_203929 the forward filter ran clean for 275 ticks, took an
    889 km innovation on tick 276, and the stint came out at 82396 m/s.
    """
    m = _model()
    gt, gx, gy, gyro_t, gyro_z, wheel_t, wheel_v = _straight_streams()
    var = np.full(len(gt), 9e-4)         # 3 cm, the RTK-fixed figure
    course = np.zeros(len(gt))
    clean = fuse(gt, gx, gy, var, gt, course, gyro_t, gyro_z, wheel_t, wheel_v,
                 m, gps_var_y=var)

    gy = gy.copy()
    gy[len(gt) // 2] = 889537.265
    spiked = fuse(gt, gx, gy, var, gt, course, gyro_t, gyro_z, wheel_t,
                  wheel_v, m, gps_var_y=var)

    assert float(np.abs(spiked.v).max()) < 25.0
    assert float(np.abs(spiked.omega).max()) < 10.0
    np.testing.assert_allclose(spiked.y, clean.y, atol=0.01)
    np.testing.assert_allclose(spiked.v, clean.v, atol=0.01)
