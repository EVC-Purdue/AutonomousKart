"""Unit tests for the localization EKF."""
import math

import numpy as np
import pytest

from autonomous_kart.nodes.localization.ekf import LocalizationEKF


def _mk(**kw):
    defaults = dict(pos_noise=0.01)
    defaults.update(kw)
    return LocalizationEKF(**defaults)


# Default IMU per-sample variances used by predict tests.
_OMEGA_VAR = 0.001   # rad^2/s^2
_ACCEL_VAR = 0.01    # (m/s^2)^2


def test_starts_uninitialized_with_huge_covariance():
    f = _mk()
    assert f.initialized is False
    assert np.all(f.x == 0.0)
    assert np.all(np.diag(f.P) > 1e5)


def test_reset_sets_state_and_marks_initialized():
    f = _mk()
    f.reset(px=10.0, py=-5.0, yaw=math.pi / 4, v=3.0)
    assert f.x[0] == 10.0
    assert f.x[1] == -5.0
    assert f.x[2] == pytest.approx(math.pi / 4)
    assert f.x[3] == 3.0
    assert f.initialized is True


def test_reset_with_explicit_covariance():
    f = _mk()
    P0 = np.diag([0.04, 0.04, 0.01, 1.0])
    f.reset(0.0, 0.0, 0.0, 0.0, P=P0)
    np.testing.assert_allclose(f.P, P0)


def test_predict_with_zero_speed_is_identity():
    f = _mk()
    f.reset(1.0, 2.0, 0.5, 0.0)
    f.predict(dt=0.0167, omega_z=0.2, accel_x=0.0,
              omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    assert f.x[0] == pytest.approx(1.0)
    assert f.x[1] == pytest.approx(2.0)
    # yaw can drift from omega_z even at v=0 that's correct behaviour now.
    assert f.x[3] == pytest.approx(0.0)


def test_predict_constant_velocity_straight_line():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 5.0)
    dt = 0.02
    for _ in range(50):  # 1 s
        f.predict(dt, omega_z=0.0, accel_x=0.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    assert f.x[0] == pytest.approx(5.0, abs=1e-6)
    assert f.x[1] == pytest.approx(0.0, abs=1e-6)
    assert f.x[2] == pytest.approx(0.0, abs=1e-9)
    assert f.x[3] == pytest.approx(5.0)


def test_predict_constant_yaw_rate_traces_circle():
    # With constant omega_z and constant v, radius r = v / omega_z.
    v = 3.0
    omega = math.radians(15.0)
    r = v / omega
    f = _mk()
    f.reset(0.0, 0.0, 0.0, v)
    dt = 0.005
    for _ in range(2000):  # 10 s
        f.predict(dt, omega_z=omega, accel_x=0.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    # Left turn from origin pointing along +x circles center (0, r).
    cx, cy = 0.0, r
    dist = math.hypot(f.x[0] - cx, f.x[1] - cy)
    assert dist == pytest.approx(r, rel=5e-3)


def test_predict_wraps_yaw_into_minus_pi_pi():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 10.0)
    for _ in range(2000):
        f.predict(0.02, omega_z=math.radians(20.0), accel_x=0.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
        assert -math.pi - 1e-9 <= f.x[2] <= math.pi + 1e-9


def test_predict_grows_covariance_when_uncorrected():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 5.0)
    P0 = f.P.copy()
    for _ in range(60):
        f.predict(1 / 60.0, omega_z=0.05, accel_x=0.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    for i in range(4):
        assert f.P[i, i] > P0[i, i]


def test_predict_covariance_is_symmetric():
    f = _mk()
    f.reset(0.0, 0.0, 0.3, 4.0)
    for _ in range(100):
        f.predict(0.02, omega_z=0.1, accel_x=0.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    np.testing.assert_allclose(f.P, f.P.T, atol=1e-12)


def test_predict_accel_var_scales_v_variance():
    f_small = _mk()
    f_small.reset(0.0, 0.0, 0.0, 0.0)
    f_big = _mk()
    f_big.reset(0.0, 0.0, 0.0, 0.0)
    for _ in range(60):
        f_small.predict(1 / 60.0, omega_z=0.0, accel_x=0.0,
                        omega_var=_OMEGA_VAR, accel_var=0.1)
        f_big.predict(1 / 60.0, omega_z=0.0, accel_x=0.0,
                      omega_var=_OMEGA_VAR, accel_var=10.0)
    assert f_big.P[3, 3] > f_small.P[3, 3]


def test_predict_accel_drives_v_evolution():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    dt = 0.01
    for _ in range(100):  # 1 s of a_x = 1.0
        f.predict(dt, omega_z=0.0, accel_x=1.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    assert f.x[3] == pytest.approx(1.0, abs=1e-6)


def test_predict_gyro_drives_yaw_evolution():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    dt = 0.01
    for _ in range(100):  # 1 s of omega_z = 0.5
        f.predict(dt, omega_z=0.5, accel_x=0.0,
                  omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    assert f.x[2] == pytest.approx(0.5, abs=1e-6)


def test_predict_yaw_no_longer_depends_on_v():
    f_slow = _mk()
    f_slow.reset(0.0, 0.0, 0.0, 1.0)
    f_fast = _mk()
    f_fast.reset(0.0, 0.0, 0.0, 10.0)
    dt = 0.01
    for _ in range(100):
        f_slow.predict(dt, omega_z=0.3, accel_x=0.0,
                       omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
        f_fast.predict(dt, omega_z=0.3, accel_x=0.0,
                       omega_var=_OMEGA_VAR, accel_var=_ACCEL_VAR)
    assert f_slow.x[2] == pytest.approx(f_fast.x[2], abs=1e-9)


def test_update_gps_xy_pulls_state_toward_measurement():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    R = np.diag([0.04, 0.04])  # 20 cm 1sigma
    f.update_gps_xy(x=2.0, y=1.0, R_xy=R)
    # State should move *toward* (2,1) but not all the way (P0 has 25cm 1sigma).
    assert 0.0 < f.x[0] < 2.0
    assert 0.0 < f.x[1] < 1.0


def test_update_gps_xy_shrinks_position_covariance():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    P_xy_before = f.P[0, 0]
    f.update_gps_xy(x=0.0, y=0.0, R_xy=np.diag([0.01, 0.01]))
    assert f.P[0, 0] < P_xy_before
    assert f.P[1, 1] < P_xy_before


def test_update_gps_xy_with_huge_R_is_almost_a_noop():
    f = _mk()
    f.reset(1.0, 2.0, 0.3, 4.0)
    x_before = f.x.copy()
    f.update_gps_xy(x=999.0, y=999.0, R_xy=np.diag([1e8, 1e8]))
    np.testing.assert_allclose(f.x, x_before, atol=1e-2)


def test_update_gps_xy_keeps_covariance_symmetric():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    f.update_gps_xy(x=1.0, y=-1.0, R_xy=np.diag([0.1, 0.1]))
    np.testing.assert_allclose(f.P, f.P.T, atol=1e-12)


def test_update_heading_pulls_yaw_toward_measurement():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 5.0)
    f.update_heading(yaw_meas=math.radians(30.0), var=math.radians(5.0) ** 2)
    assert 0.0 < f.x[2] < math.radians(30.0)


def test_update_heading_handles_wrap_around_pi():
    f = _mk()
    f.reset(0.0, 0.0, math.pi - 0.05, 5.0)
    # Measurement is just past -pi innovation should be small, not ~2pi.
    f.update_heading(yaw_meas=-math.pi + 0.05, var=0.01)
    # Yaw moves toward +/-pi, NOT toward 0.
    assert abs(f.x[2]) > math.pi - 0.2


def test_update_heading_yaw_stays_in_minus_pi_pi():
    f = _mk()
    f.reset(0.0, 0.0, 3.0, 5.0)
    f.update_heading(yaw_meas=-3.0, var=0.01)
    assert -math.pi - 1e-9 <= f.x[2] <= math.pi + 1e-9


def test_update_heading_shrinks_yaw_covariance():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 5.0)
    P_yaw_before = f.P[2, 2]
    f.update_heading(yaw_meas=0.1, var=0.001)
    assert f.P[2, 2] < P_yaw_before


def test_update_speed_pulls_v_toward_measurement():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    f.update_speed(v_meas=4.0, var=0.05)
    assert 0.0 < f.x[3] < 4.0


def test_update_speed_shrinks_v_covariance():
    f = _mk()
    f.reset(0.0, 0.0, 0.0, 0.0)
    Pv_before = f.P[3, 3]
    f.update_speed(v_meas=2.0, var=0.01)
    assert f.P[3, 3] < Pv_before


def test_update_speed_does_not_move_position_directly():
    # H_v only touches v; position should be unchanged for a diagonal P0.
    f = _mk()
    f.reset(1.0, 2.0, 0.0, 0.0, P=np.diag([0.04, 0.04, 0.04, 1.0]))
    f.update_speed(v_meas=10.0, var=0.01)
    assert f.x[0] == pytest.approx(1.0, abs=1e-9)
    assert f.x[1] == pytest.approx(2.0, abs=1e-9)
