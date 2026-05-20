"""Tests for RegimeSignature similarity."""
import numpy as np

from autonomous_kart.nodes.pathfinder.planners.residual.regime import (
    RegimeSignature, similar,
)


def test_signature_from_arrays():
    v = np.array([3.0, 4.0, 5.0])
    d = np.array([0.1, -0.2, 0.0])
    kappa = np.array([0.01, 0.0, -0.01])
    sig = RegimeSignature.from_arrays(v, d, kappa)
    assert abs(sig.mean_v - 4.0) < 1e-9
    assert abs(sig.mean_abs_d - (0.1 + 0.2 + 0.0) / 3) < 1e-9
    assert abs(sig.std_kappa - np.std(kappa)) < 1e-9


def test_similar_within_tolerances():
    a = RegimeSignature(mean_v=4.0, mean_abs_d=0.5, std_kappa=0.02)
    b = RegimeSignature(mean_v=4.5, mean_abs_d=0.6, std_kappa=0.02)
    assert similar(a, b, dv_tol=1.0, dd_tol=0.3, dkappa_rel_tol=0.5)


def test_dissimilar_when_speed_too_far():
    a = RegimeSignature(mean_v=4.0, mean_abs_d=0.5, std_kappa=0.02)
    b = RegimeSignature(mean_v=6.0, mean_abs_d=0.5, std_kappa=0.02)
    assert not similar(a, b, dv_tol=1.0, dd_tol=0.3, dkappa_rel_tol=0.5)


def test_dissimilar_when_d_too_far():
    a = RegimeSignature(mean_v=4.0, mean_abs_d=0.2, std_kappa=0.02)
    b = RegimeSignature(mean_v=4.0, mean_abs_d=0.8, std_kappa=0.02)
    assert not similar(a, b, dv_tol=1.0, dd_tol=0.3, dkappa_rel_tol=0.5)


def test_dissimilar_when_kappa_too_far():
    a = RegimeSignature(mean_v=4.0, mean_abs_d=0.2, std_kappa=0.01)
    b = RegimeSignature(mean_v=4.0, mean_abs_d=0.2, std_kappa=0.05)
    # |0.05 - 0.01| / max(0.05, 0.01, 1e-3) = 0.8 > 0.5
    assert not similar(a, b, dv_tol=1.0, dd_tol=0.3, dkappa_rel_tol=0.5)


def test_from_empty_arrays_safe():
    sig = RegimeSignature.from_arrays(np.array([]), np.array([]), np.array([]))
    assert sig.mean_v == 0.0
    assert sig.mean_abs_d == 0.0
    assert sig.std_kappa == 0.0
