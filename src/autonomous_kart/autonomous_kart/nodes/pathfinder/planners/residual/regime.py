"""RegimeSignature: tiny fingerprint of recent driving conditions.

Used to gate revert decisions — a checkpoint trained in one regime is only restored if the current regime is similar"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RegimeSignature:
    mean_v: float
    mean_abs_d: float
    std_kappa: float

    @classmethod
    def from_arrays(cls, v: np.ndarray, d: np.ndarray, kappa: np.ndarray) -> "RegimeSignature":
        if v.size == 0:
            return cls(0.0, 0.0, 0.0)
        return cls(
            mean_v=float(np.mean(v)),
            mean_abs_d=float(np.mean(np.abs(d))),
            std_kappa=float(np.std(kappa)),
        )


def similar(a: RegimeSignature, b: RegimeSignature,
            *, dv_tol: float, dd_tol: float, dkappa_rel_tol: float) -> bool:
    """Return True iff a and b are within all three tolerances."""
    if abs(a.mean_v - b.mean_v) > dv_tol:
        return False
    if abs(a.mean_abs_d - b.mean_abs_d) > dd_tol:
        return False
    denom = max(a.std_kappa, b.std_kappa, 1e-3)
    if abs(a.std_kappa - b.std_kappa) / denom > dkappa_rel_tol:
        return False
    return True
