"""Checkpoint of a known-good residual model state + CheckpointRing.

A checkpoint snapshots both models (RLS state arrays + the two GBM
regressors) plus their validation MAEs and the regime signature at save
time. The ring keeps the last N checkpoints and protects best from eviction"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature


@dataclass
class Checkpoint:
    train_seq: int
    t_wall_ns: int
    # RLS state
    theta_s: np.ndarray
    theta_d: np.ndarray
    P: np.ndarray
    samples_trained: int
    # GBM state (Optional — Any here so tests don't need sklearn for the dataclass)
    gbm_s: Optional[Any]
    gbm_d: Optional[Any]
    val_mae_s: float
    val_mae_d: float
    rls_val_mae_s: float
    rls_val_mae_d: float
    regime: RegimeSignature


class CheckpointRing:
    def __init__(self, size: int = 5):
        self.size = max(1, int(size))
        self.entries: list[Checkpoint] = []

    def is_empty(self) -> bool:
        return not self.entries

    def record(self, cp: Checkpoint) -> None:
        if len(self.entries) < self.size:
            self.entries.append(cp)
            return
        # Need to evict. Find the all-time-best entry by val_mae_s
        best_idx = self._best_index()
        # Oldest is index 0. Don't evict the best (unless it's the only slot).
        evict_idx = 0
        if best_idx == 0 and self.size > 1:
            evict_idx = 1
        del self.entries[evict_idx]
        self.entries.append(cp)

    def best_recent(self) -> Optional[Checkpoint]:
        idx = self._best_index()
        if idx is None:
            return None
        return self.entries[idx]

    def _best_index(self) -> Optional[int]:
        best_i = None
        best_v = math.inf
        for i, c in enumerate(self.entries):
            v = c.val_mae_s
            if not math.isfinite(v):
                continue
            if v < best_v:
                best_v = v
                best_i = i
        return best_i
