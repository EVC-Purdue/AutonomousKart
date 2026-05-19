"""Fixed-capacity ring buffer of (phi, r_s, r_d, t_ns) training samples.

Writes happen on the 60 Hz hot loop"""
from __future__ import annotations

import threading

import numpy as np


class TrainBuffer:
    def __init__(self, capacity: int, feature_dim: int):
        self.capacity = int(capacity)
        self.feature_dim = int(feature_dim)
        self._X = np.zeros((self.capacity, self.feature_dim), dtype=np.float64)
        self._ys = np.zeros(self.capacity, dtype=np.float64)
        self._yd = np.zeros(self.capacity, dtype=np.float64)
        self._t = np.zeros(self.capacity, dtype=np.int64)
        self._write_idx = 0       # next index to write
        self._count = 0           # min(filled, capacity)
        self._lock = threading.Lock()

    @property
    def size(self) -> int:
        with self._lock:
            return self._count

    def append(self, phi: np.ndarray, r_s: float, r_d: float, t_ns: int) -> None:
        with self._lock:
            i = self._write_idx
            self._X[i, :] = phi
            self._ys[i] = r_s
            self._yd[i] = r_d
            self._t[i] = t_ns
            self._write_idx = (i + 1) % self.capacity
            if self._count < self.capacity:
                self._count += 1

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (X, ys, yd, t_ns) copies in chronological order."""
        with self._lock:
            n = self._count
            if n == 0:
                return (
                    np.zeros((0, self.feature_dim)),
                    np.zeros(0), np.zeros(0), np.zeros(0, dtype=np.int64),
                )
            if n < self.capacity:
                # Not yet wrapped: rows 0..n-1 in order
                X = self._X[:n].copy()
                ys = self._ys[:n].copy()
                yd = self._yd[:n].copy()
                t = self._t[:n].copy()
            else:
                # Wrapped: oldest starts at write_idx, then wraps
                wi = self._write_idx
                X = np.concatenate([self._X[wi:], self._X[:wi]], axis=0).copy()
                ys = np.concatenate([self._ys[wi:], self._ys[:wi]]).copy()
                yd = np.concatenate([self._yd[wi:], self._yd[:wi]]).copy()
                t = np.concatenate([self._t[wi:], self._t[:wi]]).copy()
            return X, ys, yd, t
