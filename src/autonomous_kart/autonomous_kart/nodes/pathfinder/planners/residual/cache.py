"""Disk persistence for the residual learner.

Layout under cache_dir:
    manifest.json       schema_version, feature_dim, train_seq, best_val_mae_*
    rls.npz             theta_s, theta_d, P, samples_trained
    gbm_s.joblib        sklearn HistGBR for s residual
    gbm_d.joblib        sklearn HistGBR for d residual
    ring/               last N checkpoints
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import joblib

import numpy as np

SCHEMA_VERSION = 1


@dataclass
class LoadedCache:
    feature_dim: int
    theta_s: np.ndarray
    theta_d: np.ndarray
    P: np.ndarray
    samples_trained: int
    train_seq: int
    best_val_mae_s: float
    best_val_mae_d: float


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def _atomic_write_npz(path: Path, **arrays) -> None:
    # np.savez appends .npz automatically, so give it a stem without .npz
    tmp = path.parent / (path.stem + ".tmp")
    np.savez(tmp, **arrays)
    os.replace(str(tmp) + ".npz", path)


def _atomic_write_joblib(path: Path, obj) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    joblib.dump(obj, tmp)
    os.replace(tmp, path)


class ResidualCache:
    def __init__(self, cache_dir: str, feature_dim: int):
        self.dir = Path(cache_dir)
        self.feature_dim = int(feature_dim)
        self.dir.mkdir(parents=True, exist_ok=True)

    @property
    def manifest_path(self) -> Path:
        return self.dir / "manifest.json"

    @property
    def rls_path(self) -> Path:
        return self.dir / "rls.npz"

    def write_rls(self, theta_s: np.ndarray, theta_d: np.ndarray, P: np.ndarray,
                  samples_trained: int, best_val_mae_s: float, best_val_mae_d: float,
                  train_seq: int) -> None:
        _atomic_write_npz(
            self.rls_path,
            theta_s=theta_s.astype(np.float64),
            theta_d=theta_d.astype(np.float64),
            P=P.astype(np.float64),
            samples_trained=np.int64(samples_trained),
        )
        manifest = dict(
            schema_version=SCHEMA_VERSION,
            feature_dim=self.feature_dim,
            train_seq=int(train_seq),
            best_val_mae_s=float(best_val_mae_s),
            best_val_mae_d=float(best_val_mae_d),
        )
        _atomic_write_bytes(self.manifest_path, json.dumps(manifest, indent=2).encode())

    @property
    def gbm_s_path(self) -> Path:
        return self.dir / "gbm_s.joblib"

    @property
    def gbm_d_path(self) -> Path:
        return self.dir / "gbm_d.joblib"

    def write_gbm(self, gbm_s, gbm_d) -> None:
        _atomic_write_joblib(self.gbm_s_path, gbm_s)
        _atomic_write_joblib(self.gbm_d_path, gbm_d)

    def load_gbm(self):
        if not (self.gbm_s_path.exists() and self.gbm_d_path.exists()):
            return None, None
        try:
            return joblib.load(self.gbm_s_path), joblib.load(self.gbm_d_path)
        except Exception:
            return None, None

    def load(self) -> Optional[LoadedCache]:
        if not self.manifest_path.exists():
            return None
        try:
            m = json.loads(self.manifest_path.read_text())
        except (OSError, json.JSONDecodeError):
            return None
        if m.get("schema_version") != SCHEMA_VERSION:
            return None
        if m.get("feature_dim") != self.feature_dim:
            return None
        if not self.rls_path.exists():
            return None
        try:
            arr = np.load(self.rls_path)
            return LoadedCache(
                feature_dim=self.feature_dim,
                theta_s=arr["theta_s"],
                theta_d=arr["theta_d"],
                P=arr["P"],
                samples_trained=int(arr["samples_trained"]),
                train_seq=int(m["train_seq"]),
                best_val_mae_s=float(m["best_val_mae_s"]),
                best_val_mae_d=float(m["best_val_mae_d"]),
            )
        except Exception:
            return None
