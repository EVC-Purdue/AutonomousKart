"""Background GBM trainer.

Runs in a daemon thread. Trains two sklearn HistGradientBoostingRegressors
from a chronological 80/20 split of the TrainBuffer snapshot. The same
held-out 20% is also scored against the RLS to compare"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np


@dataclass
class TrainResult:
    train_seq: int
    t_wall_ns: int
    n_samples: int
    train_wall_ms: float
    gbm_s: Any
    gbm_d: Any
    train_mae_s: float
    train_mae_d: float
    val_mae_s: float
    val_mae_d: float
    rls_val_mae_s: float
    rls_val_mae_d: float
    error: Optional[str] = None


class GBMTrainer:
    def __init__(self, *, buffer, rls_predict_batch: Callable,
                 max_iter: int, max_depth: int, learning_rate: float,
                 min_samples_leaf: int, min_samples_to_train: int,
                 retrain_secs: float, retrain_every_samples: int):
        self.buffer = buffer
        self.rls_predict_batch = rls_predict_batch
        self.max_iter = int(max_iter)
        self.max_depth = int(max_depth)
        self.learning_rate = float(learning_rate)
        self.min_samples_leaf = int(min_samples_leaf)
        self.min_samples_to_train = int(min_samples_to_train)
        self.retrain_secs = float(retrain_secs)
        self.retrain_every_samples = int(retrain_every_samples)

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_buffer_size_at_train = 0
        self._last_train_t = 0.0

        # Public state (read under self._lock from outside).
        self.gbm_s = None
        self.gbm_d = None
        self.train_seq = 0
        self.last_result: Optional[TrainResult] = None
        self.last_error: Optional[str] = None

    def _snapshot_and_filter(self):
        X, ys, yd, _ = self.buffer.snapshot()
        if len(X) < self.min_samples_to_train:
            return None
        finite = np.isfinite(X).all(axis=1) & np.isfinite(ys) & np.isfinite(yd)
        X = X[finite]; ys = ys[finite]; yd = yd[finite]
        if len(X) < self.min_samples_to_train:
            return None
        return X, ys, yd

    def train_once(self) -> Optional[TrainResult]:
        from sklearn.ensemble import HistGradientBoostingRegressor

        snap = self._snapshot_and_filter()
        if snap is None:
            return None
        X, ys, yd = snap
        n = len(X)
        split = int(0.8 * n)
        Xtr, Xte = X[:split], X[split:]
        ystr, yste = ys[:split], ys[split:]
        ydtr, ydte = yd[:split], yd[split:]

        t0 = time.perf_counter()
        try:
            kw = dict(max_iter=self.max_iter, max_depth=self.max_depth,
                      learning_rate=self.learning_rate,
                      min_samples_leaf=self.min_samples_leaf)
            gbm_s = HistGradientBoostingRegressor(**kw).fit(Xtr, ystr)
            gbm_d = HistGradientBoostingRegressor(**kw).fit(Xtr, ydtr)
        except Exception as exc:
            err = repr(exc)
            with self._lock:
                self.last_error = err
            return TrainResult(
                train_seq=self.train_seq, t_wall_ns=time.time_ns(),
                n_samples=n, train_wall_ms=(time.perf_counter() - t0) * 1000.0,
                gbm_s=None, gbm_d=None,
                train_mae_s=float("nan"), train_mae_d=float("nan"),
                val_mae_s=float("nan"), val_mae_d=float("nan"),
                rls_val_mae_s=float("nan"), rls_val_mae_d=float("nan"),
                error=err,
            )
        train_wall_ms = (time.perf_counter() - t0) * 1000.0

        pred_s_tr = gbm_s.predict(Xtr); pred_d_tr = gbm_d.predict(Xtr)
        pred_s_te = gbm_s.predict(Xte); pred_d_te = gbm_d.predict(Xte)
        rls_s_te, rls_d_te = self.rls_predict_batch(Xte)

        result = TrainResult(
            train_seq=self.train_seq + 1, t_wall_ns=time.time_ns(),
            n_samples=n, train_wall_ms=train_wall_ms,
            gbm_s=gbm_s, gbm_d=gbm_d,
            train_mae_s=float(np.mean(np.abs(ystr - pred_s_tr))),
            train_mae_d=float(np.mean(np.abs(ydtr - pred_d_tr))),
            val_mae_s=float(np.mean(np.abs(yste - pred_s_te))),
            val_mae_d=float(np.mean(np.abs(ydte - pred_d_te))),
            rls_val_mae_s=float(np.mean(np.abs(yste - rls_s_te))),
            rls_val_mae_d=float(np.mean(np.abs(ydte - rls_d_te))),
        )

        with self._lock:
            self.gbm_s = gbm_s
            self.gbm_d = gbm_d
            self.train_seq = result.train_seq
            self.last_result = result
            self.last_error = None
        return result

    def _loop(self):
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=min(self.retrain_secs, 1.0))
            if self._stop_event.is_set():
                return
            now = time.monotonic()
            buf_size = self.buffer.size
            time_due = (now - self._last_train_t) >= self.retrain_secs
            sample_due = self.retrain_every_samples > 0 and (
                buf_size - self._last_buffer_size_at_train) >= self.retrain_every_samples
            if not (time_due or sample_due):
                continue
            result = self.train_once()
            if result is not None:
                self._last_train_t = now
                self._last_buffer_size_at_train = buf_size

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, name="gbm-trainer", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def snapshot_models(self):
        with self._lock:
            return self.gbm_s, self.gbm_d, self.last_result
