"""Recursive least-squares residual learner for the MPC planner.

Phase 1 (this commit): tightened outlier filter, |d|-based training filter,
RLS warmup gate, effective-mode gating, counters, and a `predict()` signature
that returns (res_s, res_d, source). The GBM trainer + cache + checkpoint
ring land in Phase 2 — for now those slots stay `None`.
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Tuple

import numpy as np

# Feature-vector layout (kept in sync with mpc.py's _features helper).
NUM_STEER_HIST = 4
NUM_THROTTLE_HIST = 3
NUM_FEATURES = 1 + 4 + NUM_STEER_HIST + NUM_THROTTLE_HIST + 2  # bias, (d,v_s,v_d,k), hist, (nom_ds,nom_dd)

# Source ids returned by predict()
SOURCE_RLS = 0
SOURCE_GBM = 1


def _features(d: float, v_s: float, v_d: float, kappa: float,
              steer_hist: Tuple[float, ...],
              throttle_hist: Tuple[float, ...],
              nom_ds: float, nom_dd: float) -> np.ndarray:
    return np.array([
        1.0, d, v_s, v_d, kappa,
        *steer_hist, *throttle_hist,
        nom_ds, nom_dd,
    ], dtype=np.float64)


class ResidualLearner:
    def __init__(self, params: dict, solve_dt: float, s_total: float = 0.0):
        g = params.get
        self.mode = str(g("mode", "shadow")).lower()
        self.target_horizon_s = float(g("target_horizon_s", 0.5))
        self.horizon_steps = max(1, int(round(self.target_horizon_s / max(solve_dt, 1e-3))))
        self.lam = float(g("forgetting_factor", 0.99))
        self.min_speed = float(g("min_train_speed_mps", 0.5))
        self.err_window = int(g("error_window", 200))
        self.s_total = float(s_total)
        self.outlier_threshold = float(g("outlier_threshold_m", 2.0))
        self.max_train_d_m = float(g("max_train_d_m", 3.0))
        self.max_p_trace = float(g("max_p_trace", 1.0e5))
        self.max_theta_norm = float(g("max_theta_norm", 50.0))
        self.rls_warmup_samples = int(g("rls_warmup_samples", 50))
        self.apply_min_samples_this_run = int(g("apply_min_samples_this_run", 1200))
        self.gbm_enabled = bool(g("gbm_enabled", False))
        self.cache_enabled = bool(g("cache_enabled", True))
        self.cache_dir = str(g("cache_dir", "/root/.cache/residual_cache"))
        self.cache_load_on_start = bool(g("cache_load_on_start", True))
        self.cache_loaded = False
        self.cache = None

        self.p0 = float(g("initial_cov", 1000.0))
        self.theta_s = np.zeros(NUM_FEATURES)
        self.theta_d = np.zeros(NUM_FEATURES)
        self.P = np.eye(NUM_FEATURES) * self.p0

        self._pending: Deque[Tuple[np.ndarray, float, float, float, float]] = deque(
            maxlen=self.horizon_steps + 1
        )
        self._err_nom_s: Deque[float] = deque(maxlen=self.err_window)
        self._err_nom_d: Deque[float] = deque(maxlen=self.err_window)
        self._err_res_s: Deque[float] = deque(maxlen=self.err_window)
        self._err_res_d: Deque[float] = deque(maxlen=self.err_window)

        self.last_pred_s = 0.0
        self.last_pred_d = 0.0
        self.samples_trained = 0
        self.samples_accepted_this_run = 0
        self.outliers_dropped = 0
        self.off_line_skipped = 0
        self.divergence_resets = 0

        # Phase 2 slots (intentionally None in Phase 1; filled in later tasks).
        self.buffer = None
        self.trainer = None
        self.checkpoint_ring = None
        self.use_gbm = False
        self.last_active_model = SOURCE_RLS
        self.last_pred_clipped = 0
        self._gbm_s = None
        self._gbm_d = None
        self._last_val_mae_s = float("nan")
        self._last_val_mae_d = float("nan")
        self._last_rls_val_mae_s = float("nan")
        self._last_rls_val_mae_d = float("nan")
        self._select_eps_s = float(g("gbm_select_eps_s_m", 0.01))
        self._select_eps_d = float(g("gbm_select_eps_d_m", 0.005))
        self._predict_clip_m = float(g("gbm_predict_clip_m", 0.5))
        from autonomous_kart.nodes.pathfinder.planners.residual.checkpoint import CheckpointRing
        from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature
        self.checkpoint_ring = CheckpointRing(size=int(g("checkpoint_ring_size", 5)))
        self.regime_dv_tol = float(g("regime_dv_tol_mps", 1.0))
        self.regime_dd_tol = float(g("regime_dd_tol_m", 0.3))
        self.regime_dkappa_rel_tol = float(g("regime_dkappa_rel_tol", 0.5))
        self.revert_clip_rate_threshold = float(g("revert_clip_rate_threshold", 0.30))
        self.revert_count = 0
        self._current_regime = RegimeSignature(0.0, 0.0, 0.0)
        self.clip_rate_window_size = int(g("revert_clip_rate_window", 600))
        self._clip_history: Deque[int] = deque(maxlen=self.clip_rate_window_size)
        self._regime_window_size = 1200
        self._v_hist: Deque[float] = deque(maxlen=self._regime_window_size)
        self._d_hist: Deque[float] = deque(maxlen=self._regime_window_size)
        self._kappa_hist: Deque[float] = deque(maxlen=self._regime_window_size)

        if self.cache_enabled:
            from autonomous_kart.nodes.pathfinder.planners.residual.cache import ResidualCache
            self.cache = ResidualCache(self.cache_dir, feature_dim=NUM_FEATURES)
            if self.cache_load_on_start:
                loaded = self.cache.load()
                if loaded is not None:
                    self.theta_s = loaded.theta_s.copy()
                    self.theta_d = loaded.theta_d.copy()
                    self.P = loaded.P.copy()
                    self.samples_trained = loaded.samples_trained
                    self.cache_loaded = True
                    gs, gd = self.cache.load_gbm()
                    if gs is not None and gd is not None:
                        # Park them; use_gbm stays False until the next fresh
                        # GBM training cycle re-validates against this-run data.
                        self._gbm_s, self._gbm_d = gs, gd
                        self._last_val_mae_s = float("nan")
                        self._last_val_mae_d = float("nan")
                        self.use_gbm = False

        if self.gbm_enabled:
            from autonomous_kart.nodes.pathfinder.planners.residual.buffer import TrainBuffer
            from autonomous_kart.nodes.pathfinder.planners.residual.trainer import GBMTrainer
            self.buffer = TrainBuffer(
                capacity=int(g("gbm_buffer_capacity", 18000)),
                feature_dim=NUM_FEATURES,
            )
            self.trainer = GBMTrainer(
                buffer=self.buffer,
                rls_predict_batch=self._rls_predict_batch,
                max_iter=int(g("gbm_max_iter", 60)),
                max_depth=int(g("gbm_max_depth", 3)),
                learning_rate=float(g("gbm_learning_rate", 0.1)),
                min_samples_leaf=int(g("gbm_min_samples_leaf", 20)),
                min_samples_to_train=int(g("gbm_min_samples_to_train", 1200)),
                retrain_secs=float(g("gbm_retrain_secs", 10.0)),
                retrain_every_samples=int(g("gbm_retrain_every_samples", 1200)),
            )
            # Monkey-patch the trainer's train_once so the learner installs the
            # new GBM (subject to a hard val_mae safety floor) and recomputes
            # the selector flag after every successful retrain. This keeps the
            # trainer class generic — installation policy lives in the learner.
            original_train_once = self.trainer.train_once

            def _train_and_install():
                result = original_train_once()
                if result is None or result.gbm_s is None:
                    return result
                if result.val_mae_s > 2.0 * result.rls_val_mae_s:
                    return result
                self._install_gbm(
                    result.gbm_s, result.gbm_d,
                    val_mae_s=result.val_mae_s, val_mae_d=result.val_mae_d,
                    rls_val_mae_s=result.rls_val_mae_s, rls_val_mae_d=result.rls_val_mae_d,
                )
                # Record checkpoint
                if self.checkpoint_ring is not None:
                    from autonomous_kart.nodes.pathfinder.planners.residual.checkpoint import Checkpoint
                    self.checkpoint_ring.record(Checkpoint(
                        train_seq=result.train_seq, t_wall_ns=result.t_wall_ns,
                        theta_s=self.theta_s.copy(), theta_d=self.theta_d.copy(), P=self.P.copy(),
                        samples_trained=self.samples_trained,
                        gbm_s=result.gbm_s, gbm_d=result.gbm_d,
                        val_mae_s=result.val_mae_s, val_mae_d=result.val_mae_d,
                        rls_val_mae_s=result.rls_val_mae_s, rls_val_mae_d=result.rls_val_mae_d,
                        regime=self._current_regime,
                    ))
                # Persist to disk
                if self.cache is not None:
                    try:
                        self.cache.write_rls(
                            theta_s=self.theta_s, theta_d=self.theta_d, P=self.P,
                            samples_trained=self.samples_trained,
                            best_val_mae_s=result.val_mae_s, best_val_mae_d=result.val_mae_d,
                            train_seq=result.train_seq,
                        )
                        self.cache.write_gbm(result.gbm_s, result.gbm_d)
                    except Exception:
                        pass
                return result

            self.trainer.train_once = _train_and_install
            self.trainer.start()

    @property
    def enabled(self) -> bool:
        return self.mode in ("shadow", "apply")

    def effective_mode(self) -> str:
        """Return the *gated* mode: 'apply' is downgraded to 'shadow' until
        samples_accepted_this_run >= apply_min_samples_this_run."""
        if self.mode == "off":
            return "off"
        if self.mode == "apply" and self.samples_accepted_this_run < self.apply_min_samples_this_run:
            return "shadow"
        return self.mode

    def push(self, phi: np.ndarray, s_t: float, d_t: float,
             nom_ds: float, nom_dd: float, speed: float) -> None:
        if not self.enabled:
            return
        if speed < self.min_speed:
            return
        if abs(d_t) >= self.max_train_d_m:
            self.off_line_skipped += 1
            return
        self._pending.append((phi.copy(), s_t, d_t, nom_ds, nom_dd))

    def step(self, s_now: float, d_now: float) -> None:
        if not self.enabled or len(self._pending) <= self.horizon_steps:
            return
        phi, s0, d0, nom_ds, nom_dd = self._pending.popleft()
        actual_ds = s_now - s0
        actual_dd = d_now - d0
        if self.s_total > 0.0:
            half = 0.5 * self.s_total
            if actual_ds < -half:
                actual_ds += self.s_total
            elif actual_ds > half:
                actual_ds -= self.s_total
        r_s = actual_ds - nom_ds
        r_d = actual_dd - nom_dd

        if abs(r_s) > self.outlier_threshold or abs(r_d) > self.outlier_threshold:
            self.outliers_dropped += 1
            return

        pred_s = float(phi @ self.theta_s)
        pred_d = float(phi @ self.theta_d)
        self._err_nom_s.append(abs(actual_ds - nom_ds))
        self._err_nom_d.append(abs(actual_dd - nom_dd))
        self._err_res_s.append(abs(actual_ds - (nom_ds + pred_s)))
        self._err_res_d.append(abs(actual_dd - (nom_dd + pred_d)))

        Pphi = self.P @ phi
        denom = self.lam + float(phi @ Pphi)
        if denom < 1e-9:
            return
        K = Pphi / denom
        self.theta_s += K * (r_s - float(phi @ self.theta_s))
        self.theta_d += K * (r_d - float(phi @ self.theta_d))
        self.P = (self.P - np.outer(K, Pphi)) / self.lam
        self.samples_trained += 1
        self.samples_accepted_this_run += 1

        if self.buffer is not None:
            self.buffer.append(phi, r_s=float(r_s), r_d=float(r_d), t_ns=0)

        # phi[1] is d, phi[2] is v_s, phi[4] is kappa per the _features layout.
        self._v_hist.append(float(phi[2]))
        self._d_hist.append(float(phi[1]))
        self._kappa_hist.append(float(phi[4]))
        self._current_regime = self._compute_current_regime()

        tr = float(np.trace(self.P))
        if tr > self.max_p_trace:
            self.P *= self.max_p_trace / tr

        if (np.linalg.norm(self.theta_s) > self.max_theta_norm
                or np.linalg.norm(self.theta_d) > self.max_theta_norm):
            self._trigger_divergence_check()

    def predict(self, phi: np.ndarray) -> Tuple[float, float, int]:
        if not self.enabled:
            self.last_pred_s = 0.0
            self.last_pred_d = 0.0
            self.last_active_model = SOURCE_RLS
            self.last_pred_clipped = 0
            return 0.0, 0.0, SOURCE_RLS
        if self.samples_trained < self.rls_warmup_samples and not self.use_gbm:
            self.last_pred_s = 0.0
            self.last_pred_d = 0.0
            self.last_active_model = SOURCE_RLS
            self.last_pred_clipped = 0
            self._clip_history.append(0)
            return 0.0, 0.0, SOURCE_RLS
        if self.use_gbm and self._gbm_s is not None and self._gbm_d is not None:
            X = phi.reshape(1, -1)
            s_raw = float(self._gbm_s.predict(X)[0])
            d_raw = float(self._gbm_d.predict(X)[0])
            clip = self._predict_clip_m
            s = max(-clip, min(clip, s_raw))
            d = max(-clip, min(clip, d_raw))
            clipped = 1 if (s != s_raw or d != d_raw) else 0
            self.last_pred_s = s
            self.last_pred_d = d
            self.last_active_model = SOURCE_GBM
            self.last_pred_clipped = clipped
            self._clip_history.append(clipped)
            self._maybe_clip_revert()
            return s, d, SOURCE_GBM
        # Offline replay observed RLS spikes to ~200 m of Δs prediction in benign driving
        # Clip is hard safety
        s_raw = float(phi @ self.theta_s)
        d_raw = float(phi @ self.theta_d)
        clip = self._predict_clip_m
        s = max(-clip, min(clip, s_raw))
        d = max(-clip, min(clip, d_raw))
        clipped = 1 if (s != s_raw or d != d_raw) else 0
        self.last_pred_s = s
        self.last_pred_d = d
        self.last_active_model = SOURCE_RLS
        self.last_pred_clipped = clipped
        self._clip_history.append(clipped)
        return s, d, SOURCE_RLS

    def mean_error(self) -> Tuple[float, float, float, float]:
        def _mean(d): return float(np.mean(d)) if d else 0.0

        return (_mean(self._err_nom_s), _mean(self._err_nom_d),
                _mean(self._err_res_s), _mean(self._err_res_d))

    def clip_rate(self) -> float:
        if not self._clip_history:
            return 0.0
        return sum(self._clip_history) / len(self._clip_history)

    def save_rls_cache(self, *, best_val_mae_s: float, best_val_mae_d: float, train_seq: int) -> None:
        if self.cache is None:
            return
        self.cache.write_rls(
            theta_s=self.theta_s, theta_d=self.theta_d, P=self.P,
            samples_trained=self.samples_trained,
            best_val_mae_s=best_val_mae_s,
            best_val_mae_d=best_val_mae_d,
            train_seq=train_seq,
        )

    def _seed_checkpoint(self, *, theta_s, theta_d, P, samples_trained,
                          val_mae_s, val_mae_d, regime) -> None:
        """Test/cache-load helper that pushes a Checkpoint into the ring."""
        from autonomous_kart.nodes.pathfinder.planners.residual.checkpoint import Checkpoint
        self.checkpoint_ring.record(Checkpoint(
            train_seq=0, t_wall_ns=0,
            theta_s=theta_s.copy(), theta_d=theta_d.copy(), P=P.copy(),
            samples_trained=int(samples_trained),
            gbm_s=None, gbm_d=None,
            val_mae_s=val_mae_s, val_mae_d=val_mae_d,
            rls_val_mae_s=val_mae_s * 1.5, rls_val_mae_d=val_mae_d * 1.5,
            regime=regime,
        ))

    def _trigger_divergence_check(self) -> None:
        from autonomous_kart.nodes.pathfinder.planners.residual.regime import similar
        cp = self.checkpoint_ring.best_recent() if self.checkpoint_ring else None
        if cp is not None and similar(
            self._current_regime, cp.regime,
            dv_tol=self.regime_dv_tol, dd_tol=self.regime_dd_tol,
            dkappa_rel_tol=self.regime_dkappa_rel_tol,
        ):
            self.theta_s = cp.theta_s.copy()
            self.theta_d = cp.theta_d.copy()
            self.P = cp.P.copy()
        else:
            self.theta_s.fill(0.0)
            self.theta_d.fill(0.0)
            self.P = np.eye(NUM_FEATURES) * self.p0
            self.samples_trained = 0
        self.divergence_resets += 1
        self.revert_count += 1

    def _maybe_clip_revert(self) -> None:
        if not self.use_gbm:
            return
        rate = self.clip_rate()
        if rate >= self.revert_clip_rate_threshold and len(self._clip_history) == self._clip_history.maxlen:
            self.use_gbm = False
            self.revert_count += 1

    def manual_revert(self) -> None:
        if self.checkpoint_ring is None or self.checkpoint_ring.is_empty():
            return
        cp = self.checkpoint_ring.best_recent()
        self.theta_s = cp.theta_s.copy()
        self.theta_d = cp.theta_d.copy()
        self.P = cp.P.copy()
        if cp.gbm_s is not None and cp.gbm_d is not None:
            self._install_gbm(cp.gbm_s, cp.gbm_d,
                              val_mae_s=cp.val_mae_s, val_mae_d=cp.val_mae_d,
                              rls_val_mae_s=cp.rls_val_mae_s, rls_val_mae_d=cp.rls_val_mae_d)
        self.revert_count += 1

    def _install_gbm(self, gbm_s, gbm_d, *, val_mae_s: float, val_mae_d: float,
                     rls_val_mae_s: float, rls_val_mae_d: float) -> None:
        """Install a pair of GBM regressors and recompute the selector flag.

        Real callers are GBMTrainer (after training) and the cache loader
        (Task 17). Tests call it directly to skip the threaded trainer.
        """
        self._gbm_s = gbm_s
        self._gbm_d = gbm_d
        self._last_val_mae_s = val_mae_s
        self._last_val_mae_d = val_mae_d
        self._last_rls_val_mae_s = rls_val_mae_s
        self._last_rls_val_mae_d = rls_val_mae_d
        self._recompute_use_gbm()

    def _recompute_use_gbm(self) -> None:
        """Set self.use_gbm with hysteresis. Bad-GBM hard safety: never use
        when val_mae_s > 2 × rls_val_mae_s."""
        import math
        if self._gbm_s is None or not math.isfinite(self._last_val_mae_s):
            self.use_gbm = False
            return
        if self._last_val_mae_s > 2.0 * self._last_rls_val_mae_s:
            self.use_gbm = False
            return
        beats_on_s = self._last_val_mae_s + self._select_eps_s < self._last_rls_val_mae_s
        beats_on_d = self._last_val_mae_d + self._select_eps_d < self._last_rls_val_mae_d
        if self.use_gbm:
            # Stay on GBM if GBM still beats RLS on s OR d (hysteresis).
            self.use_gbm = beats_on_s or beats_on_d
        else:
            # Flip to GBM if GBM beats RLS on s OR d.
            self.use_gbm = beats_on_s or beats_on_d

    def _compute_current_regime(self):
        from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature
        v = np.fromiter(self._v_hist, dtype=np.float64)
        d = np.fromiter(self._d_hist, dtype=np.float64)
        k = np.fromiter(self._kappa_hist, dtype=np.float64)
        # The residual sees v_s (longitudinal speed in Frenet) only — use its
        # magnitude as a v proxy.
        return RegimeSignature.from_arrays(np.abs(v), d, k)

    def _rls_predict_batch(self, X: np.ndarray):
        """Batch RLS predictor for the trainer's held-out scoring."""
        return X @ self.theta_s, X @ self.theta_d

    def shutdown(self) -> None:
        if self.trainer is not None:
            self.trainer.stop(timeout=2.0)
