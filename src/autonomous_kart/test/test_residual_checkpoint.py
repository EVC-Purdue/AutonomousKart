"""Tests for Checkpoint dataclass + CheckpointRing eviction policy."""
import numpy as np

from autonomous_kart.nodes.pathfinder.planners.residual.checkpoint import (
    Checkpoint, CheckpointRing,
)
from autonomous_kart.nodes.pathfinder.planners.residual.regime import RegimeSignature


def _ckpt(train_seq, val_mae_s, val_mae_d=0.05):
    return Checkpoint(
        train_seq=train_seq, t_wall_ns=train_seq * 1_000_000_000,
        theta_s=np.zeros(14), theta_d=np.zeros(14), P=np.eye(14),
        samples_trained=train_seq * 100,
        gbm_s=None, gbm_d=None,
        val_mae_s=val_mae_s, val_mae_d=val_mae_d,
        rls_val_mae_s=val_mae_s * 1.1, rls_val_mae_d=val_mae_d * 1.1,
        regime=RegimeSignature(4.0, 0.5, 0.02),
    )


def test_ring_empty_returns_none():
    r = CheckpointRing(size=3)
    assert r.best_recent() is None
    assert r.is_empty()


def test_ring_records_in_order_until_full():
    r = CheckpointRing(size=3)
    r.record(_ckpt(1, 0.20))
    r.record(_ckpt(2, 0.15))
    r.record(_ckpt(3, 0.10))
    assert not r.is_empty()
    assert r.best_recent().train_seq == 3   # lowest val_mae_s wins


def test_ring_evicts_oldest_when_full():
    r = CheckpointRing(size=3)
    r.record(_ckpt(1, 0.20))
    r.record(_ckpt(2, 0.15))
    r.record(_ckpt(3, 0.10))
    # 4th insert: ckpt #1 (oldest) is NOT the best, so it gets evicted
    r.record(_ckpt(4, 0.18))
    seqs = [c.train_seq for c in r.entries]
    assert 1 not in seqs
    assert sorted(seqs) == [2, 3, 4]


def test_ring_protects_alltime_best_from_eviction():
    r = CheckpointRing(size=3)
    r.record(_ckpt(1, 0.05))   # all-time best
    r.record(_ckpt(2, 0.20))
    r.record(_ckpt(3, 0.30))
    # Inserting #4 would evict oldest=#1, but #1 is best — evict next-oldest instead
    r.record(_ckpt(4, 0.25))
    seqs = sorted(c.train_seq for c in r.entries)
    assert 1 in seqs
    assert 2 not in seqs       # second-oldest evicted instead
    assert sorted(seqs) == [1, 3, 4]


def test_best_recent_ignores_nan_val_mae():
    r = CheckpointRing(size=3)
    bad = _ckpt(1, float("nan"))
    good = _ckpt(2, 0.10)
    r.record(bad)
    r.record(good)
    assert r.best_recent().train_seq == 2
