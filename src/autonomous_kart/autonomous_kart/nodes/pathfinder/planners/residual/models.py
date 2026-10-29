"""The residual's model, chosen by `mpc.residual.model_size`.

    xs  constant          the learner with every feature but the bias masked
    s   ridge linear      recursive least squares, updated every tick
    m   gradient boosting
    l   64x64 tanh net
    xl  128x128 tanh net

xs and s are the recursive learner alone. m, l and xl add a batch model fitted
in the background thread from the ring buffer, with the recursive learner still
running underneath as the fallback and as the baseline it is scored against.
"""
from __future__ import annotations

from typing import Callable

SIZES = ("xs", "s", "m", "l", "xl")
BATCH = ("m", "l", "xl")


def estimator(size: str, params: dict) -> Callable[[], object]:
    """A zero-argument factory for one channel's regressor."""
    g = params.get
    if size == "m":
        def gbm():
            from sklearn.ensemble import HistGradientBoostingRegressor
            return HistGradientBoostingRegressor(
                max_iter=int(g("gbm_max_iter", 60)),
                max_depth=int(g("gbm_max_depth", 3)),
                learning_rate=float(g("gbm_learning_rate", 0.1)),
                min_samples_leaf=int(g("gbm_min_samples_leaf", 20)))
        return gbm

    hidden = {"l": (64, 64), "xl": (128, 128)}[size]

    def mlp():
        # Scaled in the pipeline: the feature vector mixes metres, radians and
        # m/s^2, and an unscaled net spends its first epochs undoing that.
        from sklearn.neural_network import MLPRegressor
        from sklearn.pipeline import make_pipeline
        from sklearn.preprocessing import StandardScaler
        return make_pipeline(StandardScaler(), MLPRegressor(
            hidden_layer_sizes=hidden, activation="tanh",
            learning_rate_init=float(g("mlp_learning_rate", 2e-3)),
            alpha=float(g("mlp_alpha", 1e-4)),
            batch_size=int(g("mlp_batch_size", 256)),
            max_iter=int(g("mlp_max_iter", 150)),
            early_stopping=True,
            n_iter_no_change=int(g("mlp_n_iter_no_change", 10)),
            random_state=0))
    return mlp


def compile_single_row(est) -> Callable:
    """A fast single-row predictor for a fitted gradient-boosting model.

    sklearn's HistGradientBoostingRegressor.predict costs ~3-9 ms for one row
    (input validation plus an OpenMP dispatch across every core), which the
    60 Hz planner pays every tick. This walks the same trees with a handful
    of numpy ops instead (~tens of us) and sums the leaves in sklearn's order,
    so the output matches predict() bit for bit. Anything that isn't a plain
    numeric GBM (the MLP pipelines, categorical splits, a sklearn whose
    internals moved) gets est.predict back unchanged.
    """
    import numpy as np

    def fallback(x):
        return float(est.predict(x.reshape(1, -1))[0])

    try:
        trees = [p[0] for p in est._predictors]
        if not trees or any(len(p) != 1 for p in est._predictors):
            return fallback
        if type(est._loss.link).__name__ != "IdentityLink":
            return fallback
        offsets = np.cumsum([0] + [len(t.nodes) for t in trees[:-1]])
        nodes = np.concatenate([t.nodes for t in trees])
        if nodes["is_categorical"].any():
            return fallback
        leaf = nodes["is_leaf"].astype(bool)
        n_off = np.repeat(offsets, [len(t.nodes) for t in trees])
        idx_all = np.arange(len(nodes))
        # Leaves point at themselves so every tree can take max_depth steps.
        left = np.where(leaf, idx_all, nodes["left"] + n_off)
        right = np.where(leaf, idx_all, nodes["right"] + n_off)
        feat = np.where(leaf, 0, nodes["feature_idx"]).astype(np.intp)
        thr = nodes["num_threshold"].astype(np.float64)
        miss_left = nodes["missing_go_to_left"].astype(bool)
        value = nodes["value"].astype(np.float64)
        roots = offsets.astype(np.intp)
        depth = int(nodes["depth"].max())
        baseline = float(np.asarray(est._baseline_prediction).ravel()[0])
    except (AttributeError, KeyError, IndexError, TypeError, ValueError):
        return fallback

    def predict(x):
        i = roots
        for _ in range(depth):
            xv = x[feat[i]]
            go_left = (xv <= thr[i]) | (np.isnan(xv) & miss_left[i])
            i = np.where(go_left, left[i], right[i])
        # sklearn adds the trees one at a time onto the baseline; accumulate
        # keeps that order so the float result is identical.
        return float(np.add.accumulate(np.concatenate(([baseline], value[i])))[-1])

    # Refuse to swap in a walk that disagrees with sklearn on anything.
    n_feat = int(getattr(est, "n_features_in_", 0)) or int(feat.max()) + 1
    rng = np.random.default_rng(0)
    probe = rng.normal(scale=2.0, size=(32, n_feat))
    probe[0] = 0.0
    probe[1, 0] = np.nan
    if not np.array_equal(est.predict(probe), [predict(r) for r in probe]):
        return fallback
    return predict
