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
