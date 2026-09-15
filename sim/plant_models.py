"""Candidate plants: a linear ARX baseline and an MLP ensemble.

The baseline is not decoration. With 9.77 minutes of data a linear map on the
same features may beat the network, and without the comparison there is no way
to tell whether the network earned its parameters.
"""
from __future__ import annotations

from typing import Callable

import numpy as np

from sim.plant_dataset import HISTORY, TARGET_NAMES, Whitener


class LinearARX:
    """Least-squares map from a flattened feature window to one-step targets."""

    def __init__(self, coef: np.ndarray | None = None, intercept: np.ndarray | None = None):
        self.coef = coef
        self.intercept = intercept

    def fit(self, features: np.ndarray, targets: np.ndarray) -> "LinearARX":
        flat = features.reshape(len(features), -1)
        A = np.column_stack([flat, np.ones(len(flat))])
        sol, *_ = np.linalg.lstsq(A, targets, rcond=None)
        self.coef = sol[:-1]
        self.intercept = sol[-1]
        return self

    def predict(self, features: np.ndarray) -> np.ndarray:
        flat = features.reshape(len(features), -1)
        return flat @ self.coef + self.intercept

    @property
    def n_params(self) -> int:
        return int(self.coef.size + self.intercept.size)

    def to_dict(self) -> dict:
        return {"coef": self.coef.tolist(), "intercept": self.intercept.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "LinearARX":
        return cls(np.asarray(d["coef"], dtype=float), np.asarray(d["intercept"], dtype=float))


def as_stepper(model, feature_whitener: Whitener, target_whitener: Whitener) -> Callable:
    """Wrap a model into the (window, delta_cmd, throttle_sp) -> accelerations form."""
    def stepper(window, delta_cmd, throttle_sp):
        z = feature_whitener.apply(window[None, ...])
        pred = model.predict(z)
        out = target_whitener.invert(pred)[0]
        return tuple(float(c) for c in out[:len(TARGET_NAMES)])
    return stepper


def _torch():
    import torch
    return torch


class PlantMLP:
    """Two hidden layers plus a linear skip, built lazily so numpy-only use of
    this module does not require torch.

    The skip exists because most of this plant is linear: `ds` is v*dt to
    within measurement error, and a plain least-squares fit on these features
    reaches R^2 = 1.0000 on it. An MLP with LayerNorm after each hidden layer
    is per-sample scale-invariant, which is precisely the wrong inductive bias
    for reproducing an output proportional to an input's magnitude -- trained
    without the skip, the network carried a fixed 0.0115 m/tick offset in `ds`,
    or 0.69 m/s, and lost to the 84-parameter baseline at every horizon. The
    skip hands it the linear part for free and leaves the branch to learn the
    residual, so it starts from the baseline rather than having to rediscover
    it through a normalisation that fights it."""

    def __new__(cls, hidden: int = 32):
        torch = _torch()

        class _Net(torch.nn.Module):
            def __init__(self):
                super().__init__()
                d_in = HISTORY * 4
                self.flatten = torch.nn.Flatten()
                self.skip = torch.nn.Linear(d_in, len(TARGET_NAMES))
                self.net = torch.nn.Sequential(
                    torch.nn.Flatten(),
                    torch.nn.Linear(d_in, hidden),
                    torch.nn.LayerNorm(hidden),
                    torch.nn.SiLU(),
                    torch.nn.Linear(hidden, hidden),
                    torch.nn.LayerNorm(hidden),
                    torch.nn.SiLU(),
                    torch.nn.Linear(hidden, len(TARGET_NAMES)),
                )
                self.hidden = hidden

            def forward(self, x):
                return self.skip(self.flatten(x)) + self.net(x)

            @property
            def n_params(self):
                return sum(p.numel() for p in self.parameters())

        return _Net()


class PlantEnsemble:
    """Mean of several PlantMLP members. Spread is the uncertainty signal."""

    def __init__(self, members):
        self.members = list(members)

    def predict(self, features: np.ndarray) -> np.ndarray:
        torch = _torch()
        x = torch.as_tensor(np.asarray(features), dtype=torch.float32)
        with torch.no_grad():
            stacked = torch.stack([m(x) for m in self.members])
        return stacked.mean(dim=0).numpy()

    def spread(self, features: np.ndarray) -> np.ndarray:
        torch = _torch()
        x = torch.as_tensor(np.asarray(features), dtype=torch.float32)
        with torch.no_grad():
            stacked = torch.stack([m(x) for m in self.members])
        return stacked.std(dim=0).numpy()

    def save(self, path: str) -> None:
        torch = _torch()
        torch.save({"hidden": self.members[0].hidden,
                    "states": [m.state_dict() for m in self.members]}, path)

    @classmethod
    def load(cls, path: str) -> "PlantEnsemble":
        torch = _torch()
        # weights_only=True: the blob holds tensors and an int, nothing to unpickle.
        blob = torch.load(path, map_location="cpu", weights_only=True)
        members = []
        for state in blob["states"]:
            m = PlantMLP(hidden=blob["hidden"])
            m.load_state_dict(state)
            m.eval()
            members.append(m)
        return cls(members)
