import numpy as np
import torch

from sim.residual_mlp import (
    FEATURE_DIM,
    ResidualMLP,
    build_features,
)


def test_feature_dim_matches():
    n = 5
    feats = build_features(
        v=np.ones(n), psi_dot=np.zeros(n),
        cmd_throttle=np.full(n, 50.0), cmd_steer=np.zeros(n),
        cmd_throttle_hist=np.zeros((n, 3)),
        cmd_steer_hist=np.zeros((n, 4)),
    )
    assert feats.shape == (n, FEATURE_DIM)


def test_mlp_forward_shape():
    net = ResidualMLP()
    x = torch.zeros(8, FEATURE_DIM)
    y = net(x)
    assert y.shape == (8, 2)
