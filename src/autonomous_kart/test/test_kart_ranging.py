"""Unit tests for monocular kart ranging."""
import math

import numpy as np
import pytest

from autonomous_kart.nodes.opencv_pathfinder.kart_ranging import (
    CameraModel, RangingParams, body_to_world, project_ground_point, range_boxes,
)

# 360x202 matches what camera_node publishes; ~90 deg HFOV
CAM = CameraModel(
    fx=180.0, fy=180.0, cx=180.0, cy=101.0, width_px=360, height_px=202,
    height_m=1.3, pitch_rad=math.radians(6.0), mount_x_m=0.3,
)
# No centre offset, so the recovered point is the box's ground contact.
P0 = RangingParams(center_offset_m=0.0, center_offset_sigma_m=0.0)


def _box_for(cam, x, y, kart_h=1.0, half_w_px=10.0):
    """Synthetic box whose bottom-centre sits on ground point (x, y) and
    whose height matches a kart of kart_h at that range."""
    u, v = project_ground_point(cam, np.array([x]), np.array([y]))
    rng = math.hypot(x - cam.mount_x_m, y - cam.mount_y_m)
    h_px = cam.fy * kart_h / rng
    return np.array([[u[0] - half_w_px, v[0] - h_px, u[0] + half_w_px, v[0]]])


@pytest.mark.parametrize("x,y", [(5.0, 0.0), (8.0, 2.0), (15.0, -3.0), (25.0, 1.0)])
def test_recovers_ground_point(x, y):
    res = range_boxes(CAM, _box_for(CAM, x, y), P0)
    assert res.valid[0]
    assert res.xy[0, 0] == pytest.approx(x, rel=0.02)
    assert res.xy[0, 1] == pytest.approx(y, abs=0.05 + 0.01 * x)


def test_projection_round_trip_with_yaw():
    cam = CameraModel(**{**CAM.__dict__, "yaw_rad": math.radians(10.0), "mount_y_m": 0.2})
    res = range_boxes(cam, _box_for(cam, 10.0, 3.0), P0)
    assert res.xy[0] == pytest.approx([10.0, 3.0], abs=0.15)


def _var_x(x, **over):
    return range_boxes(CAM, _box_for(CAM, x, 0.0), RangingParams(**{**P0.__dict__, **over})).cov[0, 0, 0]


def test_fusion_leans_on_ground_near_and_beats_either_cue_far():
    ground_only = dict(kart_height_sigma_m=1e3)
    size_only = dict(pitch_sigma_rad=10.0)
    # Near: ground cue is tight, fused ~ ground alone
    assert _var_x(5.0) == pytest.approx(_var_x(5.0, **ground_only), rel=0.2)
    assert _var_x(5.0, **ground_only) < 0.25 * _var_x(5.0, **size_only)
    # Far: comparable cues, fusion beats both
    fused = _var_x(20.0)
    assert fused < 0.8 * _var_x(20.0, **ground_only)
    assert fused < 0.8 * _var_x(20.0, **size_only)


def test_covariance_grows_with_range_and_is_psd():
    boxes = np.vstack([_box_for(CAM, x, 1.0) for x in (4.0, 10.0, 20.0)])
    res = range_boxes(CAM, boxes, RangingParams())
    tr = res.cov[:, 0, 0] + res.cov[:, 1, 1]
    assert np.all(np.diff(tr) > 0)
    for c in res.cov:
        assert np.allclose(c, c.T)
        assert np.all(np.linalg.eigvalsh(c) > 0)


def test_box_above_horizon_uses_size_only():
    # Bottom edge above the principal point with the camera pitched down
    # (far above the horizon): no ground hit, size cue still gives a range.
    box = np.array([[170.0, 20.0, 190.0, 40.0]])
    res = range_boxes(CAM, box, RangingParams())
    assert res.valid[0]
    assert not res.used_ground[0] and res.used_size[0]
    assert res.range_m[0] == pytest.approx(180.0 * 1.0 / 20.0 + 0.8, rel=1e-6)


def test_bottom_truncated_box_still_reported_with_inflated_variance():
    box = np.array([[100.0, 60.0, 260.0, 201.0]])  # touches bottom edge
    res = range_boxes(CAM, box, RangingParams())
    assert res.valid[0]
    assert not res.used_ground[0]


def test_degenerate_box_invalid():
    box = np.array([[100.0, 0.0, 120.0, 1.0]])  # top-cut, tiny, above horizon
    res = range_boxes(CAM, box, RangingParams())
    assert not res.valid[0]
    assert np.all(np.isnan(res.xy[0]))


def test_center_offset_pushes_along_ray():
    box = _box_for(CAM, 10.0, 0.0)
    a = range_boxes(CAM, box, P0)
    b = range_boxes(CAM, box, RangingParams(center_offset_m=0.8, center_offset_sigma_m=0.0))
    assert b.xy[0, 0] - a.xy[0, 0] == pytest.approx(0.8, abs=1e-6)


def test_empty_batch():
    res = range_boxes(CAM, np.zeros((0, 4)))
    assert res.xy.shape == (0, 2) and res.cov.shape == (0, 2, 2)


def test_body_to_world_rotation_and_yaw_uncertainty():
    xy = np.array([[10.0, 0.0]])
    cov = np.array([np.diag([0.1, 0.01])])
    w, c = body_to_world(xy, cov, (5.0, 5.0), math.pi / 2)
    assert w[0] == pytest.approx([5.0, 15.0])
    # Range variance now lies along world y
    assert c[0, 1, 1] == pytest.approx(0.1) and c[0, 0, 0] == pytest.approx(0.01)
    _, c2 = body_to_world(xy, cov, (5.0, 5.0), math.pi / 2, ego_yaw_sigma_rad=0.02)
    # Yaw error at 10 m adds (10 * 0.02)^2 tangentially (world x here)
    assert c2[0, 0, 0] == pytest.approx(0.01 + 0.04)
