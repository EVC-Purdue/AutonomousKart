"""
Unit tests for RoadGeometry (bird's-eye warp + polynomial fit,
docs plan "pixels -> meters"). Pure Python + OpenCV, no rclpy dependency —
mirrors test_safety_checker.py.

Synthetic frames use a near-identity warp (warp_src = the raw frame's own
corners, warp_dst = the same size) so expected outputs can be computed by
hand instead of guessed — the warp math itself (a straight
cv2.getPerspectiveTransform/warpPerspective call) isn't the part worth
testing here; the fit/unit-conversion/smoothing math is.

Also worth knowing (found while writing these): angle.py's
LOWER_RED/UPPER_RED mask, reused here, does NOT actually threshold for red
— verified empirically: HSV S<=50 with a full H range means it detects
low-saturation, bright pixels (white/light-gray), and explicitly excludes
saturated red (H=0,S=255,V=255 -> unmasked). So these synthetic frames draw
white lines, matching what the mask really responds to, not the name.
"""
import numpy as np
import pytest

cv = pytest.importorskip("cv2")

from autonomous_kart.nodes.opencv_pathfinder.road_geometry import RoadGeometry

W, H = 400, 300

PARAMS = {
    # Identity-ish warp: source = the raw frame's own corners, destination
    # = the same size. Makes expected outputs computable by hand.
    "warp_src_tl": [0.0, 0.0], "warp_src_tr": [float(W), 0.0],
    "warp_src_br": [float(W), float(H)], "warp_src_bl": [0.0, float(H)],
    "warp_dst_width": W, "warp_dst_height": H,
    "meters_per_pixel_x": 0.01, "meters_per_pixel_y": 0.01,
    "scan_row_count": 20, "min_points_per_side": 5, "smoothing_alpha": 0.3,
}


def _geometry(**overrides):
    return RoadGeometry(dict(PARAMS, **overrides))


def _frame(left_x_fn, right_x_fn):
    """A black frame with two full-height white boundary lines (the mask
    ACTUALLY detects white/bright/low-saturation, not red — see module
    docstring), positioned per the given per-row functions."""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    for row in range(H):
        lx, rx = int(left_x_fn(row)), int(right_x_fn(row))
        cv.line(frame, (lx, row), (lx, row), (255, 255, 255), 6)
        cv.line(frame, (rx, row), (rx, row), (255, 255, 255), 6)
    return frame


def test_centered_straight_boundaries_give_near_zero_offset_heading_curvature():
    rg = _geometry()
    frame = _frame(lambda r: 100, lambda r: 300)  # midpoint 200 == image center
    offset_m, heading_err_rad, curvature_1pm, valid = rg.update(frame)
    assert valid is True
    assert offset_m == pytest.approx(0.0, abs=1e-9)
    assert heading_err_rad == pytest.approx(0.0, abs=1e-9)
    assert curvature_1pm == pytest.approx(0.0, abs=1e-9)


def test_offset_boundaries_give_expected_offset_in_meters():
    rg = _geometry()
    # midpoint (120+320)/2 = 220; image center = 200; offset = 20px * 0.01 = 0.2m
    frame = _frame(lambda r: 120, lambda r: 320)
    offset_m, heading_err_rad, curvature_1pm, valid = rg.update(frame)
    assert valid is True
    assert offset_m == pytest.approx(0.2, abs=1e-6)
    assert heading_err_rad == pytest.approx(0.0, abs=1e-9)


def test_curved_boundaries_recover_correctly_signed_curvature():
    rg_pos = _geometry()
    frame_pos = _frame(lambda r: 150 + 0.002 * (r - 150) ** 2,
                        lambda r: 250 + 0.002 * (r - 150) ** 2)
    _, _, curvature_pos, valid_pos = rg_pos.update(frame_pos)
    assert valid_pos is True
    assert curvature_pos > 0.1  # clearly nonzero, not just noise

    rg_neg = _geometry()
    frame_neg = _frame(lambda r: 150 - 0.002 * (r - 150) ** 2,
                        lambda r: 250 - 0.002 * (r - 150) ** 2)
    _, _, curvature_neg, valid_neg = rg_neg.update(frame_neg)
    assert valid_neg is True
    assert curvature_neg < -0.1

    # Same-magnitude opposite-sign curves should give (roughly) opposite
    # sign, same rough magnitude — not two unrelated numbers.
    assert curvature_pos == pytest.approx(-curvature_neg, rel=0.05)


def test_too_few_boundary_points_is_invalid_not_a_crash():
    rg = _geometry()
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    cv.circle(frame, (150, 150), 3, (255, 255, 255), -1)  # one dot, one row
    offset_m, heading_err_rad, curvature_1pm, valid = rg.update(frame)
    assert valid is False
    assert np.isnan(offset_m) and np.isnan(heading_err_rad) and np.isnan(curvature_1pm)


def test_invalid_frame_does_not_raise_on_first_call():
    """No prior _smoothed state yet — must NaN out cleanly, not crash on
    an unset smoothing state."""
    rg = _geometry()
    frame = np.zeros((H, W, 3), dtype=np.uint8)  # nothing drawn at all
    result = rg.update(frame)
    assert result[3] is False


def test_ema_smoothing_blends_toward_new_reading_by_alpha():
    rg = _geometry(smoothing_alpha=0.3)
    frame_a = _frame(lambda r: 100, lambda r: 300)   # offset 0.0
    offset_a, _, _, valid_a = rg.update(frame_a)
    assert valid_a is True
    assert offset_a == pytest.approx(0.0, abs=1e-6)

    frame_b = _frame(lambda r: 150, lambda r: 350)   # raw offset 0.5
    offset_b, _, _, valid_b = rg.update(frame_b)
    assert valid_b is True
    # smoothed = alpha*new + (1-alpha)*old = 0.3*0.5 + 0.7*0.0 = 0.15
    assert offset_b == pytest.approx(0.15, abs=1e-6)
    # Not equal to either the raw new reading or the previous one.
    assert offset_b != pytest.approx(0.5, abs=1e-3)
    assert offset_b != pytest.approx(0.0, abs=1e-3)


def test_scan_rows_finds_points_on_both_sides():
    rg = _geometry()
    frame = _frame(lambda r: 100, lambda r: 300)
    warped = rg.warp(rg.mask(frame))
    left_pts, right_pts = rg.scan_rows(warped)
    assert len(left_pts) >= PARAMS["min_points_per_side"]
    assert len(right_pts) >= PARAMS["min_points_per_side"]
    # left points should cluster near x=100, right points near x=300
    assert np.mean([c for _, c in left_pts]) == pytest.approx(100, abs=5)
    assert np.mean([c for _, c in right_pts]) == pytest.approx(300, abs=5)
