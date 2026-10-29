"""Tests for the ROS-free perception chain behind kart_detector_node."""
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from autonomous_kart.nodes.opencv_pathfinder.kart_perception import KartPerception, PoseBuffer
from autonomous_kart.nodes.opencv_pathfinder.kart_ranging import CameraModel, RangingParams, project_ground_point
from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import TrackFrame
from autonomous_kart.nodes.pathfinder.opponents import Opponent, decode_opponents, encode_opponents

_LINE6 = os.path.join(os.path.dirname(__file__), "..", "..", "..", "data", "racing_line", "line6.csv")
CAM = CameraModel(fx=180.0, fy=180.0, cx=180.0, cy=101.0, width_px=360, height_px=202,
                  height_m=1.2, pitch_rad=math.radians(5.0))
KART_H = 1.0
DT = 1.0 / 30.0


@pytest.fixture(scope="module")
def frame():
    if not os.path.exists(_LINE6):
        pytest.skip("racing line data not available")
    return TrackFrame([tuple(r) for r in np.genfromtxt(_LINE6, delimiter=",", skip_header=1)])


class _StubDetector:
    """Returns whatever boxes the test queued; records the crop it was given."""

    def __init__(self):
        self.boxes = np.zeros((0, 4))
        self.crop_shape = None

    def __call__(self, bgr):
        self.crop_shape = bgr.shape
        return SimpleNamespace(boxes=self.boxes)


def _box_for(ego, opp_xy):
    """Image box (full-frame pixels) of a kart centred at opp_xy seen from ego pose."""
    c, s = math.cos(ego[2]), math.sin(ego[2])
    dx, dy = opp_xy[0] - ego[0], opp_xy[1] - ego[1]
    bx, by = c * dx + s * dy, -s * dx + c * dy
    u, v = project_ground_point(CAM, np.array([bx]), np.array([by]))
    h = CAM.fy * KART_H / math.hypot(bx, by)
    return np.array([[u[0] - 0.6 * h, v[0] - h, u[0] + 0.6 * h, v[0]]])


# Pose buffer


def test_pose_buffer_interpolates_and_wraps_yaw():
    pb = PoseBuffer()
    pb.add(1.0, (0.0, 0.0, math.pi - 0.1, 5.0))
    pb.add(1.1, (1.0, 2.0, -math.pi + 0.1, 7.0))
    x, y, yaw, v = pb.at(1.05)
    assert (x, y, v) == pytest.approx((0.5, 1.0, 6.0))
    assert abs(abs(yaw) - math.pi) < 1e-9  # through +-pi, not through 0


def test_pose_buffer_edges():
    pb = PoseBuffer()
    assert pb.at(1.0) is None
    pb.add(1.0, (1.0, 0.0, 0.0, 0.0))
    assert pb.at(1.2) == (1.0, 0.0, 0.0, 0.0)  # within max_gap
    assert pb.at(2.0) is None
    pb.add(0.5, (9.0, 9.0, 0.0, 0.0))  # out of order: ignored
    assert pb.at(1.0)[0] == 1.0


# Wire format


def test_opponents_round_trip():
    opps = (Opponent(3, 120.5, -0.4, 6.1, 0.2, 1.7e9 + 0.123456, 0.8, 0.3),
            Opponent(7, 5.0, 1.0, 0.0, 0.0, 1.7e9, 0.0, 0.0))
    back = decode_opponents(encode_opponents(opps))
    assert back == opps
    assert back[0].stamp_s == 1.7e9 + 0.123456  # float64: sub-ms stamps survive
    assert decode_opponents([]) == () and decode_opponents([0.0]) == ()


def test_opponents_bad_payload():
    with pytest.raises(ValueError):
        decode_opponents([2.0, 1.0, 2.0])


# End to end


def test_tracks_a_kart_ahead_on_the_real_line(frame):
    det = _StubDetector()
    perc = KartPerception(det, CAM, frame, RangingParams(center_offset_m=0.0, kart_height_m=KART_H),
                          roi_frac=(0.0, 0.0, 1.0, 1.0))
    img = np.zeros((202, 360, 3), np.uint8)
    s_ego0, v_ego, s_opp0, v_opp, d_opp = 100.0, 6.0, 115.0, 4.0, 0.6
    opps = []
    for k in range(45):
        t = 1000.0 + k * DT
        ex, ey, epsi, _ = frame.pose_at(s_ego0 + v_ego * k * DT)
        ego = (float(ex), float(ey), float(epsi), v_ego)
        ox, oy, _, _ = frame.to_world(s_opp0 + v_opp * k * DT, d_opp)
        det.boxes = _box_for(ego, (ox, oy))
        opps = perc.process(img, t, ego)
    assert len(opps) == 1
    o = opps[0]
    assert float(frame.wrap_ds(o.s - (s_opp0 + v_opp * 44 * DT))) == pytest.approx(0.0, abs=0.6)
    assert o.d == pytest.approx(d_opp, abs=0.3)
    assert o.s_dot == pytest.approx(v_opp, abs=1.0)
    assert o.sigma_s > 0.0 and o.sigma_d > 0.0


def test_roi_crops_and_offsets_boxes(frame):
    det = _StubDetector()
    perc = KartPerception(det, CAM, frame, roi_frac=(0.1, 0.0, 0.9, 0.6))
    det.boxes = np.array([[10.0, 20.0, 30.0, 40.0]])  # crop pixels
    x, y, psi, _ = frame.pose_at(50.0)
    perc.process(np.zeros((202, 360, 3), np.uint8), 0.0, (float(x), float(y), float(psi), 5.0))
    assert det.crop_shape == (121, 288, 3)
    assert perc.last_boxes[0].tolist() == [46.0, 20.0, 66.0, 40.0]  # back in full-frame pixels


def test_box_cut_by_roi_bottom_is_treated_as_truncated(frame):
    # Bottom edge sits on the ROI boundary: the kart's ground contact is hidden,
    # so the ground cue must not be used (it would place the kart too far).
    from autonomous_kart.nodes.opencv_pathfinder.kart_ranging import range_boxes
    perc = KartPerception(_StubDetector(), CAM, frame, roi_frac=(0.0, 0.0, 1.0, 0.6))
    box = np.array([[150.0, 80.0, 210.0, 120.5]])  # bottom on the ROI edge (row 121)
    assert not range_boxes(perc._camera_for(360, 202, 121), box).used_ground[0]
    assert range_boxes(CAM, box).used_ground[0]  # full-frame camera would have trusted it
