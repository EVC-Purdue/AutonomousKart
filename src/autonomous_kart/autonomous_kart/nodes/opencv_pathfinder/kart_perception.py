"""
Camera frame -> tracked opponents, without ROS.

    frame --ROI crop--> KartDetector --boxes--> range_boxes (body frame)
          --ego pose at the image stamp--> body_to_world --> KartTracker
          --> Opponent list for /opponents

kart_detector_node is a thin ROS wrapper around KartPerception so this whole
chain is unit-testable with a stub detector and synthetic boxes.
"""

import bisect
import math
from collections import deque
from dataclasses import replace
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from autonomous_kart.nodes.opencv_pathfinder.kart_ranging import (
    CameraModel, RangingParams, body_to_world, range_boxes,
)
from autonomous_kart.nodes.opencv_pathfinder.kart_tracker import KartTracker, TrackFrame, TrackState
from autonomous_kart.nodes.pathfinder.opponents import Opponent

Pose = Tuple[float, float, float, float]  # x, y, yaw, speed


class PoseBuffer:
    """Recent ego poses so a detection is placed with the pose at its image
    stamp, not at whenever inference finished (at 6 m/s, 50 ms of lag is
    30 cm of error, and yaw lag swings far targets sideways)."""

    def __init__(self, horizon_s: float = 1.0):
        self.horizon_s = horizon_s
        self._t: deque = deque()
        self._p: deque = deque()

    def add(self, t: float, pose: Pose) -> None:
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._p.append(pose)
        while self._t and self._t[0] < t - self.horizon_s:
            self._t.popleft()
            self._p.popleft()

    def at(self, t: float, max_gap_s: float = 0.25) -> Optional[Pose]:
        """Interpolated pose at t; nearest within max_gap_s outside the range."""
        if not self._t:
            return None
        ts = list(self._t)
        i = bisect.bisect_left(ts, t)
        if i == 0 or i == len(ts):
            j = 0 if i == 0 else len(ts) - 1
            return self._p[j] if abs(ts[j] - t) <= max_gap_s else None
        t0, t1 = ts[i - 1], ts[i]
        a = (t - t0) / (t1 - t0)
        p0, p1 = self._p[i - 1], self._p[i]
        dyaw = (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi
        return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]),
                p0[2] + a * dyaw, p0[3] + a * (p1[3] - p0[3]))


class KartPerception:
    def __init__(
        self,
        detector: Callable,
        camera: CameraModel,
        frame: TrackFrame,
        ranging: RangingParams = RangingParams(),
        tracker_params: Optional[dict] = None,
        roi_frac: Sequence[float] = (0.0, 0.0, 1.0, 1.0),
        max_range_m: float = 40.0,
        ego_xy_sigma_m: float = 0.1,
        ego_yaw_sigma_rad: float = math.radians(1.0),
    ):
        self.detector = detector  # bgr -> Detections (KartDetector or a stub)
        self.camera = camera
        self.frame = frame
        self.ranging = ranging
        self.tracker = KartTracker(frame, tracker_params)
        self.roi_frac = tuple(float(v) for v in roi_frac)
        self.max_range = max_range_m
        self.ego_xy_sigma = ego_xy_sigma_m
        self.ego_yaw_sigma = ego_yaw_sigma_rad
        self._cam_cache: Optional[Tuple[Tuple[int, int, int], CameraModel]] = None
        self._ego_s: Optional[float] = None
        self.last_boxes = np.zeros((0, 4))

    def _camera_for(self, w: int, h: int, roi_bottom: int) -> CameraModel:
        key = (w, h, roi_bottom)
        if self._cam_cache is None or self._cam_cache[0] != key:
            cam = self.camera.scaled(w, h)
            # The ROI bottom is the effective image edge: a box cut off there
            # is truncated, so its bottom is not the kart's ground contact.
            self._cam_cache = (key, replace(cam, height_px=roi_bottom))
        return self._cam_cache[1]

    def _in_fov(self, cam: CameraModel, pose: Pose) -> Callable[[float, float], bool]:
        x0, y0, yaw = pose[0], pose[1], pose[2]
        half = math.atan2(0.5 * cam.width_px, cam.fx)
        c, s = math.cos(yaw), math.sin(yaw)

        def in_fov(x: float, y: float) -> bool:
            dx, dy = x - x0, y - y0
            bx, by = c * dx + s * dy - cam.mount_x_m, -s * dx + c * dy - cam.mount_y_m
            if bx <= 0.0 or math.hypot(bx, by) > self.max_range:
                return False
            b = math.atan2(by, bx) - cam.yaw_rad
            return abs((b + math.pi) % (2 * math.pi) - math.pi) <= half

        return in_fov

    def process(self, bgr: np.ndarray, stamp_s: float, pose: Pose) -> List[Opponent]:
        h, w = bgr.shape[:2]
        x1, y1, x2, y2 = (int(round(f * d)) for f, d in zip(self.roi_frac, (w, h, w, h)))
        det = self.detector(bgr[y1:y2, x1:x2])
        boxes = np.asarray(det.boxes, dtype=np.float64).reshape(-1, 4) + np.array([x1, y1, x1, y1])
        self.last_boxes = boxes
        cam = self._camera_for(w, h, y2)

        rb = range_boxes(cam, boxes, self.ranging)
        ok = rb.valid
        xy, cov = body_to_world(rb.xy[ok], rb.cov[ok], (pose[0], pose[1]), pose[2],
                                self.ego_xy_sigma, self.ego_yaw_sigma)
        far = np.hypot(rb.xy[ok][:, 0], rb.xy[ok][:, 1]) > self.max_range
        xy, cov = xy[~far], cov[~far]

        sd, _ = self.frame.to_frenet(np.array([[pose[0], pose[1]]]), np.eye(2)[None], hint_s=self._ego_s)
        self._ego_s = float(sd[0, 0])
        tracks = self.tracker.step(stamp_s, xy, cov, ego_s=self._ego_s, ego_s_dot=max(0.0, pose[3]),
                                   in_fov=self._in_fov(cam, pose))
        return [to_opponent(t) for t in tracks]


def to_opponent(t: TrackState) -> Opponent:
    return Opponent(t.id, t.s, t.d, t.s_dot, t.d_dot, t.t,
                    math.sqrt(max(t.cov[0, 0], 0.0)), math.sqrt(max(t.cov[1, 1], 0.0)))
