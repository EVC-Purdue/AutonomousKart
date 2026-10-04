"""Bird's-eye warp + polynomial fit: pixels -> meters (see docs plan).

Reuses angle.py's existing red-mask step (same tuned HSV threshold), then
undoes perspective distortion via a warp *before* measuring anything — a
straight road reads as straight only after this warp; before it, a kart
centered mid-corner and a kart drifting on a straight can look the same to
a single angle-to-corner metric the way AngleFinder's does.

No ROS dependency, same design as AngleFinder — directly unit-testable.

Every warp/scale parameter here is a placeholder until calibrated against
footage of the real taped course (see params/road_geometry.yaml for the
calibration procedure) — this class takes them as a plain params dict
rather than reading ROS parameters itself, same pattern as SafetyChecker/
the planner classes use for testability.
"""
import math
import os
import time

import cv2 as cv
import numpy as np

from autonomous_kart.nodes.opencv_pathfinder import utils
from autonomous_kart.nodes.opencv_pathfinder.angle import KERNEL, LOWER_RED, UPPER_RED


class RoadGeometry:
    def __init__(self, params: dict, logger=None):
        self.params = params
        self.logger = logger

        src = np.float32([
            params.get("warp_src_tl", [700.0, 550.0]),
            params.get("warp_src_tr", [936.0, 550.0]),
            params.get("warp_src_br", [1400.0, 900.0]),
            params.get("warp_src_bl", [236.0, 900.0]),
        ])
        self.dst_w = int(params.get("warp_dst_width", 400))
        self.dst_h = int(params.get("warp_dst_height", 600))
        dst = np.float32([
            [0.0, 0.0], [self.dst_w, 0.0],
            [self.dst_w, self.dst_h], [0.0, self.dst_h],
        ])
        self.M = cv.getPerspectiveTransform(src, dst)

        self.meters_per_pixel_x = float(params.get("meters_per_pixel_x", 0.01))
        self.meters_per_pixel_y = float(params.get("meters_per_pixel_y", 0.01))
        self.scan_row_count = int(params.get("scan_row_count", 20))
        self.min_points_per_side = int(params.get("min_points_per_side", 5))
        self.smoothing_alpha = float(params.get("smoothing_alpha", 0.3))

        self._smoothed = None  # (offset_m, heading_err_rad, curvature_1pm)
        self.start_time = time.strftime("%Y_%m_%d_%H_%M_%S")

    def mask(self, frame: np.ndarray) -> np.ndarray:
        hsv = utils.convert_bgr_to_hsv(frame)
        red = cv.inRange(hsv, LOWER_RED, UPPER_RED)
        return cv.morphologyEx(red, cv.MORPH_OPEN, KERNEL)

    def warp(self, mask: np.ndarray) -> np.ndarray:
        return cv.warpPerspective(mask, self.M, (self.dst_w, self.dst_h))

    def scan_rows(self, warped_mask: np.ndarray):
        """Per-row boundary points: the tape pixel nearest to center on
        each side (the tape's inner edge is what actually delineates the
        drivable area), for scan_row_count evenly-spaced rows."""
        h, w = warped_mask.shape[:2]
        mid = w // 2
        left_pts = []
        right_pts = []
        rows = np.linspace(0, h - 1, self.scan_row_count).astype(int)
        for row in rows:
            cols = np.nonzero(warped_mask[row])[0]
            if cols.size == 0:
                continue
            left_cols = cols[cols < mid]
            right_cols = cols[cols >= mid]
            if left_cols.size:
                left_pts.append((float(row), float(left_cols.max())))
            if right_cols.size:
                right_pts.append((float(row), float(right_cols.min())))
        return left_pts, right_pts

    def fit_and_measure(self, left_pts, right_pts):
        """Returns (offset_m, heading_err_rad, curvature_1pm) or None if
        either side has too few points to fit reliably."""
        if len(left_pts) < self.min_points_per_side or len(right_pts) < self.min_points_per_side:
            return None

        left_pts = np.array(left_pts, dtype=np.float64)
        right_pts = np.array(right_pts, dtype=np.float64)

        # Convert to physical units *before* fitting, so the fit
        # coefficients are already correctly scaled — avoids re-deriving
        # how polynomial coefficients transform under an independent x/y
        # rescale, which is easy to get subtly wrong after the fact.
        left_y = left_pts[:, 0] * self.meters_per_pixel_y
        left_x = left_pts[:, 1] * self.meters_per_pixel_x
        right_y = right_pts[:, 0] * self.meters_per_pixel_y
        right_x = right_pts[:, 1] * self.meters_per_pixel_x

        left_fit = np.polyfit(left_y, left_x, 2)    # x = a*y^2 + b*y + c
        right_fit = np.polyfit(right_y, right_x, 2)

        near_y = (self.dst_h - 1) * self.meters_per_pixel_y
        left_x_near = np.polyval(left_fit, near_y)
        right_x_near = np.polyval(right_fit, near_y)
        center_x_near = 0.5 * (left_x_near + right_x_near)
        image_center_x = (self.dst_w / 2.0) * self.meters_per_pixel_x

        offset_m = float(center_x_near - image_center_x)

        left_slope = 2 * left_fit[0] * near_y + left_fit[1]
        right_slope = 2 * right_fit[0] * near_y + right_fit[1]
        center_slope = 0.5 * (left_slope + right_slope)
        heading_err_rad = float(math.atan(center_slope))

        center_a = 0.5 * (left_fit[0] + right_fit[0])
        curvature_1pm = float(2.0 * center_a / (1.0 + center_slope ** 2) ** 1.5)

        return offset_m, heading_err_rad, curvature_1pm

    def update(self, frame: np.ndarray):
        """Runs mask -> warp -> scan -> fit, EMA-smooths the numeric
        outputs when valid. Returns (offset_m, heading_err_rad,
        curvature_1pm, valid). On invalid, returns NaNs (no stale-value
        fallback) — matches this codebase's existing fail-open convention
        (SafetyChecker's NaN handling) rather than silently holding a
        possibly-outdated reading."""
        m = self.mask(frame)
        warped = self.warp(m)
        left_pts, right_pts = self.scan_rows(warped)
        result = self.fit_and_measure(left_pts, right_pts)

        if result is None:
            return (math.nan, math.nan, math.nan, False)

        if self._smoothed is None:
            self._smoothed = result
        else:
            a = self.smoothing_alpha
            self._smoothed = tuple(
                a * new + (1.0 - a) * old
                for new, old in zip(result, self._smoothed)
            )
        return (*self._smoothed, True)

    def write_debug_frame(self, warped_mask, left_pts, right_pts, log_folder, frame_count):
        """Dumps the warped mask with detected boundary points overlaid —
        the only real way to sanity-check the warp calibration once real
        tape footage exists. Mirrors AngleFinder's debug-dump pattern."""
        img = cv.cvtColor(warped_mask, cv.COLOR_GRAY2BGR)
        for row, col in left_pts:
            cv.circle(img, (int(col), int(row)), 2, (255, 0, 0), -1)
        for row, col in right_pts:
            cv.circle(img, (int(col), int(row)), 2, (0, 0, 255), -1)
        out_dir = os.path.join(log_folder, "road_geometry_debug", self.start_time)
        os.makedirs(out_dir, exist_ok=True)
        cv.imwrite(os.path.join(out_dir, f"frame_{frame_count}.jpg"), img)
