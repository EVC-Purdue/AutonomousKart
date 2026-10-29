"""
Monocular range + bearing for detected karts.

One forward camera, no depth sensor. Each bounding box gives two
independent range cues:

  ground : the box's bottom edge is where the kart meets the track. Casting
           that pixel's ray onto the ground plane gives range. Precise up
           close; far out it degrades fast because a 1 deg pitch wobble
           (kart chassis bouncing) moves a shallow intersection by metres.
  size   : the box's pixel height against a prior kart height. Immune to
           pitch, but only as good as the height prior (teams' karts differ).

The cues are fused by inverse variance, so near targets lean on the ground
cue and far targets on size. Output is the kart CENTRE in the ego body frame
(x forward, y left, REP-103) with a 2x2 covariance the tracker uses as its
measurement noise.

Everything is vectorized over boxes: one call per frame, no per-box Python.
"""

import math
from dataclasses import dataclass, replace
from typing import Tuple

import numpy as np

# Rays shallower than this below the horizon are treated as "no ground hit".
_MIN_DEPRESSION_RAD = math.radians(0.3)


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width_px: int  # image size the intrinsics refer to
    height_px: int
    height_m: float  # lens above ground
    pitch_rad: float  # positive = looking down
    yaw_rad: float = 0.0  # positive = looking left
    mount_x_m: float = 0.0  # lens position in the body frame
    mount_y_m: float = 0.0
    dist: Tuple[float, ...] = ()  # OpenCV distortion coeffs (k1, k2, p1, p2, k3)

    def scaled(self, width_px: int, height_px: int) -> "CameraModel":
        """Intrinsics for a resized image (camera_node downsizes before publishing)."""
        sx = width_px / self.width_px
        sy = height_px / self.height_px
        return replace(
            self,
            fx=self.fx * sx, fy=self.fy * sy, cx=self.cx * sx, cy=self.cy * sy,
            width_px=width_px, height_px=height_px,
        )

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class RangingParams:
    kart_height_m: float = 1.0  # prior on opponent height, ground to top of box
    kart_height_sigma_m: float = 0.2
    # Box bottom-centre is the near face; the centre is roughly half a kart
    # further along the ray. Orientation-dependent, hence the sigma.
    center_offset_m: float = 0.8
    center_offset_sigma_m: float = 0.25
    box_sigma_px: float = 2.0  # detector box-edge jitter
    pitch_sigma_rad: float = math.radians(1.0)  # chassis bounce not in the model
    edge_margin_px: float = 2.0  # box within this of the border = truncated


@dataclass
class RangeBatch:
    """Per-box results. Rows with valid=False carry NaNs."""

    xy: np.ndarray  # (M, 2) kart centre, body frame
    cov: np.ndarray  # (M, 2, 2)
    range_m: np.ndarray  # (M,) horizontal range from the lens
    bearing_rad: np.ndarray  # (M,) left positive
    used_ground: np.ndarray  # (M,) bool
    used_size: np.ndarray  # (M,) bool
    valid: np.ndarray  # (M,) bool


def _normalized(cam: CameraModel, u: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if cam.dist:
        import cv2  # only needed with distortion; keeps the math importable without it

        pts = np.stack([u, v], axis=-1).reshape(-1, 1, 2).astype(np.float64)
        n = cv2.undistortPoints(pts, cam.K, np.asarray(cam.dist, dtype=np.float64)).reshape(-1, 2)
        return n[:, 0], n[:, 1]
    return (u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy


def _rays_body(cam: CameraModel, xn: np.ndarray, yn: np.ndarray) -> Tuple[np.ndarray, ...]:
    """Camera ray (xn, yn, 1) (OpenCV: x right, y down, z fwd) -> body frame."""
    c, s = math.cos(cam.pitch_rad), math.sin(cam.pitch_rad)
    # Un-pitched body axes: fwd = z_cam, left = -x_cam, up = -y_cam; then
    # pitch down about +y (left).
    bx = c - s * yn
    by = -xn
    bz = -s - c * yn
    if cam.yaw_rad:
        cy, sy = math.cos(cam.yaw_rad), math.sin(cam.yaw_rad)
        bx, by = cy * bx - sy * by, sy * bx + cy * by
    return bx, by, bz


def range_boxes(cam: CameraModel, boxes_xyxy: np.ndarray, p: RangingParams = RangingParams()) -> RangeBatch:
    """Boxes (M, 4) as x1, y1, x2, y2 in pixels of cam's image size."""
    b = np.asarray(boxes_xyxy, dtype=np.float64).reshape(-1, 4)
    m = b.shape[0]
    u = 0.5 * (b[:, 0] + b[:, 2])
    v_bot = b[:, 3]
    h_px = b[:, 3] - b[:, 1]

    xn, yn = _normalized(cam, u, v_bot)
    bx, by, bz = _rays_body(cam, xn, yn)
    hn = np.hypot(bx, by)
    bearing = np.arctan2(by, bx)

    # Ground cue
    depression = np.arctan2(-bz, hn)
    bottom_cut = v_bot >= cam.height_px - 1 - p.edge_margin_px
    g_ok = (depression > _MIN_DEPRESSION_RAD) & ~bottom_cut
    with np.errstate(divide="ignore", invalid="ignore"):
        r_g = cam.height_m / np.tan(depression)
        sig_a2 = (p.box_sigma_px / cam.fy) ** 2 + p.pitch_sigma_rad ** 2
        # dR/d(alpha) = -h / sin^2(alpha)
        var_g = (cam.height_m / np.sin(depression) ** 2) ** 2 * sig_a2

    # Size cue
    top_cut = b[:, 1] <= p.edge_margin_px
    s_ok = (h_px > 2.0 * p.box_sigma_px) & ~top_cut & ~bottom_cut
    with np.errstate(divide="ignore", invalid="ignore"):
        r_s = cam.fy * p.kart_height_m / h_px
        rel2 = (p.kart_height_sigma_m / p.kart_height_m) ** 2 + 2.0 * (p.box_sigma_px / h_px) ** 2
        var_s = r_s * r_s * rel2

    # Truncated at the bottom (kart right on top of us): neither cue is
    # trustworthy, but dropping the box would be the worst outcome. Fall
    # back to size-with-inflated-variance; the range is an upper bound.
    fallback = bottom_cut & (h_px > 2.0 * p.box_sigma_px)
    s_ok = s_ok | fallback
    var_s = np.where(fallback, var_s * 9.0, var_s)

    w_g = np.where(g_ok, 1.0 / np.where(g_ok, var_g, 1.0), 0.0)
    w_s = np.where(s_ok, 1.0 / np.where(s_ok, var_s, 1.0), 0.0)
    w = w_g + w_s
    valid = w > 0.0
    with np.errstate(divide="ignore", invalid="ignore"):
        r = (w_g * np.where(g_ok, r_g, 0.0) + w_s * np.where(s_ok, r_s, 0.0)) / w
        var_r = 1.0 / w

    r_c = r + p.center_offset_m
    var_r = var_r + p.center_offset_sigma_m ** 2
    var_b = (p.box_sigma_px / cam.fx) ** 2

    cb, sb = np.cos(bearing), np.sin(bearing)
    xy = np.stack([cam.mount_x_m + r_c * cb, cam.mount_y_m + r_c * sb], axis=-1)
    # Polar -> Cartesian: J diag(var_r, var_b) J^T, J = [[cb, -r sb], [sb, r cb]]
    cov = np.empty((m, 2, 2))
    rb2 = r_c * r_c * var_b
    cov[:, 0, 0] = cb * cb * var_r + sb * sb * rb2
    cov[:, 1, 1] = sb * sb * var_r + cb * cb * rb2
    cov[:, 0, 1] = cov[:, 1, 0] = cb * sb * (var_r - rb2)

    xy[~valid] = np.nan
    cov[~valid] = np.nan
    return RangeBatch(
        xy=xy, cov=cov, range_m=np.where(valid, r_c, np.nan), bearing_rad=bearing,
        used_ground=g_ok, used_size=s_ok, valid=valid,
    )


def body_to_world(
    xy_body: np.ndarray,
    cov_body: np.ndarray,
    ego_xy: Tuple[float, float],
    ego_yaw: float,
    ego_xy_sigma_m: float = 0.0,
    ego_yaw_sigma_rad: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Rotate/translate body-frame points into the map frame, carrying ego
    pose uncertainty into the covariance (yaw error grows with range)."""
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    R = np.array([[c, -s], [s, c]])
    dR = np.array([[-s, -c], [c, -s]])  # dR/dyaw
    xy = np.asarray(xy_body, dtype=np.float64).reshape(-1, 2)
    world = xy @ R.T + np.asarray(ego_xy, dtype=np.float64)
    cov = R @ np.asarray(cov_body, dtype=np.float64).reshape(-1, 2, 2) @ R.T
    if ego_yaw_sigma_rad:
        j = xy @ dR.T  # (M, 2)
        cov = cov + ego_yaw_sigma_rad ** 2 * j[:, :, None] * j[:, None, :]
    if ego_xy_sigma_m:
        cov = cov + ego_xy_sigma_m ** 2 * np.eye(2)
    return world, cov


def project_ground_point(cam: CameraModel, x_body: np.ndarray, y_body: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Body-frame ground point(s) -> pixel (u, v). Inverse of the ground cue;
    used for debug overlays and tests. Ignores distortion."""
    x = np.asarray(x_body, dtype=np.float64) - cam.mount_x_m
    y = np.asarray(y_body, dtype=np.float64) - cam.mount_y_m
    z = np.full_like(x, -cam.height_m)
    if cam.yaw_rad:
        cy, sy = math.cos(cam.yaw_rad), math.sin(cam.yaw_rad)
        x, y = cy * x + sy * y, -sy * x + cy * y
    c, s = math.cos(cam.pitch_rad), math.sin(cam.pitch_rad)
    # Undo pitch (R_y(theta)^T), then body -> OpenCV camera axes
    fx_ = c * x - s * z
    fz_ = s * x + c * z
    zc, xc, yc = fx_, -y, -fz_
    return cam.fx * xc / zc + cam.cx, cam.fy * yc / zc + cam.cy
