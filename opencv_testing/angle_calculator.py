"""
Compute theta_left and theta_right from a video.
Requirements: opencv-python (cv2), numpy
Usage: tweak CAMERA_FX or HFOV_HORIZONTAL to match your camera.
"""

import cv2
import numpy as np
import math

# ----------------- CONFIG -----------------
VIDEO_PATH = "IMG_8824.mp4"   # change to your file or use 0 for webcam
USE_CALIBRATION = False           # True if you have fx, cx from calibration
CAMERA_FX = 800.0                 # focal length in pixels (only if USE_CALIBRATION)
CAMERA_CX = None                  # principal point x; if None -> image_width/2
HFOV_DEG = 70.0                   # horizontal field of view (deg) if no fx available

SMOOTH_ALPHA = 0.7                # for low-pass filtering of theta
CANNY_THRESH1 = 50
CANNY_THRESH2 = 150

# Ray sampling parameters
MAX_SAMPLE_DIST = 400             # max pixels to scan along each ray
FORWARD_STEP = 1                  # pixels per sample along forward ray
RIGHT_STEP = 1                    # pixels per sample along right ray

# -----------------------------------------

def compute_fx_from_hfov(width, hfov_deg):
    hfov = math.radians(hfov_deg)
    return (width / 2.0) / math.tan(hfov / 2.0)

def pixel_to_angle(u, fx, cx):
    # returns angle in degrees, negative = left of center
    return math.degrees(math.atan2((u - cx), fx))

def find_intersection_along_column(mask, col, start_row, step=1, max_dist=400):
    """
    Scan downwards along column 'col' starting at start_row (row index),
    return (u,v) of first mask nonzero pixel, or None if none found.
    """
    h, w = mask.shape
    row = start_row
    dist = 0
    while dist < max_dist and 0 <= row < h:
        if mask[row, col]:
            return col, row
        row += step
        dist += abs(step)
    return None

def find_intersection_along_row(mask, row, start_col, step=1, max_dist=400):
    """
    Scan rightwards along row 'row' starting at start_col, return first mask hit.
    """
    h, w = mask.shape
    col = start_col
    dist = 0
    while dist < max_dist and 0 <= col < w:
        if mask[row, col]:
            return col, row
        col += step
        dist += abs(step)
    return None

def main():
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("Cannot open video:", VIDEO_PATH)
        return

    theta_left_f = None
    theta_right_f = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        cx = CAMERA_CX if CAMERA_CX is not None else w / 2.0
        if USE_CALIBRATION:
            fx = CAMERA_FX
        else:
            fx = compute_fx_from_hfov(w, HFOV_DEG)

        # 1) preprocess and boundary mask
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, CANNY_THRESH1, CANNY_THRESH2)
        # optional morphology to fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # 2) define rays: use image center
        center_col = int(round(cx))
        center_row = int(round(h / 2))

        # forward ray: downwards from center_row toward bottom (increasing row)
        f_hit = find_intersection_along_column(mask, center_col, center_row, step=FORWARD_STEP, max_dist=MAX_SAMPLE_DIST)

        # right ray: from center out to the right along center_row
        r_hit = find_intersection_along_row(mask, center_row, center_col, step=RIGHT_STEP, max_dist=MAX_SAMPLE_DIST)

        # convert hits to angles (default to None/confidence if not found)
        theta_left = None
        theta_right = None

        if f_hit is not None:
            u_f, v_f = f_hit
            theta_forward = pixel_to_angle(u_f, fx, cx)  # bearing of forward intersection
            # If the forward ray hits boundary on right side of center, we can treat that as right boundary?
            # However your definition: theta_left is angle from left boundary to center — we approximate:
            # We'll set theta_left to the bearing of the left-side boundary detected forward of center,
            # but since forward ray is center column, use it as a nearby boundary reading if needed.
            # For consistent approach: treat forward ray hit on its x location to compute whichever boundary that represents.
            if u_f < cx:
                theta_left = pixel_to_angle(u_f, fx, cx)
            else:
                theta_right = pixel_to_angle(u_f, fx, cx)

        if r_hit is not None:
            u_r, v_r = r_hit
            # point to the right of center -> this is likely the right boundary
            theta_right = pixel_to_angle(u_r, fx, cx)

        # If we failed to find one of them from the rays, try alternate strategy:
        # find contours and pick nearest contour point to the ray direction (omitted for brevity),
        # or fallback to last frame value.

        # fallback: if missing, reuse previous smoothed value or compute from any contour
        if theta_left is None and theta_left_f is not None:
            theta_left = theta_left_f
        if theta_right is None and theta_right_f is not None:
            theta_right = theta_right_f

        # smoothing
        if theta_left is not None:
            theta_left_f = theta_left if theta_left_f is None else (SMOOTH_ALPHA * theta_left_f + (1-SMOOTH_ALPHA)*theta_left)
        if theta_right is not None:
            theta_right_f = theta_right if theta_right_f is None else (SMOOTH_ALPHA * theta_right_f + (1-SMOOTH_ALPHA)*theta_right)

        # compute desired heading if both exist (or using whichever exists)
        desired_heading = None
        if (theta_left_f is not None) and (theta_right_f is not None):
            desired_heading = 0.5 * (theta_left_f + theta_right_f)
        elif theta_left_f is not None:
            desired_heading = theta_left_f  # single-side fallback
        elif theta_right_f is not None:
            desired_heading = theta_right_f

        # Display overlay for debugging
        vis = frame.copy()
        # draw rays
        cv2.line(vis, (center_col, center_row), (center_col, min(h, center_row + MAX_SAMPLE_DIST)), (0,255,0), 1)
        cv2.line(vis, (center_col, center_row), (min(w-1, center_col + MAX_SAMPLE_DIST), center_row), (0,255,0), 1)
        if f_hit is not None:
            cv2.circle(vis, (f_hit[0], f_hit[1]), 6, (0,0,255), -1)
        if r_hit is not None:
            cv2.circle(vis, (r_hit[0], r_hit[1]), 6, (255,0,0), -1)

        # text
        info = [
            f"theta_left_f: {theta_left_f:.2f}" if theta_left_f is not None else "theta_left_f: N/A",
            f"theta_right_f: {theta_right_f:.2f}" if theta_right_f is not None else "theta_right_f: N/A",
            f"desired_heading: {desired_heading:.2f}" if desired_heading is not None else "desired_heading: N/A"
        ]
        y = 30
        for line in info:
            cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            y += 25

        cv2.imshow("vis", vis)
        cv2.imshow("mask", mask)

        # press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
