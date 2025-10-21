from typing import Tuple
import cv2
import numpy as np

def calculate_track_angles(frame: cv2.Mat) -> Tuple[float, float]:
    """
    Calculates angle from center of image to left/right side of track with OpenCV efficiently.
    Part of hot loop so efficiency is very important.
    :param frame: Most recent image from camera
    :return: Tuple of [left angle (degrees), right angle (degrees)]. For example, return 75.0, 75.0
    """
    frame = frame[2 * frame.shape[0] // 3:, :]
    display_h, display_w = frame.shape[:2]
    frame_small = cv2.resize(frame, (display_w // 2, display_h // 2), interpolation=cv2.INTER_AREA)

    hsv = cv2.cvtColor(frame_small, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, np.array([20, 40, 40]), np.array([80, 255, 255]))

    h, w = frame_small.shape[:2]
    center_x = w // 2
    bottom_point_y = h - 1

    left_hit_y = None
    for y in range(h - 1, -1, -1):
        if green_mask[y, 0] != 0:
            left_hit_y = y
            break

    right_hit_y = None
    for y in range(h - 1, -1, -1):
        if green_mask[y, w - 1] != 0:
            right_hit_y = y
            break

    angle_left = float('inf')
    if left_hit_y is not None:
        dx = -center_x
        dy = bottom_point_y - left_hit_y
        angle_left = 90 - np.degrees(np.arctan2(dy * 3, -dx))

    angle_right = float('inf')
    if right_hit_y is not None:
        dx = (w - 1) - center_x
        dy = bottom_point_y - right_hit_y
        angle_right = np.degrees(np.arctan2(dy * 3, -dx)) - 90

    return angle_left, angle_right


