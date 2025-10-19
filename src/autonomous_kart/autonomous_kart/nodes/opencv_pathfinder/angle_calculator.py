from typing import Tuple
import cv2


def calculate_track_angles(frame: cv2.Mat) -> Tuple[float, float]:
    """
    Calculates angle from center of image to left/right side of track with OpenCV efficiently.
    Part of hot loop so efficiency is very important.
    TODO(OpenCV Team): Calculate angles from frame and return them
    :param frame: Most recent image from camera
    :return: Tuple of [left angle (degrees), right angle (degrees)]. For example, return 75.0, 75.0
    """
    return 75.0, 75.0  # dummy