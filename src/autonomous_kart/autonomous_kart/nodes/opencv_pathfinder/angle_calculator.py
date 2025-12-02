from typing import Tuple
import cv2
import time

_start_time = None

def calculate_track_angles(frame: cv2.Mat) -> Tuple[float, float]:
    """
    Calculates angle from center of image to left/right side of track with OpenCV efficiently.
    Part of hot loop so efficiency is very important.
    TODO(OpenCV Team): Calculate angles from frame and return them
    :param frame: Most recent image from camera
    :return: Tuple of [left angle (degrees), right angle (degrees)]. For example, return 75.0, 75.0
    """

    '''
    DUMMY CODE FOR SIMULATING OPENCV FAILURE BELOW
    '''
    global _start_time

    if _start_time is None:
        _start_time = time.time()
    
    elapsed = time.time() - _start_time
    
    # normal angles for first 5 seconds
    if elapsed < 5.0:
        # variation for realism!!
        left_angle = 70.0 + (elapsed * 2)  
        right_angle = 80.0 - (elapsed * 1)
        return left_angle, right_angle
    # return infinity (simulating detection failure)
    else:
        return float('inf'), float('inf')
    
    '''
    OG DUMMY CODE:
    return 75.0, 75.0  # dummy
    '''