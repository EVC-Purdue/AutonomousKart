import random
from typing import Tuple


def pathfinder(opencv_output: Tuple):
    """
    Calculate commands for steering and motor from opencv_pathfinder efficiently
    Part of hot loop so must be efficient.
    TODO(Pathfinder Team): Calculate motor_speed & steering_angle
    :param opencv_output: Tuple of [left angle from center to base of track from image (float32), right angle ...]
    :return: Returns commands to motor & steering in (speed % of total, steering angle (degrees from -90 to 90 with 0 as straight)
    """

    # Dummy
    motor_speed = 20 * random.random()
    steering_angle = 180 * random.random() - 90

    return motor_speed, steering_angle