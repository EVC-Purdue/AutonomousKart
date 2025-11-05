import random
from typing import Tuple


def pathfinder(opencv_output: Tuple, current_speed: float, dt, logger):
    """
    Calculate commands for steering and motor from opencv_pathfinder efficiently
    Part of hot loop so must be efficient.
    :param opencv_output: Tuple of [left angle from center to base of track from image (float32), right angle ...]
    :param current_speed: current speed from motor node
    :return: Returns commands to motor & steering in (speed % of total, steering angle (degrees from -90 to 90 with 0 as straight)
    """

    max_accel = 3.0  # mph per second
    max_steering = 25  # degrees
    max_speed_straight = 30.0 # max speed on straights (mph)
    max_speed_turning = 15.0 # max speed while completing turns (mph)
    max_speed = 35.0 # max speed car is capable of going (mph) (for calculating speed % of total)

    # convert current speed from motor node (presumably also a % of total?) to mph for calculations
    current_speed *= max_speed

    speed_command = current_speed

    theta1 = -1 * opencv_output[0]
    theta2 = opencv_output[1]

    desired_heading = (theta1 + theta2) / 2 #average of both angles

    if abs(desired_heading) < 10:
            target_speed = max_speed_straight  #max speed on straights
    else:
        target_speed = max_speed_turning  #target speed on turns

    if current_speed < target_speed:
        speed_command += max_accel * dt #physics c: mechanics
        if speed_command > target_speed:
            speed_command = target_speed
    else:
        speed_command -= max_accel * dt
        if speed_command < target_speed:
            speed_command = target_speed

    steering_command = max(min(desired_heading, max_steering), -max_steering)

    # calculate speed command as % of total 
    speed_command /= max_speed

    logger.info(f"Theta1: {theta1:.1f}°, Theta2: {theta2:.1f}°")
    logger.info(f"Desired Heading: {desired_heading:.1f}°")
    logger.info(f"Target Speed: {target_speed} mph | Current Speed: {current_speed:.1f} mph | Commanded Speed: {speed_command:.3f} of total")
    logger.info(f"Steering Command: {steering_command:.1f}° {'Left' if steering_command < 0 else 'Right' if steering_command > 0 else 'Straight'}")

    

    return float(speed_command), float(steering_command)
