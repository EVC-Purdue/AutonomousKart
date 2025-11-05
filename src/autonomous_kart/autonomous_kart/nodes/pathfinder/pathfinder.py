from typing import Tuple
from std_msgs.msg import Float32

def pathfinder(opencv_output: Tuple, current_speed: Float32, dt: Float32, max_accel:Float32, max_steering: Float32, max_speed_straight: Float32, max_speed_turning: Float32, max_speed: Float32, logger):
    """
    Calculate commands for steering and motor from opencv_pathfinder efficiently
    Part of hot loop so must be efficient.
    :param opencv_output: Tuple of [left angle from center to base of track from image (float32), right angle ...]
    :param current_speed: Current actual speed from motor node (% of total)
    :param dt: Time elapsed time between last calculated speed command and current
    :param max_accel: Max allowed acceleration of kart (m/s)
    :param  max_steering: Max steering angle (degrees)
    :param max_speed_straight: Max allowed speed on straights (m/s)
    :param max_speed_turning: Max allowed speed while turning (m/s)
    :param max_speed: Max possible speed (m/s)
    :param logger: Allows for logging info
    :return: Returns commands to motor & steering in (speed % of total, steering angle (degrees from -90 to 90 with 0 as straight)
    """

    # convert current speed from motor node to m/s
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
    logger.info(f"dt {dt:.3f}")

    

    return float(speed_command), float(steering_command)
