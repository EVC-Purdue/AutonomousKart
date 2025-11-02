import time
import os

speed = 0.0  # mph
max_accel = 3.0  # mph per second
max_steering = 90  # degrees
time_step = 1  # seconds

theta1 = -5
theta2 = 5

for t in range(1, 31): 
    if t <= 10: 
        theta1 = -5
        theta2 = 5
    elif 10 < t <= 20:  
        theta1 -= 2  
        theta2 += 0.5
    elif 20 < t <= 30:  
        theta1 += 2  
        theta2 += 2

    desired_heading = (theta1 + theta2) / 2 #average of both angles

    if abs(desired_heading) < 10:
        target_speed = 30  #max speed on straights
    else:
        target_speed = 15  #target speed on turns

    if speed < target_speed:
        speed += max_accel * time_step #physics c: mechanics
        if speed > target_speed:
            speed = target_speed
    else:
        speed -= max_accel * time_step
        if speed < target_speed:
            speed = target_speed

    steering_command = max(min(desired_heading, max_steering), -max_steering)

    os.system('cls' if os.name == 'nt' else 'clear')

    print(f"Time: {t} sec")
    print(f"Theta1: {theta1:.1f}°, Theta2: {theta2:.1f}°")
    print(f"Desired Heading: {desired_heading:.1f}°")
    print(f"Target Speed: {target_speed} mph | Current Speed: {speed:.1f} mph")
    print(f"Steering Command: {steering_command:.1f}° {'Left' if steering_command < 0 else 'Right' if steering_command > 0 else 'Straight'}")

    time.sleep(1)

