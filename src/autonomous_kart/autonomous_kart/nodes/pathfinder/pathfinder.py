import math
from typing import Tuple


# Copied from nav2 https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller
def pathfinder(
        current_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        yaw_rad: float,
        speed_mps: float,
        *,
        wheelbase_m: float,
        v_max_mps: float,
        steer_max_deg: float,

        # geometry in meters/seconds, speed in m/s
        desired_speed_mps: float = 1.0,  # like desired_linear_vel
        use_velocity_scaled_lookahead: bool = True,
        lookahead_time_s: float = 1.0,  # like lookahead_time
        min_lookahead_m: float = 0.6,  # like min_lookahead_dist
        max_lookahead_m: float = 3.0,  # like max_lookahead_dist

        # Regulated linear velocity scaling by curvature
        use_curvature_regulation: bool = True,  # like use_regulated_linear_velocity_scaling
        min_radius_m: float = 2.0,  # like regulated_linear_scaling_min_radius
        min_reg_speed_mps: float = 4.0,  # like regulated_linear_scaling_min_speed

        # Slow down as you run out of path (approx using distance to target)
        approach_dist_m: float = 1.0,  # like approach_velocity_scaling_dist
        min_approach_speed_mps: float = 1.0  # like min_approach_linear_velocity
) -> Tuple[float, float]:
    """
    Nav2-style Regulated Pure Pursuit (RPP)

    Inputs:
      current_xy: (x, y) in meters, world/map frame
      target_xy:  (x, y) lookahead ("carrot") point in meters, world/map frame
      yaw_rad:    vehicle heading (yaw) in radians, world/map frame
      speed_mps:  current speed (m/s)

    Outputs:
      (throttle_mps, steering_deg)
        throttle_mps: commanded speed in m/s, clamped to [0, v_max_mps]
        steering_deg: front-wheel angle in degrees (left=positive)
    """

    # Constraints
    if v_max_mps <= 0.0:
        return 0.0, 0.0
    speed_mps = 0.0 if speed_mps < 0.0 else (v_max_mps if speed_mps > v_max_mps else speed_mps)
    desired_speed_mps = 0.0 if desired_speed_mps < 0.0 else (v_max_mps if desired_speed_mps > v_max_mps else desired_speed_mps)

    steer_max_rad = math.radians(steer_max_deg)
    if steer_max_rad <= 1e-6 or wheelbase_m <= 1e-6:
        return 0.0, 0.0

    v_des = desired_speed_mps
    v_cur = speed_mps

    # Lookahead distance (Nav2: velocity-scaled lookahead)
    if use_velocity_scaled_lookahead:
        Ld = v_cur * lookahead_time_s  # Lt = vt * lt
        if Ld < min_lookahead_m:
            Ld = min_lookahead_m
        elif Ld > max_lookahead_m:
            Ld = max_lookahead_m
    else:
        # If turned off, min/max still provide reasonable bounds
        Ld = min_lookahead_m

    # Change to vehicle frame
    dx = target_xy[0] - current_xy[0]
    dy = target_xy[1] - current_xy[1]
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)

    # vehicle frame: x forward, y left
    x_v = cy * dx + sy * dy
    y_v = -sy * dx + cy * dy

    # If target is behind, can't pure pursue so creep + turn hard toward it.
    if x_v <= 1e-6:
        steering_deg = -steer_max_deg if y_v < 0.0 else steer_max_deg
        return min(0.10 * v_max_mps, desired_speed_mps), steering_deg

    # Use actual distance to target as L (pure pursuit chord length).
    # If target is actually a lookahead point, this will be close to Ld regardless
    L = math.hypot(x_v, y_v)
    if L < 1e-6:
        return 0.0, 0.0

    # Pure Pursuit curvature (Nav2 paper Eq. 3)
    kappa = (2.0 * y_v) / (L * L)

    # Regulate linear speed by curvature (Nav2/RPP)
    v_cmd = v_des

    if use_curvature_regulation and min_radius_m > 1e-6:
        # Trigger when radius < min_radius  <=>  abs(kappa) > 1/min_radius
        abs_k = abs(kappa)
        k_thresh = 1.0 / min_radius_m
        if abs_k > k_thresh:
            # Scale by radius/min_radius = (1/abs(kappa))/min_radius = 1/(min_radius*abs(kappa))
            v_cmd = v_des / (min_radius_m * abs_k)

        # Enforce minimum regulated speed
        if v_cmd < min_reg_speed_mps:
            v_cmd = min_reg_speed_mps
        if v_cmd > v_des:
            v_cmd = v_des

    # Slow down when approaching end-of-path (simplified with distance-to-target)
    # Nav2 does this using integrated distance near end of transformed path; we approximate with L.
    if approach_dist_m > 1e-6 and L < approach_dist_m:
        # Linear ramp: at L=approach_dist -> 1.0, at L=0 -> min_approach
        ramp = L / approach_dist_m
        v_cmd = max(min_approach_speed_mps, v_cmd * ramp)

    # Curvature to steering w/ bicycle model
    # For bicycle model: R = L/tan(delta), yaw_rate = V/R => tan(delta) = wheelbase * kappa
    delta = math.atan(wheelbase_m * kappa)

    # Clamp steering to rack limits
    if delta > steer_max_rad:
        delta = steer_max_rad
    elif delta < -steer_max_rad:
        delta = -steer_max_rad

    # Final clamps
    if v_cmd < 0.0:
        v_cmd = 0.0
    elif v_cmd > v_max_mps:
        v_cmd = v_max_mps

    return v_cmd, math.degrees(delta)


