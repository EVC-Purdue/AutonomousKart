import math
from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs


class OpenCVPlanner(Planner):
    """Camera-only planner: drives solely from the OpenCV pathfinder's edge
    angles (track_angles = (right_angle_deg, left_angle_deg), published by
    opencv_pathfinder_node). No odom/racing-line input.

    Steering is a P controller on (right_angle - left_angle), zero when the
    two edges are symmetric (kart centered on the track). Sign/magnitude is
    set by `steer_gain_deg_per_deg` -- flip its sign on the bench if a
    correction steers the wrong way rather than editing this file. Throttle
    is a fixed cruise percentage while track_angles keeps arriving; the kart
    stops once `max_missing_ticks` consecutive readings come back NaN (lost
    line / lost camera feed).
    """

    name = "opencv"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)

        self.cruise_speed_pct = float(params.get("cruise_speed_pct", 20.0))
        self.steer_gain_deg_per_deg = float(params.get("steer_gain_deg_per_deg", 1.0))
        self.angle_deadband_deg = float(params.get("angle_deadband_deg", 1.0))
        self.max_missing_ticks = int(params.get("max_missing_ticks", 5))

        self.missing_ticks = 0

    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        angles = inputs.track_angles
        valid = angles is not None and not (math.isnan(angles[0]) or math.isnan(angles[1]))

        if not valid:
            self.missing_ticks += 1
            if self.missing_ticks >= self.max_missing_ticks:
                return 0.0, 0.0
            return None  # brief dropout: hold last command instead of jerking to a stop

        self.missing_ticks = 0
        right_angle, left_angle = angles[0], angles[1]

        error_deg = right_angle - left_angle
        if abs(error_deg) < self.angle_deadband_deg:
            error_deg = 0.0

        steer_max_deg = self.kart.steer_max_deg
        steering_deg = max(-steer_max_deg, min(steer_max_deg, error_deg * self.steer_gain_deg_per_deg))

        return self.cruise_speed_pct, steering_deg
