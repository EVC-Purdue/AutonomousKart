import math

from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs

class OpenCVPlanner(Planner):
    """Camera-only planner: drives solely from the OpenCV
    pathfinder's edge angles"""

    name = "opencv"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)
        self.prev_angles = None
        self.angle_threshold = float(params.get("angle_threshold", 5.0))
        self.steer_max_deg = float(params.get("steer_max_deg", 25.0))
        self.throttle_mps= float(params.get("throttle_mps", 5.0))
        self.mode = params.get("mode", "absolute")

    def compute_angle(self, cur_angles):
        angle_dif: float = cur_angles[0] - cur_angles[1]
        self.prev_angles = angle_dif

        kp = 3.5

        if angle_dif <= self.angle_threshold and angle_dif > 1e-6:
            return self.prev_angles

        return angle_dif / kp
        
    
    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        if inputs is None or inputs.track_angles is None or len(inputs.track_angles) < 2:
            self.logger.warning("Inputs or inputs.track_angles is invalid")
            return 0.0, 0.0
        
        self.logger.info(f"{inputs.track_angles}")
        
        steering_deg = self.compute_angle(inputs.track_angles)
    
        if steering_deg is None:
            self.logger.warning("Degrees Was Never Changed")
            steering_deg = 0.0

        return self.throttle_mps, steering_deg
