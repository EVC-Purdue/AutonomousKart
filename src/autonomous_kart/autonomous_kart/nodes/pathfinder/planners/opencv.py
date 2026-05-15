import math

from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs

class OpenCVPlanner(Planner):
    """Camera-only planner: drives solely from the OpenCV
    pathfinder's edge angles"""

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)
        self.prev_angles = []
        self.angle_threshold = params.get("angle_threshold", 0.25)
        self.max_turning_angle = params.get("max_turning_angle", 45.0)
    
    def compute_delta_angle(self, cur_angles):
        left_delta: float = cur_angles[0] - self.prev_angles[0]
        right_delta: float = cur_angles[1] - self.prev_angles[1]

        angle_delta: float = right_delta - left_delta

        steering = angle_delta / self.max_turning_angle

        return max(-1.0, min(steering, 1.0))

    def compute_angle(self, cur_angles):
        angle: float = cur_angles[0] - cur_angles[1]

        steering = angle / self.max_turning_angle

        return max(-1.0, min(steering, 1.0))
    
    def check_threshold(self, steering):
        pass
        
    
    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        return 0.0, 0.0
