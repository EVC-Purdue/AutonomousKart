from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs


class OpenCVPlanner(Planner):
    """Camera-only planner: drives solely from the OpenCV
    pathfinder's edge angles"""

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)
        prev_left_delta = None
        prev_right_delta = None
        
    def compute_deltas(self, angles):
        left_delta = angles[0] 
        

    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        return 0.0, 0.0
