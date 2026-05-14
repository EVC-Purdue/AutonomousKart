from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs


class OpenCVPlanner(Planner):
    """Camera-only planner: drives solely from the OpenCV
    pathfinder's edge angles"""

    name = "opencv"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        super().__init__(params, kart, racing_line, logger, node=node)
        raise NotImplementedError("OpenCVPlanner is not implemented yet")

    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        return 0.0, 0.0
