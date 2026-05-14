from typing import Optional, Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, Planner, PlannerInputs


class MPCPlanner(Planner):
    """Model predictive controller over the racing line"""

    name = "mpc"

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None):
        super().__init__(params, kart, racing_line, logger)
        raise NotImplementedError("MPCPlanner is not implemented yet")

    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        return 0.0, 0.0
