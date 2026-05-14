from typing import Tuple

from autonomous_kart.nodes.pathfinder.planners.base import PlannerInputs


class SafetyChecker:
    """Single safety chokepoint between the active planner and cmd_drive."""

    def __init__(self, params: dict):
        self.params = params

    def check(
        self,
        proposed: Tuple[float, float],
        inputs: PlannerInputs,
    ) -> Tuple[float, float]:
        return proposed
