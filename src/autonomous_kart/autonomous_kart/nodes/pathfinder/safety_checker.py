import math
from typing import Tuple

from autonomous_kart.nodes.pathfinder.planners.base import KartConstants, PlannerInputs

MODES = ("camera_only", "hard_override", "blend")


class SafetyChecker:
    """Single safety chokepoint between the active planner and cmd_drive.

    Reacts to `track_angles` (right, left) from the CV pathfinder: each is
    the angle, at the ROI's top-center point, between the ray to that side's
    top corner and the ray to the CV-detected concrete/grass divide point.
    ~0deg means that side's divide sits out near the corner (wide-open
    road); the angle grows as the divide point swings in toward the
    vehicle's path (that side's margin narrowing). So a larger angle means
    that side's boundary is closer/encroaching, and the worse (larger) of
    the two sides is the one to steer away from.

    Positive steering_deg = left, negative = right (matches the MPC's own
    "d is signed left positive" convention).
    """

    def __init__(self, params: dict, kart: KartConstants):
        self.params = params
        self.kart = kart
        self.mode = str(params.get("mode", "blend"))
        if self.mode not in MODES:
            self.mode = "blend"
        # Latest decision, for safety/status telemetry (pathfinder_node reads
        # this on its own timer rather than every tick).
        self.last_status = {
            "mode": self.mode, "enabled": True,
            "right_angle": math.nan, "left_angle": math.nan,
            "worst": math.nan, "engaged": False, "blend_t": 0.0,
        }

    def set_mode(self, mode: str) -> bool:
        if mode not in MODES:
            return False
        self.mode = mode
        return True

    def check(
        self,
        proposed: Tuple[float, float],
        inputs: PlannerInputs,
    ) -> Tuple[float, float]:
        enabled = bool(self.params.get("enabled", True))
        if not enabled:
            self.last_status = {
                "mode": self.mode, "enabled": False,
                "right_angle": math.nan, "left_angle": math.nan,
                "worst": math.nan, "engaged": False, "blend_t": 0.0,
            }
            return proposed

        angles = inputs.track_angles
        right = left = math.nan
        if angles is not None and len(angles) >= 2:
            right, left = angles[0], angles[1]

        if math.isnan(right) or math.isnan(left):
            self.last_status = {
                "mode": self.mode, "enabled": True,
                "right_angle": right, "left_angle": left,
                "worst": math.nan, "engaged": False, "blend_t": 0.0,
            }
            return proposed

        proposed_speed, proposed_steer = proposed
        worst = max(right, left)
        sign = 1.0 if right >= left else -1.0

        warn = float(self.params.get("warn_angle_deg", 35.0))
        critical = float(self.params.get("critical_angle_deg", 55.0))
        max_correction = float(self.params.get("max_correction_deg", 20.0))
        floor_speed = float(self.params.get("min_speed_at_critical_mps", 2.0))

        if self.mode == "camera_only":
            gain = float(self.params.get("camera_steer_gain", 1.0))
            speed = float(self.params.get("camera_only_speed_mps", 2.0))
            steer = sign * gain * (worst - min(right, left))
            result = self._clamp(speed, steer)
            self._record(right, left, worst, engaged=True, blend_t=1.0)
            return result

        if self.mode == "hard_override":
            if worst < critical:
                self._record(right, left, worst, engaged=False, blend_t=0.0)
                return self._clamp(proposed_speed, proposed_steer)
            result = self._clamp(floor_speed, sign * max_correction)
            self._record(right, left, worst, engaged=True, blend_t=1.0)
            return result

        # blend
        span = critical - warn
        t = 0.0 if span <= 0 else (worst - warn) / span
        t = min(1.0, max(0.0, t))
        corrective_steer = sign * max_correction
        steer = (1.0 - t) * proposed_steer + t * corrective_steer
        speed = (1.0 - t) * proposed_speed + t * floor_speed
        result = self._clamp(speed, steer)
        self._record(right, left, worst, engaged=(t > 0.0), blend_t=t)
        return result

    def _record(self, right: float, left: float, worst: float,
                engaged: bool, blend_t: float) -> None:
        self.last_status = {
            "mode": self.mode, "enabled": True,
            "right_angle": right, "left_angle": left,
            "worst": worst, "engaged": engaged, "blend_t": blend_t,
        }

    def _clamp(self, speed: float, steer: float) -> Tuple[float, float]:
        speed = min(self.kart.v_max_mps, max(0.0, speed))
        steer = min(self.kart.steer_max_deg, max(-self.kart.steer_max_deg, steer))
        return speed, steer
