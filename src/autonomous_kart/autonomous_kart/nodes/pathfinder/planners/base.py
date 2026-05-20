from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import nan
from typing import ClassVar, Optional, Tuple


@dataclass(frozen=True)
class KartConstants:
    v_max_mps: float
    wheelbase_m: float
    steer_max_deg: float
    steer_rate_max_degps: float
    a_max_mps2: float
    a_min_mps2: float
    a_lat_max_mps2: float


@dataclass(frozen=True)
class PlannerInputs:
    pose_xy: Tuple[float, float]
    yaw_rad: float
    speed_mps: float
    track_angles: Optional[Tuple[float, ...]]
    now_ns: int
    # Raw GPS pose passthrough for EKF-vs-GPS comparison in mpc/status.
    # NaN when no GPS fix has been seen yet.
    gps_xy: Tuple[float, float] = (nan, nan)
    gps_yaw_rad: float = nan
    gps_speed_mps: float = nan
    # Latest VESC wheel-speed (m/s). NaN until first /e_comms/kart_speed msg.
    wheel_speed_mps: float = nan
    # Latest /localization/gps_event snapshot
    gps_event_seq: float = 0.0
    gps_have_yaw: float = 0.0
    gps_have_speed: float = 0.0
    ekf_prior_xy: Tuple[float, float] = (nan, nan)
    ekf_prior_yaw_rad: float = nan
    ekf_prior_v: float = nan
    ekf_post_xy: Tuple[float, float] = (nan, nan)
    ekf_post_yaw_rad: float = nan
    ekf_post_v: float = nan


class Planner(ABC):
    """Base class for top-level driving planners.

    Each planner gets the same inputs (PlannerInputs) plus its own param block
    from pathfinder.yaml. The node calls plan() each tick and wraps the result
    in the SafetyChecker before publishing on cmd_drive.
    """

    name: ClassVar[str]

    def __init__(self, params: dict, kart: KartConstants, racing_line: list, logger=None, node=None):
        self.params = params
        self.kart = kart
        self.racing_line = racing_line
        self.logger = logger
        self.node = node # To publish mpc/status

    @abstractmethod
    def plan(self, inputs: PlannerInputs) -> Optional[Tuple[float, float]]:
        """Return (throttle_mps, steering_deg) or None to skip publishing this tick."""
        ...

    def dynamic_line_state(self) -> Optional[dict]:
        """Optional telemetry hook for planners for the dynamic line overlay in frontend"""
        return None
