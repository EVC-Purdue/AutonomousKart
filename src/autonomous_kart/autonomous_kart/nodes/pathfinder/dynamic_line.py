"""
Dynamic line generation framework.

DynamicLineManager sits between the node and the racing line.
It checks registered strategies each tick. If any strategy activates,
its generated line is used for Pure Pursuit instead of the racing line.

To add a new strategy:
  1. Subclass LineStrategy
  2. Implement should_activate() and generate()
  3. Register it with the manager via manager.register(strategy)

"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.impl.rcutils_logger import RcutilsLogger


@dataclass
class KartState:
    """Snapshot of kart state passed to strategies each tick"""
    xy: Tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    speed_mps: float = 0.0
    closest_idx: int = 0
    cross_track_error: float = 0.0
    extra: dict = field(default_factory=dict)


# A waypoint row matching your racing_line format: (s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps)
WaypointRow = tuple


class LineStrategy(ABC):
    """
    Base class for dynamic line strategies.

    Subclass this and implement:
      - should_activate(state, racing_line) -> bool
      - generate(state, racing_line) -> (waypoints, merge_idx)
      - should_deactivate(state, racing_line) -> bool  (optional override)
      - priority  (lower = higher priority, default 100)
    """

    priority: int = 100  # Lower number = higher priority

    @abstractmethod
    def should_activate(self, state: KartState, racing_line: list) -> bool:
        """Return True if this strategy should take over line generation."""
        ...

    @abstractmethod
    def generate(
            self, state: KartState, racing_line: list
    ) -> Optional[Tuple[List[WaypointRow], int]]:
        """
        Generate a dynamic line.

        Returns:
            (waypoints, merge_idx) where:
              - waypoints: list of tuples matching racing_line row format
              - merge_idx: index on the original racing line where this line ends
            or None if generation fails (manager falls back to racing line).
        """
        ...

    def should_deactivate(self, state: KartState, racing_line: list) -> bool:
        """Return True if the strategy should hand control back. Override as needed."""
        return False

    def on_activate(self, state: KartState) -> None:
        """Called once when strategy becomes active. Override for setup/logging."""
        pass

    def on_deactivate(self, state: KartState) -> None:
        """Called once when strategy is deactivated. Override for cleanup."""
        pass


class DynamicLineManager:
    """
    Manages strategy selection and active dynamic line state.
    The node calls update() each tick and uses get_line() for Pure Pursuit.
    """

    def __init__(self, racing_line: list, logger: Optional["RcutilsLogger"] = None):
        self.racing_line = racing_line
        self.logger = logger
        self._strategies: List[LineStrategy] = []

        # Active dynamic line state
        self.active_strategy: Optional[LineStrategy] = None
        self.dynamic_line: List[WaypointRow] = []
        self.dynamic_closest_idx: int = 0
        self.merge_idx: int = 0

    def register(self, strategy: LineStrategy) -> None:
        """Register a strategy. Strategies are checked in priority order."""
        self._strategies.append(strategy)
        self._strategies.sort(key=lambda s: s.priority)

    def update(self, state: KartState) -> None:
        """
        Call each control tick. Checks for activation/deactivation of strategies.
        """

        # If a strategy is active, check if it should deactivate
        if self.active_strategy is not None:
            end_of_line = self.dynamic_closest_idx >= len(self.dynamic_line) - 5
            should_deactivate = (
                    end_of_line
                    or self.active_strategy.should_deactivate(state, self.racing_line)
            )
            if should_deactivate:
                if self.logger:
                    name = type(self.active_strategy).__name__
                    self.logger.info(f"Dynamic line [{name}] deactivated")
                self.active_strategy.on_deactivate(state)
                self.active_strategy = None
                self.dynamic_line = []
                # Snap closest_idx to merge point so racing line resumes smoothly
                state.closest_idx = self.merge_idx
                return

        # If no strategy is active, check if any should activate (priority order)
        if self.active_strategy is None:
            for strategy in self._strategies:
                if strategy.should_activate(state, self.racing_line):
                    result = strategy.generate(state, self.racing_line)
                    if result is not None:
                        self.dynamic_line, self.merge_idx = result
                        self.dynamic_closest_idx = 0
                        self.active_strategy = strategy
                        strategy.on_activate(state)
                        if self.logger:
                            name = type(strategy).__name__
                            self.logger.warn(
                                f"Dynamic line [{name}] activated — "
                                f"CTE={state.cross_track_error:.2f}m, "
                                f"merge_idx={self.merge_idx}"
                            )
                        break  # Only one active strategy at a time

    @property
    def is_active(self) -> bool:
        return self.active_strategy is not None

    def get_line_and_idx(self) -> Tuple[list, int]:
        """
        Returns (line, closest_idx) to use for pick_lookahead_point.
        If a dynamic line is active, returns that. Else return racing line info.
        """
        if self.is_active and self.dynamic_line:
            return self.dynamic_line, self.dynamic_closest_idx
        return self.racing_line, -1  # -1 = node's own closest_idx

    def set_dynamic_closest_idx(self, idx: int) -> None:
        """Update progress along the dynamic line (called after pick_lookahead_point)."""
        self.dynamic_closest_idx = idx

    def compute_cte(self, xy: Tuple[float, float], racing_line: list, closest_idx: int) -> float:
        if not racing_line:
            return 0.0
        row = racing_line[closest_idx]
        dx = xy[0] - float(row[1])
        dy = xy[1] - float(row[2])
        return math.hypot(dx, dy)
