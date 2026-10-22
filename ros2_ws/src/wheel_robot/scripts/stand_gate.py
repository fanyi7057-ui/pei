"""Pure pre-race stand-confirmation state machine.

This module deliberately has no ROS dependency so the safety state transition
can be tested on the development computer before it is connected to a robot.
"""

from enum import Enum
import math


DEFAULT_STAND_HEIGHT_M = 0.25
MIN_STAND_HEIGHT_M = 0.15
MAX_STAND_HEIGHT_M = 0.35


class StandGateState(str, Enum):
    """States used before normal race motion is permitted."""

    DISABLED = "DISABLED"
    STAND_INIT = "STAND_INIT"
    STAND_SETTLE = "STAND_SETTLE"
    STAND_CONFIRM = "STAND_CONFIRM"
    RACE_ACTIVE = "RACE_ACTIVE"


def is_enter_confirmation(value: str) -> bool:
    """Accept only a direct Enter keypress, represented by an empty input line."""
    return value == ""


def safe_stand_height(value: object) -> float:
    """Return a finite, conservative standing height or the safe default."""
    try:
        height = float(value)
    except (TypeError, ValueError):
        return DEFAULT_STAND_HEIGHT_M
    if not math.isfinite(height):
        return DEFAULT_STAND_HEIGHT_M
    if not MIN_STAND_HEIGHT_M <= height <= MAX_STAND_HEIGHT_M:
        return DEFAULT_STAND_HEIGHT_M
    return height


class StandGate:
    """Require a timed stand hold and explicit confirmation before racing."""

    def __init__(self, enabled: bool, settle_seconds: float) -> None:
        self.enabled = bool(enabled)
        self.settle_seconds = max(0.0, float(settle_seconds))
        self.state = (
            StandGateState.STAND_INIT
            if self.enabled
            else StandGateState.DISABLED
        )
        self.deadline = 0.0

    def advance(self, now: float) -> bool:
        """Advance timing and return whether normal race motion is blocked."""
        if self.state in (StandGateState.DISABLED, StandGateState.RACE_ACTIVE):
            return False
        if self.state is StandGateState.STAND_INIT:
            self.deadline = float(now) + self.settle_seconds
            self.state = StandGateState.STAND_SETTLE
        elif self.state is StandGateState.STAND_SETTLE and float(now) >= self.deadline:
            self.state = StandGateState.STAND_CONFIRM
        return True

    def confirm(self) -> bool:
        """Unlock racing only after the timed hold has completed."""
        if self.state is not StandGateState.STAND_CONFIRM:
            return False
        self.state = StandGateState.RACE_ACTIVE
        return True

    def accept_ros_start_confirmation(self) -> bool:
        """Accept an explicit ROS start request only at the confirmation gate."""
        return self.confirm()
