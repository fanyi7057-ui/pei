"""Safe command data shared by the stand-only ROS2 node."""

from dataclasses import dataclass
import math
from typing import Tuple

from race_posture import RacePosture


DEFAULT_STAND_HEIGHT_M = 0.25
MIN_STAND_HEIGHT_M = 0.15
MAX_STAND_HEIGHT_M = 0.35
STAND_JOINT_NAMES = (
    "joint_height",
    "joint_roll",
    "joint_pitching",
    "joint_slide",
)


def safe_stand_height(value: object) -> float:
    """Return a finite, supported standing height or the safe default."""
    try:
        height = float(value)
    except (TypeError, ValueError):
        return DEFAULT_STAND_HEIGHT_M
    if not math.isfinite(height):
        return DEFAULT_STAND_HEIGHT_M
    if not MIN_STAND_HEIGHT_M <= height <= MAX_STAND_HEIGHT_M:
        return DEFAULT_STAND_HEIGHT_M
    return height


@dataclass(frozen=True)
class StandCommand:
    """Complete safe output expected from the stand-only node."""

    linear_x: float
    angular_z: float
    joint_names: Tuple[str, str, str, str]
    joint_positions: Tuple[float, float, float, float]
    jump: bool

    @classmethod
    def default(cls, height_m: object = DEFAULT_STAND_HEIGHT_M) -> "StandCommand":
        return cls(
            linear_x=0.0,
            angular_z=0.0,
            joint_names=STAND_JOINT_NAMES,
            joint_positions=(safe_stand_height(height_m), 0.0, 0.0, 0.0),
            jump=False,
        )

    @classmethod
    def from_posture(
        cls,
        height_m: object,
        roll_deg: object,
        pitch_deg: object,
        slide_m: object,
    ) -> "StandCommand":
        """Build a stationary command from the complete competition posture."""
        posture = RacePosture.from_values(
            height_m, roll_deg, pitch_deg, slide_m
        )
        return cls(
            linear_x=0.0,
            angular_z=0.0,
            joint_names=posture.names,
            joint_positions=posture.positions,
            jump=False,
        )
