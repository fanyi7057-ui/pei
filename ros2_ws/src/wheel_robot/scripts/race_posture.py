"""Validated complete body-posture values for the normal competition state."""

from dataclasses import dataclass
import math
from typing import Tuple


RACE_POSTURE_NAMES = (
    "joint_height",
    "joint_roll",
    "joint_pitching",
    "joint_slide",
)

LEG_HEIGHT_MIN_M = 0.14
LEG_HEIGHT_MAX_M = 0.36
ROLL_MIN_DEG = -15.0
ROLL_MAX_DEG = 15.0
PITCH_MIN_DEG = -35.0
PITCH_MAX_DEG = 35.0
SLIDE_MIN_M = -0.08
SLIDE_MAX_M = 0.08

DEFAULT_RACE_HEIGHT_M = 0.15
DEFAULT_RACE_ROLL_DEG = 0.0
# 实车确认：正俯仰为摄像头一端低头。
DEFAULT_RACE_PITCH_DEG = 28.0
DEFAULT_RACE_SLIDE_M = 0.0


def _bounded(value: object, lower: float, upper: float, default: float) -> float:
    """Return a finite value clamped to one lower-controller interface range."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    if not math.isfinite(number):
        number = default
    return max(lower, min(upper, number))


@dataclass(frozen=True)
class RacePosture:
    """One complete normal-race posture published through ``/cmd_posture``."""

    height_m: float
    roll_deg: float
    pitch_deg: float
    slide_m: float

    @classmethod
    def from_values(
        cls,
        height_m: object = DEFAULT_RACE_HEIGHT_M,
        roll_deg: object = DEFAULT_RACE_ROLL_DEG,
        pitch_deg: object = DEFAULT_RACE_PITCH_DEG,
        slide_m: object = DEFAULT_RACE_SLIDE_M,
    ) -> "RacePosture":
        return cls(
            height_m=_bounded(
                height_m,
                LEG_HEIGHT_MIN_M,
                LEG_HEIGHT_MAX_M,
                DEFAULT_RACE_HEIGHT_M,
            ),
            roll_deg=_bounded(
                roll_deg,
                ROLL_MIN_DEG,
                ROLL_MAX_DEG,
                DEFAULT_RACE_ROLL_DEG,
            ),
            pitch_deg=_bounded(
                pitch_deg,
                PITCH_MIN_DEG,
                PITCH_MAX_DEG,
                DEFAULT_RACE_PITCH_DEG,
            ),
            slide_m=_bounded(
                slide_m,
                SLIDE_MIN_M,
                SLIDE_MAX_M,
                DEFAULT_RACE_SLIDE_M,
            ),
        )

    @property
    def names(self) -> Tuple[str, str, str, str]:
        return RACE_POSTURE_NAMES

    @property
    def positions(self) -> Tuple[float, float, float, float]:
        return (self.height_m, self.roll_deg, self.pitch_deg, self.slide_m)


class RacePostureTransition:
    """Interpolate normal-race posture targets without importing ROS."""

    def __init__(
        self, initial: RacePosture, duration_seconds: float = 0.5
    ) -> None:
        self._start = initial
        self._target = initial
        self._started_at = 0.0
        self._duration_seconds = max(1e-6, float(duration_seconds))

    def set_target(self, target: RacePosture, now: float) -> None:
        """Start the next transition from the current interpolated posture."""
        self._start = self.at(now)
        self._target = target
        self._started_at = float(now)

    def at(self, now: float) -> RacePosture:
        """Return the bounded posture that should be published at this time."""
        ratio = max(
            0.0,
            min(
                1.0,
                (float(now) - self._started_at) / self._duration_seconds,
            ),
        )
        return RacePosture.from_values(
            self._start.height_m
            + (self._target.height_m - self._start.height_m) * ratio,
            self._start.roll_deg
            + (self._target.roll_deg - self._start.roll_deg) * ratio,
            self._start.pitch_deg
            + (self._target.pitch_deg - self._start.pitch_deg) * ratio,
            self._start.slide_m
            + (self._target.slide_m - self._start.slide_m) * ratio,
        )
