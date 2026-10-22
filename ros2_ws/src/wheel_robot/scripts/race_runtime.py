"""ROS-independent validation for safe, live competition tuning.

This module intentionally does not import ROS or command hardware.  It only
validates values and records software-only cue events so it can be tested on a
development computer before the Orange Pi workspace is built.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
import hashlib
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml


LIVE_PREFIX = "live_tuning."
LIVE_TUNING_FILE_PATH = Path(
    "/home/orangepi/robot/ros2_ws/src/wheel_robot/config/race_tuning.yaml"
)


@dataclass(frozen=True)
class BeepPattern:
    """One event's requested beep pattern, in seconds."""

    count: int
    on_seconds: float
    off_seconds: float

    @property
    def total_seconds(self) -> float:
        """Duration from first beep start until the final beep ends."""
        return self.count * self.on_seconds + max(0, self.count - 1) * self.off_seconds


@dataclass(frozen=True)
class SoftwareBuzzerEvent:
    """A truthful record of a requested cue, not a hardware-delivery claim."""

    name: str
    delivery: str
    pattern: BeepPattern
    complete_at: float


@dataclass
class PredictiveTurnGate:
    """Confirm an upcoming curve before a short direction-aware override."""

    confirm_frames: int
    far_error_threshold: float
    _candidate_direction: int = 0
    _candidate_frames: int = 0

    def reset(self) -> None:
        self._candidate_direction = 0
        self._candidate_frames = 0

    def observe(self, near_error: float, mid_error: float, far_error: float) -> int:
        values = (near_error, mid_error, far_error)
        if not all(math.isfinite(float(value)) for value in values):
            self.reset()
            return 0

        threshold = float(self.far_error_threshold)
        candidate = 0
        if (
            abs(float(near_error)) <= threshold
            and abs(float(mid_error)) >= threshold * 0.5
            and abs(float(far_error)) >= threshold
            and float(mid_error) * float(far_error) > 0.0
        ):
            candidate = 1 if float(far_error) > 0.0 else -1

        if candidate == 0:
            self.reset()
            return 0
        if candidate != self._candidate_direction:
            self._candidate_direction = candidate
            self._candidate_frames = 1
        else:
            self._candidate_frames += 1
        if self._candidate_frames >= int(self.confirm_frames):
            self.reset()
            return candidate
        return 0


@dataclass(frozen=True)
class FirstTurnCommand:
    """One pure first-left-bend decision before ROS command publication."""

    linear: float = 0.0
    angular: float = 0.0
    mode: str = "first_turn_wait"


@dataclass
class FirstTurnFollower:
    """Track the first physical left bend from every fresh vision frame.

    The image error convention used by this project maps a negative far-field
    curve to a positive (left) chassis command.  This controller therefore
    detects negative mid/far curvature, but always returns a positive physical
    left angular command.  It permanently completes after the first bend.
    """

    entered: bool = False
    done: bool = False
    candidate_frames: int = 0
    straight_frames: int = 0
    entered_at: float = 0.0
    _last_sequence: int = -1
    _last_command: FirstTurnCommand = FirstTurnCommand()

    def reset(self) -> None:
        self.entered = False
        self.done = False
        self.candidate_frames = 0
        self.straight_frames = 0
        self.entered_at = 0.0
        self._last_sequence = -1
        self._last_command = FirstTurnCommand()

    def complete_without_rearm(self) -> None:
        """End first-turn ownership permanently when the course leaves start seek."""
        if not self.done:
            self.entered = False
            self.done = True
            self.candidate_frames = 0
            self.straight_frames = 0
            self._last_command = FirstTurnCommand(mode="first_turn_complete")

    def update(
        self,
        sequence: int,
        now: float,
        near_error: float,
        mid_error: float,
        far_error: float,
        tuning: "LiveRaceTuning",
    ) -> FirstTurnCommand:
        if self.done or not tuning.first_turn_enabled:
            return FirstTurnCommand(mode="first_turn_done")
        if sequence == self._last_sequence:
            return self._last_command
        self._last_sequence = int(sequence)

        values = (near_error, mid_error, far_error)
        if not all(math.isfinite(float(value)) for value in values):
            self.candidate_frames = 0
            self.straight_frames = 0
            self._last_command = FirstTurnCommand()
            return self._last_command

        threshold = float(tuning.first_turn_far_error_threshold)
        left_candidate = (
            abs(float(near_error)) <= threshold
            and float(mid_error) <= -0.5 * threshold
            and float(far_error) <= -threshold
        )
        if not self.entered:
            self.candidate_frames = self.candidate_frames + 1 if left_candidate else 0
            if self.candidate_frames < int(tuning.first_turn_confirm_frames):
                self._last_command = FirstTurnCommand()
                return self._last_command
            self.entered = True
            self.entered_at = float(now)

        if float(now) - self.entered_at >= float(tuning.first_turn_max_seconds):
            self.done = True
            self._last_command = FirstTurnCommand(mode="first_turn_complete")
            return self._last_command

        straight = all(abs(float(value)) <= threshold for value in values)
        self.straight_frames = self.straight_frames + 1 if straight else 0
        if self.straight_frames >= int(tuning.first_turn_exit_confirm_frames):
            self.done = True
            self._last_command = FirstTurnCommand(mode="first_turn_complete")
            return self._last_command

        curve = max(
            0.0,
            -(0.20 * float(near_error) + 0.30 * float(mid_error) + 0.50 * float(far_error)),
        )
        angular = min(
            float(tuning.first_turn_max_left_turn_speed),
            float(tuning.first_turn_turn_kp) * curve,
        )
        self._last_command = FirstTurnCommand(
            linear=float(tuning.first_turn_forward_speed),
            angular=angular,
            mode="first_turn_tracking",
        )
        return self._last_command


@dataclass
class FastRoundaboutEntryGate:
    """Confirm the user-defined high-speed ring entrance on distinct frames.

    This helper deliberately has no timeout or fallback.  After the race node
    changes into fast ring entry, only seeing the inner line can release the
    forced left search.
    """

    candidate_frames: int = 0
    _last_sequence: int = -1

    def reset(self) -> None:
        self.candidate_frames = 0
        self._last_sequence = -1

    def observe(
        self,
        sequence: int,
        total_components: int,
        left_components: int,
        tuning: "LiveRaceTuning",
    ) -> bool:
        if not tuning.roundabout_fast_entry_enabled:
            self.reset()
            return False
        if int(sequence) == self._last_sequence:
            return self.candidate_frames >= int(
                tuning.roundabout_fast_entry_confirm_frames
            )
        self._last_sequence = int(sequence)
        candidate = (
            int(total_components) >= int(tuning.roundabout_fast_entry_min_components)
            and int(left_components)
            >= int(tuning.roundabout_fast_entry_min_left_components)
        )
        self.candidate_frames = self.candidate_frames + 1 if candidate else 0
        return self.candidate_frames >= int(
            tuning.roundabout_fast_entry_confirm_frames
        )


@dataclass(frozen=True)
class LiveRaceTuning:
    """Small allowlist of settings that are safe to adjust during a race run."""

    step_enabled: bool
    step_visual_jump_enabled: bool
    step_reverse_align_enabled: bool
    step_reverse_speed: float
    step_reverse_align_timeout_seconds: float
    step_align_center_tolerance: float
    step_align_heading_tolerance: float
    step_align_confirm_frames: int
    step_align_turn_kp: float
    step_align_max_turn_speed: float
    step_sprint_speed: float
    step_sprint_accel: float
    step_sprint_min_seconds: float
    step_sprint_min_distance_m: float
    step_sprint_max_seconds: float
    step_sprint_max_distance_m: float
    step_line_loss_confirm_frames: int
    step_heading_hold_enabled: bool
    step_heading_kp: float
    step_heading_max_turn_speed: float
    step_jump_on_loss_enabled: bool
    step_jump_forward_speed: float
    step_jump_min_height: float
    step_jump_max_height: float
    step_jump_stand_height_m: float
    step_jump_pulse_seconds: float
    step_landing_seconds: float
    step_post_forward_speed: float
    step_post_line_confirm_frames: int
    step_post_timeout_seconds: float
    step_post_max_distance_m: float
    crosswalk_enabled: bool
    crosswalk_confirm_frames: int
    crosswalk_stop_seconds: float
    crosswalk_brake_reverse_enabled: bool
    crosswalk_brake_reverse_seconds: float
    crosswalk_brake_reverse_speed: float
    crosswalk_delay_after_roundabout: float
    crosswalk_approach_distance_m: float
    drive_forward_speed: float
    drive_start_soft_enabled: bool
    drive_start_soft_seconds: float
    drive_start_soft_speed: float
    drive_min_forward_speed: float
    drive_max_turn_speed: float
    drive_linear_accel: float
    drive_angular_accel: float
    drive_curve_slowdown: float
    drive_curve_speed_cap: float
    drive_curve_strong_speed_cap: float
    drive_edge_rescue_speed_cap: float
    drive_edge_rescue_max_turn_speed: float
    drive_hard_turn_speed_cap: float
    drive_hard_turn_max_turn_speed: float
    drive_big_turn_loss_seconds: float
    drive_big_turn_loss_speed: float
    drive_big_turn_loss_turn: float
    drive_preturn_enabled: bool
    drive_preturn_confirm_frames: int
    drive_preturn_far_error_threshold: float
    drive_preturn_turn_speed: float
    drive_preturn_seconds: float
    drive_preturn_speed: float
    first_turn_enabled: bool
    first_turn_confirm_frames: int
    first_turn_far_error_threshold: float
    first_turn_turn_kp: float
    first_turn_max_left_turn_speed: float
    first_turn_forward_speed: float
    first_turn_exit_confirm_frames: int
    first_turn_max_seconds: float
    line_loss_left_enabled: bool
    line_loss_left_speed_ratio: float
    line_loss_left_turn_speed: float
    line_loss_left_timeout_seconds: float
    roundabout_forward_speed: float
    roundabout_exit_speed: float
    roundabout_entry_turn_speed: float
    roundabout_entry_turn_distance: float
    roundabout_exit_min_turn_speed: float
    roundabout_exit_max_turn_speed: float
    roundabout_follow_min_turn_speed: float
    roundabout_blind_recovery_speed: float
    roundabout_blind_recovery_left_turn: float
    roundabout_blue_guard_speed: float
    roundabout_blue_guard_turn_speed: float
    roundabout_approach_speed: float
    roundabout_approach_distance_m: float
    roundabout_fast_entry_enabled: bool
    roundabout_fast_entry_arm_delay_seconds: float
    roundabout_fast_entry_min_seek_distance_m: float
    roundabout_fast_entry_min_components: int
    roundabout_fast_entry_min_left_components: int
    roundabout_fast_entry_confirm_frames: int
    roundabout_fast_entry_min_search_seconds: float
    roundabout_fast_entry_forward_speed: float
    roundabout_fast_entry_left_turn_speed: float
    roundabout_fast_entry_reacquire_frames: int
    posture_height_m: float
    posture_roll_deg: float
    posture_pitch_deg: float
    posture_slide_m: float
    buzzer_enabled: bool
    buzzer_start_count: int
    buzzer_start_on_seconds: float
    buzzer_start_off_seconds: float
    buzzer_crosswalk_count: int
    buzzer_crosswalk_on_seconds: float
    buzzer_crosswalk_off_seconds: float
    buzzer_finish_count: int
    buzzer_finish_on_seconds: float
    buzzer_finish_off_seconds: float

    @classmethod
    def defaults(cls) -> "LiveRaceTuning":
        """Safe trial defaults; the ROS YAML supplies the same baseline."""
        return cls(
            step_enabled=False,
            step_visual_jump_enabled=True,
            step_reverse_align_enabled=True,
            step_reverse_speed=0.06,
            step_reverse_align_timeout_seconds=3.0,
            step_align_center_tolerance=0.06,
            step_align_heading_tolerance=0.12,
            step_align_confirm_frames=2,
            step_align_turn_kp=1.0,
            step_align_max_turn_speed=0.35,
            step_sprint_speed=0.70,
            step_sprint_accel=0.65,
            step_sprint_min_seconds=0.15,
            step_sprint_min_distance_m=0.05,
            step_sprint_max_seconds=3.0,
            step_sprint_max_distance_m=1.20,
            step_line_loss_confirm_frames=2,
            step_heading_hold_enabled=True,
            step_heading_kp=1.20,
            step_heading_max_turn_speed=0.40,
            step_jump_on_loss_enabled=True,
            step_jump_forward_speed=0.60,
            step_jump_min_height=0.15,
            step_jump_max_height=0.33,
            step_jump_stand_height_m=0.25,
            step_jump_pulse_seconds=0.20,
            step_landing_seconds=0.50,
            step_post_forward_speed=0.12,
            step_post_line_confirm_frames=2,
            step_post_timeout_seconds=3.8,
            step_post_max_distance_m=0.38,
            crosswalk_enabled=True,
            crosswalk_confirm_frames=2,
            crosswalk_stop_seconds=3.0,
            crosswalk_brake_reverse_enabled=True,
            crosswalk_brake_reverse_seconds=0.50,
            crosswalk_brake_reverse_speed=0.06,
            crosswalk_delay_after_roundabout=1.5,
            crosswalk_approach_distance_m=0.30,
            drive_forward_speed=0.25,
            drive_start_soft_enabled=True,
            drive_start_soft_seconds=3.00,
            drive_start_soft_speed=0.20,
            drive_min_forward_speed=0.10,
            drive_max_turn_speed=0.70,
            drive_linear_accel=0.42,
            drive_angular_accel=2.50,
            drive_curve_slowdown=0.72,
            drive_curve_speed_cap=0.17,
            drive_curve_strong_speed_cap=0.145,
            drive_edge_rescue_speed_cap=0.085,
            drive_edge_rescue_max_turn_speed=0.42,
            drive_hard_turn_speed_cap=0.105,
            drive_hard_turn_max_turn_speed=0.56,
            drive_big_turn_loss_seconds=1.00,
            drive_big_turn_loss_speed=0.075,
            drive_big_turn_loss_turn=0.34,
            drive_preturn_enabled=True,
            drive_preturn_confirm_frames=2,
            drive_preturn_far_error_threshold=0.12,
            drive_preturn_turn_speed=0.45,
            drive_preturn_seconds=0.30,
            drive_preturn_speed=0.18,
            first_turn_enabled=True,
            first_turn_confirm_frames=2,
            first_turn_far_error_threshold=0.07,
            first_turn_turn_kp=1.20,
            first_turn_max_left_turn_speed=0.80,
            first_turn_forward_speed=0.25,
            first_turn_exit_confirm_frames=2,
            first_turn_max_seconds=4.0,
            line_loss_left_enabled=True,
            line_loss_left_speed_ratio=1.0,
            line_loss_left_turn_speed=0.60,
            line_loss_left_timeout_seconds=1.5,
            roundabout_forward_speed=0.19,
            roundabout_exit_speed=0.20,
            roundabout_entry_turn_speed=0.34,
            roundabout_entry_turn_distance=1.00,
            roundabout_exit_min_turn_speed=0.10,
            roundabout_exit_max_turn_speed=0.22,
            roundabout_follow_min_turn_speed=-0.04,
            roundabout_blind_recovery_speed=0.12,
            roundabout_blind_recovery_left_turn=0.16,
            roundabout_blue_guard_speed=0.045,
            roundabout_blue_guard_turn_speed=0.30,
            roundabout_approach_speed=0.20,
            roundabout_approach_distance_m=1.20,
            roundabout_fast_entry_enabled=True,
            roundabout_fast_entry_arm_delay_seconds=0.0,
            roundabout_fast_entry_min_seek_distance_m=0.0,
            roundabout_fast_entry_min_components=3,
            roundabout_fast_entry_min_left_components=2,
            roundabout_fast_entry_confirm_frames=2,
            roundabout_fast_entry_min_search_seconds=0.50,
            roundabout_fast_entry_forward_speed=0.20,
            roundabout_fast_entry_left_turn_speed=0.82,
            roundabout_fast_entry_reacquire_frames=3,
            posture_height_m=0.15,
            posture_roll_deg=0.0,
            posture_pitch_deg=28.0,
            posture_slide_m=0.0,
            buzzer_enabled=False,
            buzzer_start_count=2,
            buzzer_start_on_seconds=0.10,
            buzzer_start_off_seconds=1.00,
            buzzer_crosswalk_count=1,
            buzzer_crosswalk_on_seconds=1.00,
            buzzer_crosswalk_off_seconds=0.00,
            buzzer_finish_count=1,
            buzzer_finish_on_seconds=1.00,
            buzzer_finish_off_seconds=0.00,
        )

    @classmethod
    def parameter_defaults(cls) -> Dict[str, Any]:
        """Return ROS parameter names and YAML-equivalent default values."""
        tuning = cls.defaults()
        return {
            f"{LIVE_PREFIX}{name}": int(value) if isinstance(value, bool) else value
            for name, value in tuning.__dict__.items()
        }

    def as_parameter_values(self) -> Dict[str, Any]:
        """Return the current snapshot using ROS parameter names."""
        return {
            f"{LIVE_PREFIX}{name}": int(value) if isinstance(value, bool) else value
            for name, value in self.__dict__.items()
        }

    def with_updates(
        self, changes: Mapping[str, Any]
    ) -> Tuple["LiveRaceTuning", str]:
        """Validate all requested updates atomically.

        On error this returns the existing object unchanged, allowing the ROS
        callback to reject one complete parameter request without partial
        mutation of the active race configuration.
        """
        current = self.as_parameter_values()
        allowed = set(current)
        unknown = sorted(set(changes) - allowed)
        if unknown:
            return self, f"不允许实时修改参数: {', '.join(unknown)}"

        candidate = dict(current)
        candidate.update(changes)
        reason = self._validate(candidate)
        if reason:
            return self, reason

        values = {}
        for parameter_name, value in candidate.items():
            field_name = parameter_name.removeprefix(LIVE_PREFIX)
            if field_name in self._switch_names():
                values[field_name] = bool(value)
            elif field_name in self._integer_names():
                values[field_name] = int(value)
            else:
                values[field_name] = float(value)
        return replace(self, **values), ""

    @staticmethod
    def _switch_names() -> Tuple[str, ...]:
        return (
            "step_enabled",
            "step_visual_jump_enabled",
            "step_reverse_align_enabled",
            "step_heading_hold_enabled",
            "step_jump_on_loss_enabled",
            "crosswalk_enabled",
            "crosswalk_brake_reverse_enabled",
            "drive_start_soft_enabled",
            "drive_preturn_enabled",
            "first_turn_enabled",
            "line_loss_left_enabled",
            "roundabout_fast_entry_enabled",
            "buzzer_enabled",
        )

    @staticmethod
    def _integer_names() -> Tuple[str, ...]:
        return (
            "crosswalk_confirm_frames",
            "step_align_confirm_frames",
            "step_line_loss_confirm_frames",
            "step_post_line_confirm_frames",
            "drive_preturn_confirm_frames",
            "first_turn_confirm_frames",
            "first_turn_exit_confirm_frames",
            "roundabout_fast_entry_min_components",
            "roundabout_fast_entry_min_left_components",
            "roundabout_fast_entry_confirm_frames",
            "roundabout_fast_entry_reacquire_frames",
            "buzzer_start_count",
            "buzzer_crosswalk_count",
            "buzzer_finish_count",
        )

    @classmethod
    def _validate(cls, values: Mapping[str, Any]) -> str:
        for name in cls._switch_names():
            value = values[f"{LIVE_PREFIX}{name}"]
            if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1):
                return f"{LIVE_PREFIX}{name} 只能是整数 0 或 1"

        int_ranges = {
            "step_align_confirm_frames": (1, 10),
            "step_line_loss_confirm_frames": (1, 10),
            "step_post_line_confirm_frames": (1, 10),
            "crosswalk_confirm_frames": (1, 10),
            "drive_preturn_confirm_frames": (1, 10),
            "first_turn_confirm_frames": (1, 10),
            "first_turn_exit_confirm_frames": (1, 10),
            "roundabout_fast_entry_min_components": (1, 20),
            "roundabout_fast_entry_min_left_components": (0, 20),
            "roundabout_fast_entry_confirm_frames": (1, 10),
            "roundabout_fast_entry_reacquire_frames": (1, 10),
            "buzzer_start_count": (1, 5),
            "buzzer_crosswalk_count": (1, 5),
            "buzzer_finish_count": (1, 5),
        }
        for name, (minimum, maximum) in int_ranges.items():
            value = values[f"{LIVE_PREFIX}{name}"]
            if isinstance(value, bool) or not isinstance(value, int):
                return f"{LIVE_PREFIX}{name} 必须是整数"
            if not minimum <= value <= maximum:
                return f"{LIVE_PREFIX}{name} 必须在 {minimum}..{maximum} 范围内"

        unbounded_nonnegative_speeds = (
            "step_reverse_speed",
            "step_align_turn_kp",
            "step_align_max_turn_speed",
            "step_sprint_speed",
            "step_sprint_accel",
            "step_sprint_min_seconds",
            "step_sprint_min_distance_m",
            "step_heading_kp",
            "step_heading_max_turn_speed",
            "step_jump_forward_speed",
            "step_landing_seconds",
            "step_post_forward_speed",
            "crosswalk_approach_distance_m",
            "drive_forward_speed",
            "drive_start_soft_seconds",
            "drive_start_soft_speed",
            "drive_min_forward_speed",
            "drive_max_turn_speed",
            "drive_linear_accel",
            "drive_angular_accel",
            "drive_curve_speed_cap",
            "drive_curve_strong_speed_cap",
            "drive_edge_rescue_speed_cap",
            "drive_edge_rescue_max_turn_speed",
            "drive_hard_turn_speed_cap",
            "drive_hard_turn_max_turn_speed",
            "drive_big_turn_loss_seconds",
            "drive_big_turn_loss_speed",
            "drive_big_turn_loss_turn",
            "drive_preturn_turn_speed",
            "drive_preturn_seconds",
            "drive_preturn_speed",
            "first_turn_turn_kp",
            "first_turn_max_left_turn_speed",
            "first_turn_forward_speed",
            "first_turn_max_seconds",
            "line_loss_left_speed_ratio",
            "line_loss_left_turn_speed",
            "line_loss_left_timeout_seconds",
            "roundabout_forward_speed",
            "roundabout_exit_speed",
            "roundabout_entry_turn_speed",
            "roundabout_entry_turn_distance",
            "roundabout_exit_max_turn_speed",
            "roundabout_blind_recovery_speed",
            "roundabout_blind_recovery_left_turn",
            "roundabout_blue_guard_speed",
            "roundabout_blue_guard_turn_speed",
            "roundabout_approach_speed",
            "roundabout_approach_distance_m",
            "roundabout_fast_entry_arm_delay_seconds",
            "roundabout_fast_entry_min_seek_distance_m",
            "roundabout_fast_entry_min_search_seconds",
            "roundabout_fast_entry_forward_speed",
            "roundabout_fast_entry_left_turn_speed",
        )
        for name in unbounded_nonnegative_speeds:
            value = values[f"{LIVE_PREFIX}{name}"]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f"{LIVE_PREFIX}{name} 必须是数字"
            numeric_value = float(value)
            if not math.isfinite(numeric_value) or numeric_value < 0.0:
                return f"{LIVE_PREFIX}{name} 必须是有限的非负数"

        finite_signed_turns = (
            "roundabout_exit_min_turn_speed",
            "roundabout_follow_min_turn_speed",
        )
        for name in finite_signed_turns:
            value = values[f"{LIVE_PREFIX}{name}"]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f"{LIVE_PREFIX}{name} 必须是数字"
            if not math.isfinite(float(value)):
                return f"{LIVE_PREFIX}{name} 必须是有限数"

        float_ranges = {
            "step_align_center_tolerance": (0.0, 1.0),
            "step_align_heading_tolerance": (0.0, 2.0),
            "step_jump_min_height": (0.14, 0.36),
            "step_jump_max_height": (0.14, 0.36),
            "step_jump_stand_height_m": (0.14, 0.36),
            "crosswalk_stop_seconds": (0.0, 15.0),
            "crosswalk_brake_reverse_seconds": (0.0, 2.0),
            "crosswalk_brake_reverse_speed": (0.0, 0.30),
            "crosswalk_delay_after_roundabout": (0.0, 10.0),
            "drive_curve_slowdown": (0.0, 1.0),
            "drive_preturn_far_error_threshold": (0.01, 1.0),
            "first_turn_far_error_threshold": (0.01, 1.0),
            "posture_height_m": (0.14, 0.36),
            "posture_roll_deg": (-15.0, 15.0),
            "posture_pitch_deg": (-35.0, 35.0),
            "posture_slide_m": (-0.08, 0.08),
            "buzzer_start_on_seconds": (0.05, 2.00),
            "buzzer_start_off_seconds": (0.00, 3.00),
            "buzzer_crosswalk_on_seconds": (0.05, 2.00),
            "buzzer_crosswalk_off_seconds": (0.00, 3.00),
            "buzzer_finish_on_seconds": (0.05, 2.00),
            "buzzer_finish_off_seconds": (0.00, 3.00),
        }
        for name, (minimum, maximum) in float_ranges.items():
            value = values[f"{LIVE_PREFIX}{name}"]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f"{LIVE_PREFIX}{name} 必须是数字"
            if not minimum <= float(value) <= maximum:
                return f"{LIVE_PREFIX}{name} 必须在 {minimum}..{maximum} 范围内"

        if (
            float(values[f"{LIVE_PREFIX}drive_min_forward_speed"])
            > float(values[f"{LIVE_PREFIX}drive_forward_speed"])
        ):
            return "live_tuning.drive_min_forward_speed 不能大于 drive_forward_speed"
        if (
            float(values[f"{LIVE_PREFIX}drive_start_soft_speed"])
            > float(values[f"{LIVE_PREFIX}drive_forward_speed"])
        ):
            return "live_tuning.drive_start_soft_speed 不能大于 drive_forward_speed"
        if (
            int(values[f"{LIVE_PREFIX}roundabout_fast_entry_min_left_components"])
            > int(values[f"{LIVE_PREFIX}roundabout_fast_entry_min_components"])
        ):
            return "高速进环左半画面块数不能大于总块数"
        for name in (
            "step_reverse_align_timeout_seconds", "step_sprint_max_seconds",
            "step_sprint_max_distance_m", "step_jump_pulse_seconds",
            "step_post_timeout_seconds", "step_post_max_distance_m",
        ):
            value = values[f"{LIVE_PREFIX}{name}"]
            if (
                isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(float(value)) or float(value) <= 0.0
            ):
                return f"{LIVE_PREFIX}{name} 必须是有限正数"
        if values[f"{LIVE_PREFIX}step_jump_min_height"] > values[f"{LIVE_PREFIX}step_jump_max_height"]:
            return "台阶跳跃低位腿高不能大于高位腿高"
        if values[f"{LIVE_PREFIX}step_sprint_min_seconds"] >= values[f"{LIVE_PREFIX}step_sprint_max_seconds"]:
            return "冲刺最短时间必须小于最长时间"
        if values[f"{LIVE_PREFIX}step_sprint_min_distance_m"] >= values[f"{LIVE_PREFIX}step_sprint_max_distance_m"]:
            return "冲刺最短距离必须小于最长距离"
        if values[f"{LIVE_PREFIX}step_landing_seconds"] >= values[f"{LIVE_PREFIX}step_post_timeout_seconds"]:
            return "落地保护时间必须小于落地找线最长时间"
        return ""

    def pattern_for(self, name: str) -> BeepPattern:
        """Return the configured pattern for one named course event."""
        patterns = {
            "start": BeepPattern(
                self.buzzer_start_count,
                self.buzzer_start_on_seconds,
                self.buzzer_start_off_seconds,
            ),
            "crosswalk": BeepPattern(
                self.buzzer_crosswalk_count,
                self.buzzer_crosswalk_on_seconds,
                self.buzzer_crosswalk_off_seconds,
            ),
            "finish": BeepPattern(
                self.buzzer_finish_count,
                self.buzzer_finish_on_seconds,
                self.buzzer_finish_off_seconds,
            ),
        }
        if name not in patterns:
            raise ValueError(f"未知蜂鸣事件: {name}")
        return patterns[name]


@dataclass(frozen=True)
class VisualStepCommand:
    """One testable step decision; the ROS node alone publishes it."""

    linear: float = 0.0
    angular: float = 0.0
    jump: bool = False
    mode: str = "step_visual_idle"


class VisualStepController:
    """Reverse-align, save odometry heading, sprint on line loss, reacquire.

    A fresh RGB frame is distinct from a frame with no line. Only distinct,
    valid camera frames count toward alignment, line loss and reacquisition.
    Time/distance exits are checked on every control tick.
    """

    def __init__(self) -> None:
        self.state = "IDLE"
        self.reason = "等待台阶"
        self.entered_at = 0.0
        self.target_yaw: Optional[float] = None
        self.origin: Optional[Tuple[float, float, float]] = None
        self.aligned_frames = 0
        self.lost_frames = 0
        self.found_frames = 0
        self.last_line_valid = False
        self.velocity = 0.0
        self.last_tick = 0.0

    @property
    def active(self) -> bool:
        return self.state not in ("IDLE", "DONE")

    def _enter(self, state: str, now: float, reason: str) -> None:
        self.state = state
        self.entered_at = now
        self.reason = reason

    def start(self, now: float, pose: Tuple[float, float, float]) -> None:
        self.__init__()
        self.origin = pose
        self.last_tick = now
        self._enter("VISUAL_ALIGN", now, "台阶确认，后退并居中黑线")

    def stop(self, now: float, reason: str) -> VisualStepCommand:
        self.velocity = 0.0
        self._enter("STOPPED", now, reason)
        return VisualStepCommand(mode="step_visual_stopped")

    def distance(self, pose: Tuple[float, float, float]) -> float:
        if self.origin is None:
            return 0.0
        return math.hypot(pose[0] - self.origin[0], pose[1] - self.origin[1])

    def _heading_turn(self, pose: Tuple[float, float, float], tuning: LiveRaceTuning) -> float:
        if not tuning.step_heading_hold_enabled or self.target_yaw is None:
            return 0.0
        error = math.atan2(
            math.sin(self.target_yaw - pose[2]), math.cos(self.target_yaw - pose[2])
        )
        return max(-tuning.step_heading_max_turn_speed, min(
            tuning.step_heading_max_turn_speed, tuning.step_heading_kp * error
        ))

    def update(
        self, now: float, tuning: LiveRaceTuning, *,
        pose: Optional[Tuple[float, float, float]], camera_ok: bool,
        fresh_frame: bool = False, line_valid: bool = False,
        near_error: float = 0.0, mid_error: float = 0.0,
        steering_sign: float = -1.0,
    ) -> VisualStepCommand:
        if self.state in ("IDLE", "DONE", "STOPPED"):
            return VisualStepCommand(mode=f"step_visual_{self.state.lower()}")
        if pose is None or not all(math.isfinite(v) for v in pose):
            return self.stop(now, "偏航/里程计缺失或过期，停车且不触发跳跃")
        # A pulse already started must end at its deadline even if the camera
        # disappears. No new jump may start from a missing camera image.
        if self.state == "JUMP":
            angular = self._heading_turn(pose, tuning)
            if now - self.entered_at < tuning.step_jump_pulse_seconds:
                return VisualStepCommand(tuning.step_jump_forward_speed, angular, True, "step_visual_jump")
            self.origin = pose
            self.found_frames = 0
            self.last_line_valid = False
            self._enter("VISUAL_REACQUIRE", now, "脉冲结束，按已记录偏航前行找线")
        if not camera_ok:
            return self.stop(now, "相机断帧或过期，不能把断帧当丢线")
        if fresh_frame:
            self.last_line_valid = bool(line_valid)
        dt = max(0.0, min(now - self.last_tick, 0.20))
        self.last_tick = now
        age = now - self.entered_at
        if self.state == "VISUAL_ALIGN":
            if fresh_frame:
                centered = (
                    line_valid and math.isfinite(near_error) and math.isfinite(mid_error)
                    and abs((near_error + mid_error) * 0.5) <= tuning.step_align_center_tolerance
                    and abs(mid_error - near_error) <= tuning.step_align_heading_tolerance
                )
                self.aligned_frames = self.aligned_frames + 1 if centered else 0
            timed_out = age >= tuning.step_reverse_align_timeout_seconds
            if self.aligned_frames >= tuning.step_align_confirm_frames or timed_out:
                if not self.last_line_valid:
                    return self.stop(now, "后退对线超时且仍无黑线，结束倒车并停车")
                self.target_yaw = pose[2]
                self.origin = pose
                self.velocity = 0.0
                self.lost_frames = 0
                reason = "对线超时强制退出，记录当前偏航" if timed_out else "黑线居中确认，记录当前偏航"
                self._enter("VISUAL_SPRINT", now, reason)
                return VisualStepCommand(mode="step_visual_sprint_begin")
            angular = 0.0
            if self.last_line_valid and math.isfinite(near_error) and math.isfinite(mid_error):
                # Heading error has the same correction sign in reverse;
                # lateral position correction changes sign with velocity.
                center = (near_error + mid_error) * 0.5
                heading = mid_error - near_error
                lateral = -center if tuning.step_reverse_align_enabled else center
                angular = steering_sign * tuning.step_align_turn_kp * (heading + lateral)
                angular = max(-tuning.step_align_max_turn_speed, min(tuning.step_align_max_turn_speed, angular))
            reverse = -tuning.step_reverse_speed if tuning.step_reverse_align_enabled else 0.0
            return VisualStepCommand(reverse, angular, False, "step_visual_reverse_align")
        angular = self._heading_turn(pose, tuning)
        if self.state == "VISUAL_SPRINT":
            distance = self.distance(pose)
            if age >= tuning.step_sprint_max_seconds or distance >= tuning.step_sprint_max_distance_m:
                return self.stop(now, "冲刺超时/超距仍未满足丢线起跳，停车")
            armed = age >= tuning.step_sprint_min_seconds and distance >= tuning.step_sprint_min_distance_m
            if fresh_frame:
                self.lost_frames = self.lost_frames + 1 if armed and not line_valid else 0
            if self.lost_frames >= tuning.step_line_loss_confirm_frames:
                if not tuning.step_jump_on_loss_enabled:
                    return self.stop(now, "丢线条件已满足，起跳开关关闭，仅停车")
                self._enter("JUMP", now, "有效画面连续丢线，触发一次跳跃脉冲")
                return VisualStepCommand(tuning.step_jump_forward_speed, angular, True, "step_visual_jump")
            self.velocity = min(tuning.step_sprint_speed, self.velocity + tuning.step_sprint_accel * dt)
            return VisualStepCommand(self.velocity, angular, False, "step_visual_sprint")
        if self.state == "VISUAL_REACQUIRE":
            if age >= tuning.step_post_timeout_seconds or self.distance(pose) >= tuning.step_post_max_distance_m:
                return self.stop(now, "落地找线超时/超距，停车")
            if fresh_frame:
                self.found_frames = self.found_frames + 1 if age >= tuning.step_landing_seconds and line_valid else 0
            if self.found_frames >= tuning.step_post_line_confirm_frames:
                self._enter("DONE", now, "落地后连续看见黑线，恢复普通巡线")
                return VisualStepCommand(mode="step_visual_done")
            return VisualStepCommand(tuning.step_post_forward_speed, angular, False, "step_visual_heading_reacquire")
        return VisualStepCommand()


@dataclass(frozen=True)
class LiveTuningFileResult:
    """One non-actuating result from observing the external tuning file."""

    kind: str
    message: str
    tuning: LiveRaceTuning


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def parse_live_tuning_file(
    text: str, current: LiveRaceTuning
) -> Tuple[LiveRaceTuning, str]:
    """Validate one complete external YAML snapshot without mutating state."""
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as error:
        return current, f"YAML 解析失败: {error}"
    if not isinstance(document, Mapping):
        return current, "ICN YAML 顶层必须是参数表"
    node_config = document.get("wheelleg_vision_race")
    if not isinstance(node_config, Mapping):
        return current, "缺少 wheelleg_vision_race 节点参数段"
    ros_parameters = node_config.get("ros__parameters")
    if not isinstance(ros_parameters, Mapping):
        return current, "缺少 ros__parameters 参数段"
    values = ros_parameters.get("live_tuning")
    if not isinstance(values, Mapping):
        return current, "live_tuning 必须是参数表"

    expected = {
        parameter.removeprefix(LIVE_PREFIX)
        for parameter in current.as_parameter_values()
    }
    actual = set(values)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        return current, f"必须完整保存全部参数；缺少={missing}，未知={unknown}"

    changes = {
        f"{LIVE_PREFIX}{name}": values[name]
        for name in expected
    }
    return current.with_updates(changes)


class LiveTuningFileWatcher:
    """Watch the real ICN YAML without publishing ROS or hardware commands."""

    def __init__(self, path: Path = LIVE_TUNING_FILE_PATH) -> None:
        self.path = Path(path)
        self._last_seen_digest: Optional[str] = None

    def poll(self, current: LiveRaceTuning) -> LiveTuningFileResult:
        """Report a changed complete live_tuning section from the real ICN YAML."""
        try:
            if not self.path.exists():
                return LiveTuningFileResult(
                    "rejected", "找不到 ICN 参数文件，保留当前实时参数", current
                )
            text = self.path.read_text(encoding="utf-8")
        except OSError as error:
            return LiveTuningFileResult(
                "rejected", f"文件无法读取或创建：{error}", current
            )

        digest = _sha256_text(text)
        if digest == self._last_seen_digest:
            return LiveTuningFileResult("unchanged", "文件未变化", current)
        self._last_seen_digest = digest

        updated, reason = parse_live_tuning_file(text, current)
        if reason:
            return LiveTuningFileResult(
                "rejected", f"文件未生效：{reason}", current
            )
        return LiveTuningFileResult("loaded", "文件参数已生效", updated)


class SoftwareBuzzerEvents:
    """Record requested cue events before a real ESP32 buzzer transport exists."""

    def emit(
        self, name: str, tuning: LiveRaceTuning, now: float
    ) -> SoftwareBuzzerEvent:
        pattern = tuning.pattern_for(name)
        delivery = "software_only" if tuning.buzzer_enabled else "disabled"
        complete_at = now + pattern.total_seconds if tuning.buzzer_enabled else now
        return SoftwareBuzzerEvent(
            name=name,
            delivery=delivery,
            pattern=pattern,
            complete_at=complete_at,
        )
