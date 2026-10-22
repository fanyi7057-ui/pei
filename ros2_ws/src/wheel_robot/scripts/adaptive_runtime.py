#!/usr/bin/env python3
"""Pure helpers for adaptive line recovery and timed roundabout entry gating."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Deque, Mapping, Optional, Sequence, Tuple

import yaml


@dataclass(frozen=True)
class AdaptiveLogicTuning:
    line_history_frames: int = 5
    line_near_weight: float = 0.55
    line_mid_weight: float = 0.30
    line_far_weight: float = 0.15
    line_trend_gain: float = 1.80
    line_recovery_kp: float = 0.95
    line_recovery_max_turn_speed: float = 0.68
    line_recovery_speed_ratio: float = 0.78
    line_recovery_timeout_seconds: float = 1.50
    line_recovery_min_speed: float = 0.035
    # Short visual flicker keeps almost-normal motion; after timeout, do not stop.
    line_recovery_bridge_seconds: float = 0.22
    line_recovery_bridge_speed_ratio: float = 0.92
    line_recovery_persistent_enabled: bool = True
    line_recovery_persistent_speed: float = 0.07
    line_recovery_persistent_turn_start: float = 0.18
    line_recovery_persistent_turn_max: float = 0.42
    line_recovery_persistent_turn_ramp_seconds: float = 1.50
    line_recovery_direction_deadband: float = 0.04

    roundabout_dual_center_x_min: float = 0.30
    roundabout_dual_center_x_max: float = 0.70
    roundabout_dual_left_x_min: float = 0.02
    roundabout_dual_left_x_max: float = 0.46
    roundabout_dual_center_min_components: int = 2
    roundabout_dual_left_min_components: int = 2
    roundabout_dual_center_min_span_ratio: float = 0.07
    roundabout_dual_left_min_span_ratio: float = 0.03
    roundabout_dual_max_vertical_gap_ratio: float = 0.32
    # Strict geometry remains a diagnostic only.  Marker #2 may also use a
    # strong raw left branch after marker #1 + center-corridor evidence, because
    # the real entrance can overlap the center chain under perspective.
    roundabout_dual_min_horizontal_gap_ratio: float = 0.08
    # V5: only a real exit-like shape can arm the ring sequence. Weak one-block
    # fragments like the V4 false trigger (c=1/l=1) are rejected.
    roundabout_exit_marker_center_min_components: int = 2
    roundabout_exit_marker_center_min_span_ratio: float = 0.06
    roundabout_exit_marker_left_min_components: int = 2
    roundabout_exit_marker_left_min_span_ratio: float = 0.08
    roundabout_exit_marker_confirm_frames: int = 2
    # Legacy two-marker fields remain loadable for compatibility, but V5 no
    # longer waits for branch-clear/corridor/new-branch before entering.
    roundabout_marker1_center_follow_seconds: float = 1.40
    roundabout_marker1_raw_confirm_frames: int = 1
    roundabout_marker_confirm_frames: int = 1
    roundabout_marker_clear_frames: int = 3
    roundabout_marker_between_center_frames: int = 1
    roundabout_marker_min_separation_seconds: float = 2.00
    roundabout_marker_min_separation_distance_m: float = 0.0
    roundabout_marker2_raw_min_components: int = 3
    roundabout_marker2_raw_min_span_ratio: float = 0.14
    # Real exit confirmed -> keep following the MAIN line for a timed pass window,
    # while branch events are ignored; then direct FAST_ENTRY. Legacy straight fields
    # stay loadable for backward-compatible YAMLs but no longer force w=0.
    roundabout_marker1_straight_seconds: float = 2.00
    roundabout_marker1_straight_speed: float = 0.25
    roundabout_marker1_straight_max_turn_speed: float = 0.0
    roundabout_marker1_follow_line_enabled: bool = True
    roundabout_marker1_follow_max_turn_speed: float = 0.28
    roundabout_marker1_branch_left_turn_cap: float = 0.10
    roundabout_marker1_gap_bridge_seconds: float = 0.45
    roundabout_marker1_gap_speed_ratio: float = 0.90
    roundabout_marker1_pass_loss_min_speed: float = 0.32
    roundabout_marker1_post_hold_enabled: bool = False
    roundabout_marker1_post_hold_speed_cap: float = 0.40
    roundabout_marker1_post_hold_loss_speed_cap: float = 0.12
    roundabout_marker1_post_hold_max_turn_speed: float = 0.15
    roundabout_ignore_after_lap_start_seconds: float = 0.0
    # Only the first roundabout search is globally armed after race start.
    roundabout_initial_arm_delay_seconds: float = 10.0

    # Output smoothing for adaptive patrol and ring FOLLOW.
    motion_smoothing_enabled: bool = True
    motion_linear_accel_up: float = 0.34
    motion_linear_decel: float = 0.90
    motion_angular_slew_normal: float = 3.0
    motion_angular_slew_roundabout: float = 2.4
    # Reversing steering direction is deliberately slower than building turn in
    # the same direction; this removes left-right snapping without making entry lazy.
    motion_angular_reverse_slew: float = 1.80
    motion_angular_target_alpha: float = 0.60
    motion_angular_deadband: float = 0.015

    roundabout_entry_min_search_seconds: float = 0.60
    roundabout_entry_max_search_seconds: float = 1.10
    roundabout_entry_forward_speed: float = 0.30
    roundabout_entry_left_bias: float = 0.20
    roundabout_entry_min_left_turn_speed: float = 0.45
    roundabout_entry_max_turn_speed: float = 0.70
    roundabout_entry_reacquire_frames: int = 3
    # FAST_ENTRY -> FOLLOW handoff guard: if the line blinks immediately after
    # entry, keep moving with a mild LEFT arc instead of inheriting stale/right trend.
    roundabout_follow_entry_guard_seconds: float = 1.20
    roundabout_follow_entry_guard_speed: float = 0.22
    roundabout_follow_entry_guard_min_left_turn: float = 0.12
    roundabout_follow_entry_guard_max_left_turn: float = 0.30

    roundabout_follow_steering_gain: float = 1.10
    # V10: a single isolated dashed block inside the ring is still a useful
    # local target even when it does not cross enough horizontal scan bands.
    roundabout_single_block_rescue_enabled: bool = True
    roundabout_single_block_speed: float = 0.18
    roundabout_single_block_kp: float = 0.78
    roundabout_single_block_max_turn_speed: float = 0.45

    # V10: once the car has already turned far enough around the ring, a lost
    # line near the exit must keep searching LEFT instead of resurrecting an
    # old/right trend.
    roundabout_exit_loss_left_arm_turn_deg: float = 200.0
    roundabout_exit_loss_speed: float = 0.24
    roundabout_exit_loss_left_turn: float = 0.30
    # Keep a left search active across ROUNDABOUT_EXIT -> SEEK_CROSSWALK until
    # vision has seen a trustworthy road line for several consecutive frames.
    roundabout_exit_reacquire_confirm_frames: int = 2
    roundabout_exit_reacquire_min_confidence: float = 0.35
    roundabout_exit_reacquire_min_bands: int = 2

    # V11: from ~330deg onward the car is close to the entry heading again.
    # Use the gyro-integrated entry heading as the primary exit reference so a
    # left branch cannot physically pull the chassis back into the roundabout.
    roundabout_exit_heading_lock_enabled: bool = True
    roundabout_exit_heading_lock_arm_deg: float = 330.0
    roundabout_exit_heading_target_deg: float = 360.0
    roundabout_exit_heading_tolerance_deg: float = 8.0
    roundabout_exit_heading_kp: float = 0.010
    roundabout_exit_heading_lock_speed: float = 0.24
    roundabout_exit_heading_max_left_turn: float = 0.30
    roundabout_exit_heading_max_right_turn: float = 0.12
    roundabout_exit_heading_visual_weight: float = 0.35
    # If vision suddenly asks for a large LEFT turn near/after the exit, treat it
    # as the ring branch for one second and steer only toward the saved heading.
    roundabout_exit_left_branch_trigger_turn: float = 0.18
    roundabout_exit_left_branch_hold_seconds: float = 1.50
    # Keep gyro heading available briefly after ROUNDABOUT_EXIT -> SEEK_CROSSWALK.
    # Normal line following remains active; this window only enables the one-shot
    # suspicious-left suppression described above.
    roundabout_exit_heading_post_exit_seconds: float = 3.00

    # Ring blind-line recovery belongs to adaptive_tuning.yaml as well. Keep a
    # small LEFT floor so it never grabs the right branch, but cap it so a
    # short line loss cannot create the old +0.38rad/s forced-left lurch.
    roundabout_blind_recovery_speed: float = 0.12
    roundabout_blind_recovery_left_turn: float = 0.10
    roundabout_blind_recovery_max_turn_speed: float = 0.45

    # The orientation quaternion in the real log stayed near 0deg while the car
    # physically circled. Use gyro_z integration from the true entry instead.
    # This is STATE-ONLY; it never enters the steering equation.
    roundabout_gyro_exit_enabled: bool = True
    roundabout_gyro_arm_turn_deg: float = 270.0
    roundabout_gyro_target_turn_deg: float = 360.0
    roundabout_gyro_return_tolerance_deg: float = 15.0
    roundabout_gyro_return_confirm_frames: int = 3
    roundabout_gyro_require_line_valid: bool = True
    # A fresh gyro that has actually crossed a full turn must not be allowed to
    # miss the exit forever just because the visual line disappears at the junction.
    roundabout_gyro_force_exit_after_target_enabled: bool = True
    roundabout_gyro_force_exit_margin_deg: float = 5.0
    roundabout_gyro_force_exit_confirm_frames: int = 2
    roundabout_gyro_imu_timeout_seconds: float = 0.50
    roundabout_gyro_deadband_deg_s: float = 1.00
    roundabout_gyro_scale: float = 1.00
    roundabout_gyro_max_rate_deg_s: float = 240.0
    roundabout_gyro_max_integration_dt_seconds: float = 0.10
    # When gyro is enabled, visual fork exit is accepted only after >= arm angle.
    roundabout_exit_require_gyro_arm: bool = True

    roundabout_exit_arm_seconds: float = 4.00
    # The photographed exit is a fork: one broad dashed row plus one diagonal
    # dashed branch. It is NOT an alternating left/right zig-zag.
    roundabout_exit_fork_line_tolerance_ratio: float = 0.04
    roundabout_exit_fork_min_main_components: int = 3
    roundabout_exit_fork_min_branch_components: int = 3
    roundabout_exit_fork_min_main_x_span_ratio: float = 0.28
    roundabout_exit_fork_max_main_y_span_ratio: float = 0.14
    roundabout_exit_fork_min_branch_y_span_ratio: float = 0.18
    roundabout_exit_fork_min_angle_deg: float = 20.0
    roundabout_exit_fork_junction_margin_ratio: float = 0.18
    roundabout_exit_fork_confirm_frames: int = 2

    # These remain as a timeout-only fallback when the visual exit is missed.
    roundabout_exit_commit_seconds: float = 1.20
    roundabout_exit_speed: float = 0.20
    roundabout_exit_left_bias: float = 0.10
    roundabout_exit_max_turn_speed: float = 0.42

    roundabout_gyro_exit_commit_seconds: float = 1.00
    roundabout_gyro_exit_speed: float = 0.25
    roundabout_gyro_exit_left_bias: float = 0.00
    roundabout_gyro_exit_max_turn_speed: float = 0.12

    roundabout_follow_timeout_seconds: float = 28.0
    roundabout_ignore_after_exit_seconds: float = 25.0
    roundabout_post_exit_guard_seconds: float = 2.50
    roundabout_post_exit_guard_max_seconds: float = 4.00
    roundabout_post_exit_reacquire_frames: int = 4
    roundabout_post_exit_speed_cap: float = 0.30
    roundabout_post_exit_min_speed: float = 0.18
    roundabout_post_exit_loss_left_turn: float = 0.50
    # V11.4 smooth post-exit search: build LEFT authority progressively, then
    # blend back into ordinary visual line following after reacquisition.
    roundabout_post_exit_loss_left_turn_start: float = 0.35
    roundabout_post_exit_loss_left_turn_max: float = 0.55
    roundabout_post_exit_turn_ramp_seconds: float = 0.40
    roundabout_post_exit_reacquire_blend_seconds: float = 0.30
    roundabout_post_exit_search_speed: float = 0.24
    # V9: during the post-exit guard, even a valid but branch-biased line may not
    # command a right turn. Keep a small left arc until the main line is stably reacquired.
    roundabout_post_exit_min_left_turn_speed: float = 0.12
    roundabout_post_exit_max_left_turn_speed: float = 0.22
    roundabout_post_exit_max_right_turn_speed: float = 0.00

    # After the first crosswalk is confirmed, do not arm the second-lap
    # roundabout detector until this many seconds have elapsed.
    second_roundabout_arm_delay_after_first_crosswalk_seconds: float = 15.0

    crosswalk_far_y_top_ratio: float = 0.18
    crosswalk_far_y_bottom_ratio: float = 0.66
    crosswalk_far_min_runs: int = 5
    crosswalk_far_min_good_rows: int = 3
    crosswalk_far_min_span_ratio: float = 0.30
    crosswalk_far_min_run_width_px: int = 3
    crosswalk_far_max_run_width_ratio: float = 0.085
    crosswalk_row_occupancy: float = 0.32
    crosswalk_confirm_frames: int = 2
    crosswalk_detect_delay_after_roundabout: float = 0.80
    crosswalk_approach_distance_m: float = 0.22
    crosswalk_approach_min_seconds: float = 0.30
    crosswalk_approach_fallback_seconds: float = 3.20

    def validate(self) -> str:
        if not 3 <= int(self.line_history_frames) <= 12:
            return "line_history_frames 必须在 3..12"
        weights = (self.line_near_weight, self.line_mid_weight, self.line_far_weight)
        if any((not math.isfinite(v) or v < 0.0) for v in weights):
            return "巡线 near/mid/far 权重必须是有限非负数"
        if sum(weights) <= 1e-9:
            return "巡线 near/mid/far 权重之和必须大于 0"
        if not isinstance(self.line_recovery_persistent_enabled, bool):
            return "line_recovery_persistent_enabled 必须是 true/false"
        if not isinstance(self.roundabout_marker1_post_hold_enabled, bool):
            return "roundabout_marker1_post_hold_enabled 必须是 true/false"
        if not isinstance(self.roundabout_gyro_exit_enabled, bool):
            return "roundabout_gyro_exit_enabled 必须是 true/false"
        if not isinstance(self.roundabout_gyro_require_line_valid, bool):
            return "roundabout_gyro_require_line_valid 必须是 true/false"
        if not isinstance(self.roundabout_marker1_follow_line_enabled, bool):
            return "roundabout_marker1_follow_line_enabled 必须是 true/false"
        if not isinstance(self.roundabout_single_block_rescue_enabled, bool):
            return "roundabout_single_block_rescue_enabled 必须是 true/false"
        if not isinstance(self.roundabout_exit_heading_lock_enabled, bool):
            return "roundabout_exit_heading_lock_enabled 必须是 true/false"
        if not isinstance(self.roundabout_gyro_force_exit_after_target_enabled, bool):
            return "roundabout_gyro_force_exit_after_target_enabled 必须是 true/false"
        if not isinstance(self.roundabout_exit_require_gyro_arm, bool):
            return "roundabout_exit_require_gyro_arm 必须是 true/false"
        if not isinstance(self.motion_smoothing_enabled, bool):
            return "motion_smoothing_enabled 必须是 true/false"
        positive = (
            self.line_trend_gain,
            self.line_recovery_kp,
            self.line_recovery_max_turn_speed,
            self.line_recovery_timeout_seconds,
            self.line_recovery_persistent_speed,
            self.line_recovery_persistent_turn_start,
            self.line_recovery_persistent_turn_max,
            self.line_recovery_persistent_turn_ramp_seconds,
            self.roundabout_marker_min_separation_seconds,
            self.roundabout_single_block_speed,
            self.roundabout_single_block_kp,
            self.roundabout_single_block_max_turn_speed,
            self.roundabout_exit_loss_speed,
            self.roundabout_exit_loss_left_turn,
            self.roundabout_exit_heading_target_deg,
            self.roundabout_exit_heading_kp,
            self.roundabout_exit_heading_lock_speed,
            self.roundabout_exit_heading_max_left_turn,
            self.roundabout_exit_heading_max_right_turn,
            self.roundabout_exit_left_branch_hold_seconds,
            self.roundabout_exit_heading_post_exit_seconds,
            self.motion_linear_accel_up,
            self.motion_linear_decel,
            self.motion_angular_slew_normal,
            self.motion_angular_slew_roundabout,
            self.motion_angular_reverse_slew,
            self.roundabout_entry_max_search_seconds,
            self.roundabout_follow_steering_gain,
            self.roundabout_blind_recovery_speed,
            self.roundabout_blind_recovery_max_turn_speed,
            self.roundabout_gyro_arm_turn_deg,
            self.roundabout_gyro_target_turn_deg,
            self.roundabout_gyro_return_tolerance_deg,
            self.roundabout_gyro_imu_timeout_seconds,
            self.roundabout_gyro_scale,
            self.roundabout_gyro_max_rate_deg_s,
            self.roundabout_gyro_max_integration_dt_seconds,
            self.roundabout_gyro_force_exit_confirm_frames,
            self.roundabout_exit_commit_seconds,
            self.roundabout_gyro_exit_commit_seconds,
            self.roundabout_gyro_exit_speed,
            self.roundabout_gyro_exit_max_turn_speed,
            self.roundabout_follow_timeout_seconds,
            self.crosswalk_approach_fallback_seconds,
        )
        if any((not math.isfinite(v) or v <= 0.0) for v in positive):
            return "趋势/环岛/横道的正数参数必须是有限正数"
        nonnegative = (
            self.line_recovery_speed_ratio,
            self.line_recovery_min_speed,
            self.line_recovery_bridge_seconds,
            self.line_recovery_bridge_speed_ratio,
            self.line_recovery_direction_deadband,
            self.roundabout_marker_min_separation_distance_m,
            self.roundabout_marker1_center_follow_seconds,
            self.roundabout_exit_marker_center_min_span_ratio,
            self.roundabout_exit_marker_left_min_span_ratio,
            self.roundabout_marker2_raw_min_span_ratio,
            self.roundabout_marker1_straight_seconds,
            self.roundabout_marker1_straight_speed,
            self.roundabout_marker1_straight_max_turn_speed,
            self.roundabout_marker1_follow_max_turn_speed,
            self.roundabout_marker1_branch_left_turn_cap,
            self.roundabout_marker1_gap_bridge_seconds,
            self.roundabout_marker1_gap_speed_ratio,
            self.roundabout_marker1_pass_loss_min_speed,
            self.roundabout_marker1_post_hold_speed_cap,
            self.roundabout_marker1_post_hold_loss_speed_cap,
            self.roundabout_marker1_post_hold_max_turn_speed,
            self.roundabout_ignore_after_lap_start_seconds,
            self.roundabout_initial_arm_delay_seconds,
            self.motion_angular_target_alpha,
            self.motion_angular_deadband,
            self.roundabout_entry_min_search_seconds,
            self.roundabout_entry_forward_speed,
            self.roundabout_entry_left_bias,
            self.roundabout_entry_min_left_turn_speed,
            self.roundabout_entry_max_turn_speed,
            self.roundabout_follow_entry_guard_seconds,
            self.roundabout_follow_entry_guard_speed,
            self.roundabout_follow_entry_guard_min_left_turn,
            self.roundabout_follow_entry_guard_max_left_turn,
            self.roundabout_blind_recovery_left_turn,
            self.roundabout_exit_heading_lock_arm_deg,
            self.roundabout_exit_heading_tolerance_deg,
            self.roundabout_exit_heading_visual_weight,
            self.roundabout_exit_left_branch_trigger_turn,
            self.roundabout_gyro_deadband_deg_s,
            self.roundabout_gyro_force_exit_margin_deg,
            self.roundabout_exit_arm_seconds,
            self.roundabout_exit_speed,
            self.roundabout_exit_left_bias,
            self.roundabout_exit_max_turn_speed,
            self.roundabout_gyro_exit_left_bias,
            self.roundabout_ignore_after_exit_seconds,
            self.roundabout_post_exit_guard_seconds,
            self.roundabout_post_exit_guard_max_seconds,
            self.roundabout_post_exit_speed_cap,
            self.roundabout_post_exit_min_speed,
            self.roundabout_post_exit_loss_left_turn,
            self.roundabout_post_exit_loss_left_turn_start,
            self.roundabout_post_exit_loss_left_turn_max,
            self.roundabout_post_exit_turn_ramp_seconds,
            self.roundabout_post_exit_reacquire_blend_seconds,
            self.roundabout_post_exit_search_speed,
            self.roundabout_post_exit_min_left_turn_speed,
            self.roundabout_post_exit_max_left_turn_speed,
            self.roundabout_post_exit_max_right_turn_speed,
            self.second_roundabout_arm_delay_after_first_crosswalk_seconds,
            self.crosswalk_detect_delay_after_roundabout,
            self.crosswalk_approach_distance_m,
            self.crosswalk_approach_min_seconds,
        )
        if any((not math.isfinite(v) or v < 0.0) for v in nonnegative):
            return "趋势/环岛/横道的非负参数必须是有限非负数"
        if not 0.0 < float(self.motion_angular_target_alpha) <= 1.0:
            return "motion_angular_target_alpha 必须在 (0,1]"
        if not 0.0 <= float(self.roundabout_marker1_gap_speed_ratio) <= 1.0:
            return "roundabout_marker1_gap_speed_ratio 必须在 0..1"
        ratios = {
            "roundabout_dual_center_x_min": self.roundabout_dual_center_x_min,
            "roundabout_dual_center_x_max": self.roundabout_dual_center_x_max,
            "roundabout_dual_left_x_min": self.roundabout_dual_left_x_min,
            "roundabout_dual_left_x_max": self.roundabout_dual_left_x_max,
            "roundabout_dual_center_min_span_ratio": self.roundabout_dual_center_min_span_ratio,
            "roundabout_dual_left_min_span_ratio": self.roundabout_dual_left_min_span_ratio,
            "roundabout_dual_max_vertical_gap_ratio": self.roundabout_dual_max_vertical_gap_ratio,
            "roundabout_dual_min_horizontal_gap_ratio": self.roundabout_dual_min_horizontal_gap_ratio,
            "roundabout_exit_marker_center_min_span_ratio": self.roundabout_exit_marker_center_min_span_ratio,
            "roundabout_exit_marker_left_min_span_ratio": self.roundabout_exit_marker_left_min_span_ratio,
            "roundabout_marker2_raw_min_span_ratio": self.roundabout_marker2_raw_min_span_ratio,
            "roundabout_exit_fork_line_tolerance_ratio": self.roundabout_exit_fork_line_tolerance_ratio,
            "roundabout_exit_fork_min_main_x_span_ratio": self.roundabout_exit_fork_min_main_x_span_ratio,
            "roundabout_exit_fork_max_main_y_span_ratio": self.roundabout_exit_fork_max_main_y_span_ratio,
            "roundabout_exit_fork_min_branch_y_span_ratio": self.roundabout_exit_fork_min_branch_y_span_ratio,
            "roundabout_exit_fork_junction_margin_ratio": self.roundabout_exit_fork_junction_margin_ratio,
            "crosswalk_far_y_top_ratio": self.crosswalk_far_y_top_ratio,
            "crosswalk_far_y_bottom_ratio": self.crosswalk_far_y_bottom_ratio,
            "crosswalk_far_min_span_ratio": self.crosswalk_far_min_span_ratio,
            "crosswalk_far_max_run_width_ratio": self.crosswalk_far_max_run_width_ratio,
            "crosswalk_row_occupancy": self.crosswalk_row_occupancy,
        }
        for name, value in ratios.items():
            if not 0.0 <= float(value) <= 1.0:
                return f"{name} 必须在 0..1"
        if not self.roundabout_dual_center_x_min < self.roundabout_dual_center_x_max:
            return "环岛中央区域 x_min 必须小于 x_max"
        if not self.roundabout_dual_left_x_min < self.roundabout_dual_left_x_max:
            return "环岛左侧区域 x_min 必须小于 x_max"
        if not 5.0 <= float(self.roundabout_exit_fork_min_angle_deg) <= 90.0:
            return "环岛分叉出口夹角必须在 5..90 度"
        if not 180.0 <= float(self.roundabout_gyro_arm_turn_deg) <= 355.0:
            return "roundabout_gyro_arm_turn_deg 必须在 180..355 度"
        if not 270.0 <= float(self.roundabout_gyro_target_turn_deg) <= 450.0:
            return "roundabout_gyro_target_turn_deg 必须在 270..450 度"
        if self.roundabout_gyro_arm_turn_deg >= self.roundabout_gyro_target_turn_deg:
            return "roundabout_gyro_arm_turn_deg 必须小于 target_turn_deg"
        if not 1.0 <= float(self.roundabout_gyro_return_tolerance_deg) <= 60.0:
            return "roundabout_gyro_return_tolerance_deg 必须在 1..60 度"
        if not 0.05 <= float(self.roundabout_gyro_imu_timeout_seconds) <= 2.0:
            return "roundabout_gyro_imu_timeout_seconds 必须在 0.05..2.0 秒"
        if not 0.0 <= float(self.roundabout_gyro_deadband_deg_s) <= 30.0:
            return "roundabout_gyro_deadband_deg_s 必须在 0..30 deg/s"
        if not 0.0 <= float(self.roundabout_exit_loss_left_arm_turn_deg) <= 360.0:
            return "roundabout_exit_loss_left_arm_turn_deg 必须在 0..360 deg"
        if not 0.0 <= float(self.roundabout_exit_heading_lock_arm_deg) <= 360.0:
            return "roundabout_exit_heading_lock_arm_deg 必须在 0..360 deg"
        if not 270.0 <= float(self.roundabout_exit_heading_target_deg) <= 450.0:
            return "roundabout_exit_heading_target_deg 必须在 270..450 deg"
        if self.roundabout_exit_heading_lock_arm_deg >= self.roundabout_exit_heading_target_deg:
            return "roundabout_exit_heading_lock_arm_deg 必须小于 heading_target_deg"
        if not 0.0 < float(self.roundabout_exit_heading_tolerance_deg) <= 30.0:
            return "roundabout_exit_heading_tolerance_deg 必须在 (0,30] deg"
        if not 0.0 <= float(self.roundabout_exit_heading_visual_weight) <= 1.0:
            return "roundabout_exit_heading_visual_weight 必须在 0..1"
        if not 0.0 <= float(self.roundabout_exit_left_branch_trigger_turn) <= 1.5:
            return "roundabout_exit_left_branch_trigger_turn 必须在 0..1.5"
        if not 0.0 <= float(self.roundabout_single_block_max_turn_speed) <= 1.5:
            return "roundabout_single_block_max_turn_speed 必须在 0..1.5"
        if not 0.0 <= float(self.roundabout_exit_loss_left_turn) <= 1.5:
            return "roundabout_exit_loss_left_turn 必须在 0..1.5"
        if not 0.0 <= float(self.roundabout_exit_reacquire_min_confidence) <= 1.0:
            return "roundabout_exit_reacquire_min_confidence 必须在 0..1"
        if not 10.0 <= float(self.roundabout_gyro_max_rate_deg_s) <= 1000.0:
            return "roundabout_gyro_max_rate_deg_s 必须在 10..1000 deg/s"
        if not 0.01 <= float(self.roundabout_gyro_max_integration_dt_seconds) <= 0.50:
            return "roundabout_gyro_max_integration_dt_seconds 必须在 0.01..0.50 秒"
        if self.line_recovery_persistent_turn_start > self.line_recovery_persistent_turn_max:
            return "持续找线起始转向不能大于最大转向"
        if not 0.0 <= float(self.line_recovery_bridge_speed_ratio) <= 1.5:
            return "line_recovery_bridge_speed_ratio 必须在 0..1.5"
        if not 0.0 <= float(self.line_recovery_direction_deadband) <= 0.5:
            return "line_recovery_direction_deadband 必须在 0..0.5"
        if not 0.0 <= float(self.motion_angular_deadband) <= 0.20:
            return "motion_angular_deadband 必须在 0..0.20"
        if self.roundabout_blind_recovery_left_turn > self.roundabout_blind_recovery_max_turn_speed:
            return "环内丢线最小左转不能大于最大左转"
        if self.roundabout_entry_min_search_seconds > self.roundabout_entry_max_search_seconds:
            return "环岛入环最短保持时间不能大于最大保持时间"
        if self.roundabout_entry_min_left_turn_speed > self.roundabout_entry_max_turn_speed:
            return "环岛入环最小左转不能大于最大转向"
        if self.roundabout_follow_entry_guard_min_left_turn > self.roundabout_follow_entry_guard_max_left_turn:
            return "入环交接保护最小左转不能大于最大左转"
        if self.roundabout_post_exit_guard_seconds > self.roundabout_post_exit_guard_max_seconds:
            return "出环最短保护时间不能大于最大保护时间"
        if self.roundabout_post_exit_loss_left_turn_start > self.roundabout_post_exit_loss_left_turn_max:
            return "出环找线起始左转不能大于最大左转"
        if self.roundabout_post_exit_loss_left_turn_max > 1.5:
            return "出环找线最大左转必须 <= 1.5 rad/s"
        if not self.crosswalk_far_y_top_ratio < self.crosswalk_far_y_bottom_ratio:
            return "横道远距 ROI 上边界必须小于下边界"
        ints = (
            self.roundabout_dual_center_min_components,
            self.roundabout_dual_left_min_components,
            self.roundabout_exit_marker_center_min_components,
            self.roundabout_exit_marker_left_min_components,
            self.roundabout_exit_marker_confirm_frames,
            self.roundabout_marker1_raw_confirm_frames,
            self.roundabout_marker2_raw_min_components,
            self.roundabout_marker_confirm_frames,
            self.roundabout_marker_clear_frames,
            self.roundabout_marker_between_center_frames,
            self.roundabout_entry_reacquire_frames,
            self.roundabout_post_exit_reacquire_frames,
            self.roundabout_exit_reacquire_confirm_frames,
            self.roundabout_exit_reacquire_min_bands,
            self.roundabout_gyro_return_confirm_frames,
            self.roundabout_exit_fork_min_main_components,
            self.roundabout_exit_fork_min_branch_components,
            self.roundabout_exit_fork_confirm_frames,
            self.crosswalk_far_min_runs,
            self.crosswalk_far_min_good_rows,
            self.crosswalk_far_min_run_width_px,
            self.crosswalk_confirm_frames,
        )
        if any(int(v) < 1 for v in ints):
            return "帧数、块数和条纹数量必须 >= 1"
        return ""

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> Tuple["AdaptiveLogicTuning", str]:
        expected = set(cls.__dataclass_fields__)
        missing = sorted(expected - set(values))
        unknown = sorted(set(values) - expected)
        if missing or unknown:
            return cls(), f"adaptive_logic 必须完整保存全部参数；缺少={missing}，未知={unknown}"
        try:
            converted = cls(**{name: values[name] for name in expected})
        except (TypeError, ValueError) as error:
            return cls(), f"adaptive_logic 参数类型错误: {error}"
        reason = converted.validate()
        return (converted, reason)


@dataclass(frozen=True)
class DashedMainLineEstimate:
    valid: bool = False
    error: float = 0.0
    near_error: float = 0.0
    mid_error: float = 0.0
    far_error: float = 0.0
    confidence: float = 0.0
    count: int = 0
    span_ratio: float = 0.0


def fit_dashed_main_line(
    points: Sequence[Tuple[float, float]],
    width: float,
    height: float,
    min_components: int = 2,
    min_span_ratio: float = 0.055,
) -> DashedMainLineEstimate:
    """Fit separated post-exit main-road blocks into one virtual line.

    The caller is responsible for removing the independent LEFT roundabout
    branch before passing points here.  The fitted line is used only after the
    roundabout has already been exited, so it cannot change entry detection.
    """
    w = float(width)
    h = float(height)
    clean = [
        (float(x), float(y)) for x, y in points
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    if w <= 1.0 or h <= 1.0 or len(clean) < max(2, int(min_components)):
        return DashedMainLineEstimate(count=len(clean))

    ys = [p[1] for p in clean]
    xs = [p[0] for p in clean]
    span_ratio = (max(ys) - min(ys)) / h
    if span_ratio < float(min_span_ratio):
        return DashedMainLineEstimate(count=len(clean), span_ratio=span_ratio)

    mean_y = sum(ys) / len(ys)
    mean_x = sum(xs) / len(xs)
    denom = sum((y - mean_y) ** 2 for y in ys)
    slope = (
        sum((y - mean_y) * (x - mean_x) for x, y in clean) / denom
        if denom > 1e-9 else 0.0
    )
    max_slope = 1.15 * w / h
    slope = max(-max_slope, min(max_slope, slope))
    intercept = mean_x - slope * mean_y
    min_x = min(xs) - 0.10 * w
    max_x = max(xs) + 0.10 * w

    def predict_error(y_ratio: float) -> float:
        x = intercept + slope * (float(y_ratio) * h)
        x = max(min_x, min(max_x, x))
        x = max(0.06 * w, min(0.94 * w, x))
        return max(-1.0, min(1.0, (x - 0.5 * w) / (0.5 * w)))

    near = predict_error(0.84)
    mid = predict_error(0.60)
    far = predict_error(0.38)
    error = max(-1.0, min(1.0, 0.62 * near + 0.28 * mid + 0.10 * far))
    residual = math.sqrt(
        sum((x - (intercept + slope * y)) ** 2 for x, y in clean) / len(clean)
    ) / w
    count_score = min(1.0, len(clean) / max(3.0, float(min_components + 2)))
    span_score = min(1.0, span_ratio / max(float(min_span_ratio) * 2.5, 1e-6))
    fit_score = max(0.0, min(1.0, 1.0 - residual / 0.10))
    confidence = max(0.0, min(1.0, 0.38 * count_score + 0.34 * span_score + 0.28 * fit_score))
    return DashedMainLineEstimate(
        valid=True, error=error, near_error=near, mid_error=mid, far_error=far,
        confidence=confidence, count=len(clean), span_ratio=span_ratio,
    )


def parse_adaptive_tuning_text(text: str, current: Optional[AdaptiveLogicTuning] = None) -> Tuple[AdaptiveLogicTuning, str]:
    current = current or AdaptiveLogicTuning()
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as error:
        return current, f"YAML 解析失败: {error}"
    if not isinstance(doc, Mapping):
        return current, "YAML 顶层必须是参数表"
    values = doc.get("adaptive_logic")
    if not isinstance(values, Mapping):
        return current, "缺少 adaptive_logic 参数段"
    updated, reason = AdaptiveLogicTuning.from_mapping(values)
    if reason:
        return current, reason
    return updated, ""


def load_adaptive_tuning_file(path: Path, current: Optional[AdaptiveLogicTuning] = None) -> Tuple[AdaptiveLogicTuning, str]:
    current = current or AdaptiveLogicTuning()
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as error:
        return current, f"无法读取 adaptive_tuning.yaml: {error}"
    return parse_adaptive_tuning_text(text, current)


def roundabout_exit_heading_turn(
    current_turn_deg: float,
    target_turn_deg: float,
    tolerance_deg: float,
    kp: float,
    max_left_turn: float,
    max_right_turn: float,
) -> float:
    """Return gyro-heading steering near the roundabout exit.

    Positive angular velocity means LEFT in the race controller.  The function
    deliberately drives the accumulated turn toward the saved entry heading
    instead of freezing the chassis at whatever angle first crossed the lock.
    """
    values = (
        current_turn_deg, target_turn_deg, tolerance_deg, kp,
        max_left_turn, max_right_turn,
    )
    if not all(math.isfinite(float(v)) for v in values):
        return 0.0
    error_deg = float(target_turn_deg) - float(current_turn_deg)
    if abs(error_deg) <= max(0.0, float(tolerance_deg)):
        return 0.0
    command = float(kp) * error_deg
    return max(
        -abs(float(max_right_turn)),
        min(abs(float(max_left_turn)), command),
    )


def post_exit_search_turn(
    start_turn: float,
    max_turn: float,
    elapsed_seconds: float,
    ramp_seconds: float,
) -> float:
    """Smoothly build positive (LEFT) steering after a roundabout exit."""
    start = max(0.0, float(start_turn))
    maximum = max(start, float(max_turn))
    elapsed = max(0.0, float(elapsed_seconds))
    duration = max(0.0, float(ramp_seconds))
    if duration <= 1e-9:
        return maximum
    ratio = max(0.0, min(1.0, elapsed / duration))
    return start + (maximum - start) * ratio


def post_exit_reacquire_turn(
    search_turn: float,
    visual_turn: float,
    elapsed_seconds: float,
    blend_seconds: float,
) -> float:
    """Blend steering from the LEFT search arc back to visual tracking."""
    elapsed = max(0.0, float(elapsed_seconds))
    duration = max(0.0, float(blend_seconds))
    if duration <= 1e-9:
        return float(visual_turn)
    ratio = max(0.0, min(1.0, elapsed / duration))
    return (1.0 - ratio) * float(search_turn) + ratio * float(visual_turn)


class WeightedLineTrend:
    """Remember frame-to-frame line position and extrapolate into short line loss."""

    def __init__(self, max_frames: int = 5) -> None:
        self.max_frames = max(3, int(max_frames))
        self.samples: Deque[Tuple[float, float]] = deque(maxlen=self.max_frames)
        self.last_seen_time = 0.0

    def reconfigure(self, max_frames: int) -> None:
        max_frames = max(3, int(max_frames))
        if max_frames == self.max_frames:
            return
        self.max_frames = max_frames
        self.samples = deque(list(self.samples)[-max_frames:], maxlen=max_frames)

    def reset(self) -> None:
        self.samples.clear()
        self.last_seen_time = 0.0

    def observe(
        self,
        now: float,
        near_error: float,
        mid_error: float,
        far_error: float,
        tuning: AdaptiveLogicTuning,
    ) -> float:
        total = tuning.line_near_weight + tuning.line_mid_weight + tuning.line_far_weight
        center = (
            tuning.line_near_weight * float(near_error)
            + tuning.line_mid_weight * float(mid_error)
            + tuning.line_far_weight * float(far_error)
        ) / max(total, 1e-9)
        center = max(-1.5, min(1.5, center))
        self.samples.append((float(now), center))
        self.last_seen_time = float(now)
        return center

    def estimate(self, loss_age: float, tuning: AdaptiveLogicTuning) -> Optional[Tuple[float, float, float]]:
        if not self.samples:
            return None
        values = [value for _time, value in self.samples]
        n = len(values)
        recency = [float(i + 1) for i in range(n)]
        position = sum(w * v for w, v in zip(recency, values)) / sum(recency)

        if n >= 2:
            diffs = [values[i] - values[i - 1] for i in range(1, n)]
            diff_weights = [float(i + 1) for i in range(len(diffs))]
            trend = sum(w * d for w, d in zip(diff_weights, diffs)) / sum(diff_weights)
        else:
            trend = 0.0

        age_scale = 1.0 + min(
            1.0,
            max(0.0, float(loss_age))
            / max(1e-6, tuning.line_recovery_timeout_seconds),
        )
        predicted = position + tuning.line_trend_gain * trend * age_scale
        predicted = max(-1.5, min(1.5, predicted))
        return predicted, position, trend


class MarkerOccurrenceGate:
    """Edge-trigger a visual marker only after it has cleared for several frames."""

    def __init__(self) -> None:
        self.count = 0
        self.active = False
        self.clear_frames = 0
        self.last_event_time = -1e9
        self.last_event_progress: Optional[float] = None

    def reset(self) -> None:
        self.count = 0
        self.active = False
        self.clear_frames = 0
        self.last_event_time = -1e9
        self.last_event_progress = None

    def observe(
        self,
        visible: bool,
        now: float,
        clear_frames_required: int,
        min_separation_seconds: float,
        progress: Optional[float] = None,
        min_separation_distance: float = 0.0,
    ) -> int:
        clear_frames_required = max(1, int(clear_frames_required))
        if visible:
            if self.active:
                self.clear_frames = 0
                return 0
            time_too_close = (
                now - self.last_event_time < float(min_separation_seconds)
            )
            distance_too_close = False
            if (
                progress is not None
                and self.last_event_progress is not None
                and math.isfinite(float(progress))
                and math.isfinite(float(self.last_event_progress))
            ):
                distance_too_close = (
                    float(progress) - float(self.last_event_progress)
                    < float(min_separation_distance)
                )
            if time_too_close or distance_too_close:
                # Treat this as the same physical marker until it clears again.
                # This prevents one dashed group flickering off for a few frames
                # from becoming occurrence #1 and #2.
                self.active = True
                self.clear_frames = 0
                return 0
            self.active = True
            self.clear_frames = 0
            self.count += 1
            self.last_event_time = float(now)
            if progress is not None and math.isfinite(float(progress)):
                self.last_event_progress = float(progress)
            return self.count

        if self.active:
            self.clear_frames += 1
            if self.clear_frames >= clear_frames_required:
                self.active = False
                self.clear_frames = 0
        return 0


@dataclass(frozen=True)
class ForkedDashPattern:
    """Geometry summary for the photographed fork-shaped ring exit."""

    visible: bool
    main_count: int
    branch_count: int
    main_x_span_ratio: float
    main_y_span_ratio: float
    branch_y_span_ratio: float
    angle_deg: float
    junction_margin_ratio: float


def _line_candidates(
    points: Sequence[Tuple[float, float]],
    tolerance_ratio: float,
    min_components: int,
) -> Sequence[dict]:
    """Return unique straight-line hypotheses through normalized component centers."""

    candidates = {}
    count = len(points)
    for i in range(count):
        for j in range(i + 1, count):
            x1, y1 = points[i]
            x2, y2 = points[j]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < 0.08:
                continue
            ux = dx / length
            uy = dy / length
            inliers = []
            for k, (x, y) in enumerate(points):
                vx = x - x1
                vy = y - y1
                distance = abs(vx * uy - vy * ux)
                if distance <= tolerance_ratio:
                    inliers.append(k)
            if len(inliers) < int(min_components):
                continue

            key = tuple(inliers)
            xs = [points[k][0] for k in inliers]
            ys = [points[k][1] for k in inliers]
            best_a = inliers[0]
            best_b = inliers[-1]
            best_distance = -1.0
            for ai in range(len(inliers)):
                for bi in range(ai + 1, len(inliers)):
                    a = inliers[ai]
                    b = inliers[bi]
                    distance = math.hypot(
                        points[a][0] - points[b][0],
                        points[a][1] - points[b][1],
                    )
                    if distance > best_distance:
                        best_distance = distance
                        best_a = a
                        best_b = b
            ax, ay = points[best_a]
            bx, by = points[best_b]
            vx = bx - ax
            vy = by - ay
            norm = max(1e-9, math.hypot(vx, vy))
            candidate = {
                "indices": key,
                "point": (ax, ay),
                "dir": (vx / norm, vy / norm),
                "x_min": min(xs),
                "x_max": max(xs),
                "y_min": min(ys),
                "y_max": max(ys),
                "x_span": max(xs) - min(xs),
                "y_span": max(ys) - min(ys),
                "span": best_distance,
            }
            previous = candidates.get(key)
            if previous is None or candidate["span"] > previous["span"]:
                candidates[key] = candidate
    return tuple(candidates.values())


def _line_intersection(a: dict, b: dict) -> Optional[Tuple[float, float]]:
    ax, ay = a["point"]
    adx, ady = a["dir"]
    bx, by = b["point"]
    bdx, bdy = b["dir"]
    denominator = adx * bdy - ady * bdx
    if abs(denominator) < 1e-6:
        return None
    rx = bx - ax
    ry = by - ay
    t = (rx * bdy - ry * bdx) / denominator
    return ax + t * adx, ay + t * ady


def detect_forked_exit(
    components: Sequence[Tuple[float, float, int, int, int]],
    width: float,
    height: float,
    tuning: AdaptiveLogicTuning,
) -> ForkedDashPattern:
    """Detect a dashed main row with one diagonal dashed branch."""

    width = max(1.0, float(width))
    height = max(1.0, float(height))
    points = [
        (float(item[0]) / width, float(item[1]) / height)
        for item in components
    ]
    if len(points) < (
        int(tuning.roundabout_exit_fork_min_main_components)
        + int(tuning.roundabout_exit_fork_min_branch_components)
        - 1
    ):
        return ForkedDashPattern(False, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0)

    candidates = _line_candidates(
        points,
        float(tuning.roundabout_exit_fork_line_tolerance_ratio),
        min(
            int(tuning.roundabout_exit_fork_min_main_components),
            int(tuning.roundabout_exit_fork_min_branch_components),
        ),
    )

    best = None
    diagnostic_best = None
    for main in candidates:
        for branch in candidates:
            if branch is main:
                continue

            dot = abs(
                main["dir"][0] * branch["dir"][0]
                + main["dir"][1] * branch["dir"][1]
            )
            dot = max(0.0, min(1.0, dot))
            angle = math.degrees(math.acos(dot))

            junction = _line_intersection(main, branch)
            if junction is None:
                junction_margin = 1.0
                branch_drop = 0.0
            else:
                jx, jy = junction

                def outside_distance(candidate: dict) -> float:
                    dx = max(candidate["x_min"] - jx, 0.0, jx - candidate["x_max"])
                    dy = max(candidate["y_min"] - jy, 0.0, jy - candidate["y_max"])
                    return math.hypot(dx, dy)

                junction_margin = max(
                    outside_distance(main),
                    outside_distance(branch),
                )
                branch_drop = float(branch["y_max"] - jy)

            diagnostic_score = (
                len(main["indices"])
                + len(branch["indices"])
                + 1.5 * float(main["x_span"])
                + 1.5 * float(branch["y_span"])
                + angle / 90.0
                - min(1.0, junction_margin)
            )
            diagnostic_pattern = ForkedDashPattern(
                False,
                len(main["indices"]),
                len(branch["indices"]),
                float(main["x_span"]),
                float(main["y_span"]),
                float(branch["y_span"]),
                float(angle),
                float(junction_margin),
            )
            if diagnostic_best is None or diagnostic_score > diagnostic_best[0]:
                diagnostic_best = (diagnostic_score, diagnostic_pattern)

            if len(main["indices"]) < int(tuning.roundabout_exit_fork_min_main_components):
                continue
            if main["x_span"] < tuning.roundabout_exit_fork_min_main_x_span_ratio:
                continue
            if main["y_span"] > tuning.roundabout_exit_fork_max_main_y_span_ratio:
                continue
            if len(branch["indices"]) < int(tuning.roundabout_exit_fork_min_branch_components):
                continue
            if branch["y_span"] < tuning.roundabout_exit_fork_min_branch_y_span_ratio:
                continue
            if len(set(main["indices"]).intersection(branch["indices"])) > 1:
                continue
            if angle < tuning.roundabout_exit_fork_min_angle_deg:
                continue
            if junction is None:
                continue
            if junction_margin > float(tuning.roundabout_exit_fork_junction_margin_ratio):
                continue
            if branch_drop < 0.12:
                continue

            score = diagnostic_score
            pattern = ForkedDashPattern(
                True,
                len(main["indices"]),
                len(branch["indices"]),
                float(main["x_span"]),
                float(main["y_span"]),
                float(branch["y_span"]),
                float(angle),
                float(junction_margin),
            )
            if best is None or score > best[0]:
                best = (score, pattern)

    if best is not None:
        return best[1]
    if diagnostic_best is not None:
        return diagnostic_best[1]
    return ForkedDashPattern(False, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0)


def _wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class RoundaboutGyroExitGate:
    """Integrate gyro_z from the real entry; never use gyro to steer."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.last_time: Optional[float] = None
        self.signed_turn_deg = 0.0
        self.confirm_frames = 0
        self.last_rate_deg_s = 0.0

    def start(self, now: float) -> bool:
        if not math.isfinite(float(now)):
            return False
        self.active = True
        self.last_time = float(now)
        self.signed_turn_deg = 0.0
        self.confirm_frames = 0
        self.last_rate_deg_s = 0.0
        return True

    def update(
        self,
        now: float,
        yaw_rate_rad_s: float,
        arm_turn_deg: float,
        target_turn_deg: float,
        return_tolerance_deg: float,
        confirm_frames_required: int,
        deadband_deg_s: float,
        scale: float,
        max_rate_deg_s: float,
        max_integration_dt_seconds: float,
        allow_confirm: bool,
    ) -> Tuple[bool, float, float, bool, int, float]:
        if not math.isfinite(float(now)) or not math.isfinite(float(yaw_rate_rad_s)):
            return (
                False,
                abs(self.signed_turn_deg),
                float("nan"),
                False,
                self.confirm_frames,
                self.last_rate_deg_s,
            )
        if not self.active or self.last_time is None:
            self.start(float(now))

        dt = max(0.0, float(now) - float(self.last_time))
        self.last_time = float(now)
        dt = min(dt, float(max_integration_dt_seconds))

        rate_deg_s = math.degrees(float(yaw_rate_rad_s))
        rate_deg_s = max(
            -float(max_rate_deg_s),
            min(float(max_rate_deg_s), rate_deg_s),
        )
        if abs(rate_deg_s) < float(deadband_deg_s):
            rate_deg_s = 0.0
        self.last_rate_deg_s = rate_deg_s

        self.signed_turn_deg += rate_deg_s * float(scale) * dt
        accumulated_deg = abs(self.signed_turn_deg)
        target_error_deg = accumulated_deg - float(target_turn_deg)
        armed = accumulated_deg >= float(arm_turn_deg)
        near_target = abs(target_error_deg) <= float(return_tolerance_deg)

        if armed and near_target and allow_confirm:
            self.confirm_frames += 1
        else:
            self.confirm_frames = 0

        triggered = self.confirm_frames >= max(1, int(confirm_frames_required))
        return (
            triggered,
            accumulated_deg,
            target_error_deg,
            armed,
            self.confirm_frames,
            rate_deg_s,
        )


class BranchMarkerGate:
    """V5 single-event gate for the real roundabout exit shape.

    The old two-marker sequence is intentionally gone. This gate confirms one
    robust exit-like pattern. The node then drives straight for the configured
    timer and enters FAST_ENTRY directly.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self.active = False
        self.clear_frames = 0
        self.candidate_frames = 0
        self.marker1_candidate_frames = 0
        self.between_center_frames = 0
        self.last_event_time = -1e9
        self.last_event_progress: Optional[float] = None
        self.first_branch_pending_until = -1e9
        self.first_branch_pending_time = -1e9
        self.first_branch_pending_progress: Optional[float] = None

    def observe(
        self,
        center_visible: bool,
        left_branch_visible: bool,
        now: float,
        clear_frames_required: int,
        min_separation_seconds: float,
        progress: Optional[float] = None,
        min_separation_distance: float = 0.0,
        confirm_frames_required: int = 1,
        between_center_frames_required: int = 0,
        left_branch_raw_visible: Optional[bool] = None,
        marker1_center_follow_seconds: float = 0.0,
        marker1_raw_confirm_frames: int = 1,
        left_branch_raw_count: int = 0,
        left_branch_raw_span: float = 0.0,
        marker2_raw_min_components: int = 3,
        marker2_raw_min_span_ratio: float = 0.14,
        center_raw_count: int = 0,
        center_raw_span: float = 0.0,
        exit_center_min_components: int = 2,
        exit_center_min_span_ratio: float = 0.06,
        exit_left_min_components: int = 2,
        exit_left_min_span_ratio: float = 0.08,
        exit_confirm_frames: int = 2,
    ) -> int:
        # Keep legacy args accepted so older callers/tests/config remain easy to
        # migrate, but V5 does not use the two-marker sequence at all.
        del (
            center_visible, left_branch_visible, clear_frames_required,
            min_separation_seconds, min_separation_distance,
            confirm_frames_required, between_center_frames_required,
            left_branch_raw_visible, marker1_center_follow_seconds,
            marker1_raw_confirm_frames, marker2_raw_min_components,
            marker2_raw_min_span_ratio,
        )
        if self.count >= 1:
            return 0

        real_exit = bool(
            int(center_raw_count) >= int(exit_center_min_components)
            and float(center_raw_span) >= float(exit_center_min_span_ratio)
            and int(left_branch_raw_count) >= int(exit_left_min_components)
            and float(left_branch_raw_span) >= float(exit_left_min_span_ratio)
        )
        if not real_exit:
            self.candidate_frames = 0
            return 0

        self.candidate_frames += 1
        if self.candidate_frames < max(1, int(exit_confirm_frames)):
            return 0

        self.count = 1
        self.active = True
        self.last_event_time = float(now)
        if progress is not None and math.isfinite(float(progress)):
            self.last_event_progress = float(progress)
        return 1

