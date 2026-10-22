from pathlib import Path
import sys

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from adaptive_runtime import (
    AdaptiveLogicTuning,
    BranchMarkerGate,
    MarkerOccurrenceGate,
    RoundaboutGyroExitGate,
    WeightedLineTrend,
    detect_forked_exit,
    fit_dashed_main_line,
    parse_adaptive_tuning_text,
    roundabout_exit_heading_turn,
    post_exit_search_turn,
    post_exit_reacquire_turn,
)


def test_left_motion_predicts_left_recovery():
    tuning = AdaptiveLogicTuning()
    tracker = WeightedLineTrend(5)
    tracker.observe(0.0, 0.02, 0.00, 0.00, tuning)
    tracker.observe(0.1, -0.10, -0.08, -0.05, tuning)
    tracker.observe(0.2, -0.28, -0.22, -0.14, tuning)
    predicted, position, trend = tracker.estimate(0.1, tuning)
    assert predicted < position < 0.0
    assert trend < 0.0
    assert -1.0 * tuning.line_recovery_kp * predicted > 0.0


def test_right_motion_predicts_right_recovery():
    tuning = AdaptiveLogicTuning()
    tracker = WeightedLineTrend(5)
    tracker.observe(0.0, -0.01, 0.00, 0.00, tuning)
    tracker.observe(0.1, 0.09, 0.07, 0.04, tuning)
    tracker.observe(0.2, 0.24, 0.20, 0.13, tuning)
    predicted, position, trend = tracker.estimate(0.1, tuning)
    assert predicted > position > 0.0
    assert trend > 0.0
    assert -1.0 * tuning.line_recovery_kp * predicted < 0.0


def test_centered_static_line_does_not_force_left():
    tuning = AdaptiveLogicTuning()
    tracker = WeightedLineTrend(5)
    for i in range(4):
        tracker.observe(i * 0.1, 0.01, 0.00, -0.01, tuning)
    predicted, _position, _trend = tracker.estimate(0.2, tuning)
    assert abs(predicted) < 0.03


def test_two_marker_occurrences_require_clear_frames():
    gate = MarkerOccurrenceGate()
    assert gate.observe(True, 0.0, 2, 0.5) == 1
    assert gate.observe(True, 0.1, 2, 0.5) == 0
    assert gate.observe(False, 0.2, 2, 0.5) == 0
    assert gate.observe(True, 0.3, 2, 0.5) == 0
    assert gate.observe(False, 0.4, 2, 0.5) == 0
    assert gate.observe(False, 0.5, 2, 0.5) == 0
    assert gate.observe(True, 0.8, 2, 0.5) == 2


def test_v5_weak_single_fragments_do_not_confirm_real_exit():
    gate = BranchMarkerGate()
    kwargs = dict(
        clear_frames_required=3, min_separation_seconds=2.0,
        exit_center_min_components=2, exit_center_min_span_ratio=0.06,
        exit_left_min_components=2, exit_left_min_span_ratio=0.08,
        exit_confirm_frames=2,
    )
    for i in range(5):
        assert gate.observe(
            False, False, i * 0.05, progress=1.0 + i * 0.01,
            center_raw_count=1, center_raw_span=0.0,
            left_branch_raw_count=1, left_branch_raw_span=0.0, **kwargs
        ) == 0
    assert gate.count == 0


def test_v5_real_exit_shape_requires_two_consecutive_frames():
    gate = BranchMarkerGate()
    kwargs = dict(
        clear_frames_required=3, min_separation_seconds=2.0,
        exit_center_min_components=2, exit_center_min_span_ratio=0.06,
        exit_left_min_components=2, exit_left_min_span_ratio=0.08,
        exit_confirm_frames=2,
    )
    assert gate.observe(
        True, False, 0.0, progress=3.4, center_raw_count=2, center_raw_span=0.12,
        left_branch_raw_count=2, left_branch_raw_span=0.11, **kwargs
    ) == 0
    assert gate.candidate_frames == 1
    assert gate.observe(
        True, False, 0.05, progress=3.42, center_raw_count=3, center_raw_span=0.18,
        left_branch_raw_count=2, left_branch_raw_span=0.13, **kwargs
    ) == 1
    assert gate.count == 1


def test_v5_confirmed_exit_is_single_event_no_second_marker_needed():
    gate = BranchMarkerGate()
    kwargs = dict(
        clear_frames_required=3, min_separation_seconds=2.0,
        exit_center_min_components=2, exit_center_min_span_ratio=0.06,
        exit_left_min_components=2, exit_left_min_span_ratio=0.08,
        exit_confirm_frames=1,
    )
    assert gate.observe(
        True, True, 0.0, progress=3.0, center_raw_count=2, center_raw_span=0.10,
        left_branch_raw_count=2, left_branch_raw_span=0.10, **kwargs
    ) == 1
    assert gate.observe(
        True, True, 5.0, progress=6.0, center_raw_count=4, center_raw_span=0.30,
        left_branch_raw_count=4, left_branch_raw_span=0.30, **kwargs
    ) == 0
    assert gate.count == 1


def test_fork_exit_matches_user_photographed_geometry():
    tuning = AdaptiveLogicTuning()
    w, h = 958.0, 1704.0

    # Approximate normalized centers measured from the user-supplied photo:
    # four blocks form the broad top row; three blocks peel down-right.
    photographed = [
        (0.202 * w, 0.369 * h, 20, 20, 100),
        (0.369 * w, 0.359 * h, 20, 20, 100),
        (0.606 * w, 0.374 * h, 20, 20, 100),
        (0.848 * w, 0.388 * h, 20, 20, 100),
        (0.477 * w, 0.453 * h, 20, 20, 100),
        (0.723 * w, 0.581 * h, 20, 20, 100),
        (0.932 * w, 0.762 * h, 20, 20, 100),
    ]
    pattern = detect_forked_exit(photographed, w, h, tuning)
    assert pattern.visible
    assert pattern.main_count >= 3
    assert pattern.branch_count >= 3
    assert pattern.main_x_span_ratio >= 0.28
    assert pattern.branch_y_span_ratio >= 0.18
    assert pattern.angle_deg >= 20.0


def test_fork_exit_rejects_one_straight_dashed_chain():
    tuning = AdaptiveLogicTuning()
    w, h = 640.0, 480.0
    straight = [
        (0.50 * w, y * h, 12, 14, 100)
        for y in (0.24, 0.34, 0.44, 0.54, 0.64, 0.74)
    ]
    pattern = detect_forked_exit(straight, w, h, tuning)
    assert not pattern.visible


def test_fork_exit_rejects_parallel_rows_without_diagonal_branch():
    tuning = AdaptiveLogicTuning()
    w, h = 640.0, 480.0
    parallel = [
        (x * w, 0.36 * h, 12, 14, 100)
        for x in (0.20, 0.40, 0.60, 0.80)
    ] + [
        (x * w, 0.55 * h, 12, 14, 100)
        for x in (0.22, 0.42, 0.62, 0.82)
    ]
    pattern = detect_forked_exit(parallel, w, h, tuning)
    assert not pattern.visible


def test_marker_distance_rejects_same_physical_dash_group():
    gate = MarkerOccurrenceGate()
    assert gate.observe(True, 0.0, 2, 0.5, progress=1.00, min_separation_distance=0.35) == 1
    assert gate.observe(False, 0.2, 2, 0.5, progress=1.05, min_separation_distance=0.35) == 0
    assert gate.observe(False, 0.3, 2, 0.5, progress=1.08, min_separation_distance=0.35) == 0
    # Same marker flickers back after time guard but only 0.15m farther: ignore it.
    assert gate.observe(True, 0.8, 2, 0.5, progress=1.15, min_separation_distance=0.35) == 0
    assert gate.observe(False, 1.0, 2, 0.5, progress=1.20, min_separation_distance=0.35) == 0
    assert gate.observe(False, 1.1, 2, 0.5, progress=1.22, min_separation_distance=0.35) == 0
    # A genuinely later marker is accepted as occurrence #2.
    assert gate.observe(True, 1.5, 2, 0.5, progress=1.50, min_separation_distance=0.35) == 2


def test_adaptive_yaml_round_trip():
    text = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "adaptive_tuning.yaml"
    ).read_text(encoding="utf-8")
    tuning, reason = parse_adaptive_tuning_text(text)
    assert reason == ""
    assert tuning.roundabout_ignore_after_exit_seconds == 25.0
    assert tuning.roundabout_ignore_after_lap_start_seconds == 0.0
    assert tuning.roundabout_marker_min_separation_distance_m == 0.0
    assert tuning.roundabout_marker_min_separation_seconds == 2.0
    assert tuning.roundabout_marker1_straight_seconds == 2.0
    assert tuning.roundabout_marker1_straight_speed == 0.25
    assert tuning.roundabout_marker1_straight_max_turn_speed == 0.0
    assert tuning.roundabout_marker1_follow_line_enabled is True
    assert tuning.roundabout_marker1_follow_max_turn_speed == 0.28
    assert tuning.roundabout_marker1_branch_left_turn_cap == 0.10
    assert tuning.roundabout_marker1_gap_bridge_seconds == 0.45
    assert tuning.roundabout_marker1_gap_speed_ratio == 0.90
    assert tuning.roundabout_marker1_pass_loss_min_speed == 0.32
    assert tuning.roundabout_marker1_post_hold_enabled is False
    assert tuning.roundabout_marker1_post_hold_speed_cap == 0.40
    assert tuning.roundabout_marker1_post_hold_loss_speed_cap == 0.12
    assert tuning.roundabout_marker1_post_hold_max_turn_speed == 0.15
    assert tuning.roundabout_initial_arm_delay_seconds == 10.0
    assert tuning.roundabout_exit_marker_center_min_components == 2
    assert tuning.roundabout_exit_marker_center_min_span_ratio == 0.06
    assert tuning.roundabout_exit_marker_left_min_components == 2
    assert tuning.roundabout_exit_marker_left_min_span_ratio == 0.08
    assert tuning.roundabout_exit_marker_confirm_frames == 2
    assert tuning.motion_smoothing_enabled is True
    assert tuning.motion_linear_accel_up == 0.48
    assert tuning.motion_linear_decel == 0.72
    assert tuning.motion_angular_slew_normal == 3.8
    assert tuning.motion_angular_slew_roundabout == 3.2
    assert tuning.motion_angular_reverse_slew == 1.80
    assert tuning.motion_angular_target_alpha == 0.60
    assert tuning.motion_angular_deadband == 0.015
    assert tuning.line_recovery_bridge_seconds == 0.22
    assert tuning.line_recovery_bridge_speed_ratio == 0.92
    assert tuning.line_recovery_persistent_enabled is True
    assert tuning.line_recovery_persistent_speed == 0.07
    assert tuning.line_recovery_persistent_turn_start == 0.18
    assert tuning.line_recovery_persistent_turn_max == 0.42
    assert tuning.line_recovery_persistent_turn_ramp_seconds == 1.50
    assert tuning.line_recovery_direction_deadband == 0.04
    assert tuning.roundabout_marker1_center_follow_seconds == 1.40
    assert tuning.roundabout_marker1_raw_confirm_frames == 1
    assert tuning.roundabout_marker2_raw_min_components == 3
    assert tuning.roundabout_marker2_raw_min_span_ratio == 0.14
    assert tuning.roundabout_marker_confirm_frames == 1
    assert tuning.roundabout_marker_clear_frames == 3
    assert tuning.roundabout_marker_between_center_frames == 1
    assert tuning.roundabout_dual_min_horizontal_gap_ratio == 0.08
    assert tuning.roundabout_dual_center_x_min == 0.30
    assert tuning.roundabout_dual_center_x_max == 0.70
    assert tuning.roundabout_dual_center_min_span_ratio == 0.07
    assert tuning.roundabout_dual_left_min_span_ratio == 0.03
    assert tuning.roundabout_dual_max_vertical_gap_ratio == 0.32
    assert tuning.roundabout_entry_min_search_seconds == 0.60
    assert tuning.roundabout_entry_max_search_seconds == 1.10
    assert tuning.roundabout_entry_forward_speed == 0.30
    assert tuning.roundabout_follow_steering_gain == 1.10
    assert tuning.roundabout_blind_recovery_speed == 0.12
    assert tuning.roundabout_blind_recovery_left_turn == 0.10
    assert tuning.roundabout_blind_recovery_max_turn_speed == 0.45
    assert tuning.roundabout_gyro_exit_enabled is True
    assert tuning.roundabout_gyro_arm_turn_deg == 270.0
    assert tuning.roundabout_gyro_target_turn_deg == 360.0
    assert tuning.roundabout_gyro_return_tolerance_deg == 15.0
    assert tuning.roundabout_gyro_return_confirm_frames == 3
    assert tuning.roundabout_gyro_require_line_valid is False
    assert tuning.roundabout_gyro_force_exit_after_target_enabled is True
    assert tuning.roundabout_gyro_force_exit_margin_deg == 5.0
    assert tuning.roundabout_gyro_force_exit_confirm_frames == 2
    assert tuning.roundabout_gyro_imu_timeout_seconds == 0.50
    assert tuning.roundabout_gyro_deadband_deg_s == 1.0
    assert tuning.roundabout_gyro_scale == 1.0
    assert tuning.roundabout_gyro_max_rate_deg_s == 240.0
    assert tuning.roundabout_gyro_max_integration_dt_seconds == 0.10
    assert tuning.roundabout_exit_require_gyro_arm is True
    assert tuning.roundabout_gyro_exit_commit_seconds == 1.00
    assert tuning.roundabout_gyro_exit_speed == 0.25
    assert tuning.roundabout_gyro_exit_left_bias == 0.00
    assert tuning.roundabout_gyro_exit_max_turn_speed == 0.12
    assert tuning.roundabout_post_exit_guard_seconds == 2.50
    assert tuning.roundabout_post_exit_speed_cap == 0.30
    assert tuning.roundabout_post_exit_min_left_turn_speed == 0.12
    assert tuning.roundabout_post_exit_max_left_turn_speed == 0.22
    assert tuning.roundabout_post_exit_max_right_turn_speed == 0.00
    assert tuning.second_roundabout_arm_delay_after_first_crosswalk_seconds == 15.0
    assert tuning.roundabout_entry_left_bias == 0.20
    assert tuning.roundabout_entry_min_left_turn_speed == 0.45
    assert tuning.roundabout_entry_reacquire_frames == 3
    assert tuning.roundabout_exit_fork_confirm_frames == 2
    assert tuning.roundabout_exit_fork_min_main_components == 3
    assert tuning.roundabout_exit_fork_min_branch_components == 3
    assert tuning.roundabout_exit_fork_min_angle_deg == 20.0
    assert tuning.roundabout_exit_speed == 0.20
    assert tuning.crosswalk_far_y_top_ratio == 0.18
    assert tuning.crosswalk_far_y_bottom_ratio == 0.66
    assert tuning.crosswalk_approach_distance_m == 0.22


def test_second_lap_prestep_cap_requires_step_master_switch():
    source = (SCRIPTS / "icn.py").read_text(encoding="utf-8")
    compact = "".join(source.split())
    assert (
        "SECOND_LAP_PRESTEP_SLOW_ENABLED"
        "andself._live_tuning.step_enabled"
        "andself.lap_count>=1"
    ) in compact


def test_adaptive_roundabout_follow_enforces_hot_speed_and_blind_recovery():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "self._live_tuning.roundabout_forward_speed" in source
    assert "tuning.roundabout_blind_recovery_speed" in source
    assert "tuning.roundabout_blind_recovery_left_turn" in source
    assert "tuning.roundabout_blind_recovery_max_turn_speed" in source
    assert "adaptive_roundabout_follow_speed_cap" in source
    assert "roundabout_follow_steering_gain" in source
    assert "adaptive_roundabout_blind_trend_" in source
    assert "adaptive_roundabout_blind_fallback_left" in source
    assert "max(min_left, trend_angular)" not in source


def test_race_tuning_has_slower_ring_and_gentler_legacy_blind_turn():
    text = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "race_tuning.yaml"
    ).read_text(encoding="utf-8")
    assert "roundabout_forward_speed: 0.26" in text
    assert "roundabout_blind_recovery_speed: 0.12" in text
    assert "roundabout_blind_recovery_left_turn: 0.10" in text


def test_adaptive_roundabout_source_uses_branch_gate_trend_entry_and_fork_exit():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "max_dx_ratio=0.28" in source
    assert "RINGDBG " in source
    assert "exit_interlaced_visible" not in source
    assert "roundabout_exit_interlaced" not in source
    assert "BranchMarkerGate" in source
    assert "roundabout_left_branch" in source
    assert "adaptive_exit_then_mainline_pass_left_entry" in source
    assert "adaptive_roundabout_entry_trend_" in source
    assert "roundabout_entry_min_left_turn_speed" in source
    assert "roundabout_entry_max_search_seconds" in source
    assert "FAST_ENTRY 达到最大保持时间" in source
    assert "adaptive_marker1_mainline_pass_" in source
    assert "adaptive_marker1_post_hold_line_guard" not in source
    assert "adaptive_marker1_post_hold_trend_recovery" not in source
    assert "after the real exit shape is confirmed" in source
    assert "RoundaboutGyroExitGate" in source
    assert "IMU gyro_z 出环兜底确认" in source
    assert "adaptive_post_exit_reacquire_guard_" not in source
    assert "adaptive_roundabout_gyro_exit_straight" not in source
    assert "adaptive_roundabout_gyro_exit_line_follow" in source
    assert "adaptive_roundabout_gyro_exit_loss_keep_left" in source
    assert "detect_forked_exit" in source
    assert "roundabout_exit_fork" in source
    assert "EXITDBG " in source
    assert "mainspan=" in source
    assert "junction=" in source
    assert "禁止在分叉处跟右线" in source
    assert 'self._set_phase("ROUNDABOUT_EXIT", now)' in source
    assert "min_left = max(" in source


def test_fork_exit_false_pattern_keeps_diagnostic_geometry():
    tuning = AdaptiveLogicTuning()
    w, h = 640.0, 480.0
    near_miss = [
        (0.18 * w, 0.36 * h, 12, 14, 100),
        (0.42 * w, 0.37 * h, 12, 14, 100),
        (0.72 * w, 0.38 * h, 12, 14, 100),
        (0.48 * w, 0.47 * h, 12, 14, 100),
        (0.60 * w, 0.55 * h, 12, 14, 100),
        (0.70 * w, 0.61 * h, 12, 14, 100),
    ]
    pattern = detect_forked_exit(near_miss, w, h, tuning)
    assert pattern.main_count >= 3
    assert pattern.branch_count >= 3
    assert pattern.main_x_span_ratio >= 0.0
    assert pattern.branch_y_span_ratio >= 0.0

# CI guard: soft-start/ring-entry/EXITDBG package


def test_roundabout_gyro_exit_gate_integrates_to_one_full_turn():
    import math

    gate = RoundaboutGyroExitGate()
    assert gate.start(0.0)
    triggered = False
    # 90 deg/s for four seconds => 360deg. Calls are 0.1s apart so the
    # integration dt cap does not distort the result.
    now = 0.0
    for _ in range(40):
        now += 0.1
        triggered, turn, error, armed, frames, rate = gate.update(
            now,
            math.radians(90.0),
            270.0,
            360.0,
            15.0,
            3,
            1.0,
            1.0,
            240.0,
            0.10,
            True,
        )
    assert turn >= 350.0
    assert armed
    assert abs(rate - 90.0) < 1e-6
    # Need three consecutive frames in the 360 +/- 15deg window.
    for _ in range(2):
        now += 0.01
        triggered, turn, error, armed, frames, rate = gate.update(
            now,
            0.0,
            270.0,
            360.0,
            15.0,
            3,
            1.0,
            1.0,
            240.0,
            0.10,
            True,
        )
    assert triggered
    assert abs(error) <= 15.0


def test_roundabout_gyro_exit_gate_never_confirms_without_valid_line():
    import math

    gate = RoundaboutGyroExitGate()
    gate.start(0.0)
    now = 0.0
    for _ in range(42):
        now += 0.1
        triggered, _turn, _error, _armed, frames, _rate = gate.update(
            now,
            math.radians(90.0),
            270.0,
            360.0,
            15.0,
            3,
            1.0,
            1.0,
            240.0,
            0.10,
            False,
        )
    assert not triggered
    assert frames == 0


def test_roundabout_gyro_deadband_filters_static_noise():
    import math

    gate = RoundaboutGyroExitGate()
    gate.start(0.0)
    now = 0.0
    for _ in range(100):
        now += 0.05
        triggered, turn, _error, _armed, _frames, rate = gate.update(
            now,
            math.radians(0.5),
            270.0,
            360.0,
            15.0,
            3,
            1.0,
            1.0,
            240.0,
            0.10,
            True,
        )
    assert not triggered
    assert turn == 0.0
    assert rate == 0.0


def test_v5_raw_left_alone_cannot_arm_exit():
    gate = BranchMarkerGate()
    assert gate.observe(
        False, False, 0.0, 3, 2.0, progress=1.38,
        center_raw_count=0, center_raw_span=0.0,
        left_branch_raw_count=5, left_branch_raw_span=0.30,
        exit_confirm_frames=1,
    ) == 0
    assert gate.count == 0


def test_v5_center_without_left_exit_shape_cannot_arm():
    gate = BranchMarkerGate()
    assert gate.observe(
        True, False, 0.0, 3, 2.0, progress=2.0,
        center_raw_count=4, center_raw_span=0.25,
        left_branch_raw_count=1, left_branch_raw_span=0.0,
        exit_confirm_frames=1,
    ) == 0
    assert gate.count == 0


def test_v5_overlap_does_not_matter_when_raw_exit_shape_is_strong():
    gate = BranchMarkerGate()
    assert gate.observe(
        False, False, 0.0, 3, 2.0, progress=4.6,
        center_raw_count=3, center_raw_span=0.23,
        left_branch_raw_count=3, left_branch_raw_span=0.23,
        exit_center_min_components=2, exit_center_min_span_ratio=0.06,
        exit_left_min_components=2, exit_left_min_span_ratio=0.08,
        exit_confirm_frames=1,
    ) == 1


def test_v5_exit_gate_ignores_odometry_distance_and_uses_shape_only():
    gate = BranchMarkerGate()
    assert gate.observe(
        False, False, 0.0, 3, 2.0, progress=0.0, min_separation_distance=99.0,
        center_raw_count=2, center_raw_span=0.10,
        left_branch_raw_count=2, left_branch_raw_span=0.10,
        exit_confirm_frames=1,
    ) == 1


def test_adaptive_source_keeps_searching_after_prediction_timeout():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "line_recovery_persistent_enabled" in source
    assert "adaptive_trend_recovery_{phase_tag}" in source
    assert "persistent low-speed search" in source
    assert "roundabout_chain_independent" in source
    assert "roundabout_horizontal_gap" in source
    assert "roundabout_left_branch_raw" in source
    assert "marker1_center_follow_seconds" in source


def test_v10_gyro_exit_never_blindly_drives_straight():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert 'self._roundabout_exit_source = "visual"' in source
    assert 'self._roundabout_exit_source = "gyro"' in source
    assert 'self._roundabout_exit_source = "timeout"' in source
    assert 'if self._roundabout_exit_source == "gyro"' in source
    assert "adaptive_roundabout_gyro_exit_straight" not in source
    assert "adaptive_roundabout_gyro_exit_line_follow" in source
    assert "adaptive_roundabout_gyro_exit_loss_keep_left" in source
    assert "roundabout_exit_loss_left_turn" in source
    assert "source={exit_source}" in source


def test_v11_post_exit_uses_conditional_heading_reference_not_blind_motion_override():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "adaptive_post_exit_reacquire_guard_" not in source
    assert "roundabout_exit_heading_post_exit_seconds" in source
    assert "adaptive_roundabout_exit_heading_left_branch_hold" in source
    assert "不用w=0盲直" in source
    assert "self._roundabout_completed_lap_index = int(self.lap_count)" in source


def test_marker1_uses_timed_mainline_pass_then_direct_entry():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "adaptive_marker1_mainline_pass_" in source
    assert "roundabout_marker1_straight_seconds" in source
    assert "roundabout_marker1_follow_max_turn_speed" in source
    assert "roundabout_marker1_branch_left_turn_cap" in source
    assert "left_branch_suppressed" in source
    assert "adaptive_marker1_post_hold_line_guard" not in source
    assert "adaptive_marker1_post_hold_trend_recovery" not in source
    # After the timed main-line pass, V7 directly enters FAST_ENTRY; no marker2 gap.
    assert "adaptive_exit_then_mainline_pass_left_entry" in source
    assert "记录gyro起始角" in source


def test_v5_real_exit_parameters_and_direct_entry_are_exposed_in_source():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_exit_marker_center_min_components" in source
    assert "roundabout_exit_marker_left_min_components" in source
    assert "roundabout_exit_marker_confirm_frames" in source
    assert "EXITcand=" in source
    assert "adaptive_exit_then_mainline_pass_left_entry" in source
    assert "窗口内已忽略左分叉事件" in source


def test_v4_global_ring_timing_and_gyro_exit_guards_are_present():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_initial_arm_delay_seconds" in source
    assert "now - self._race_motion_start_time" in source
    assert "roundabout_exit_require_gyro_arm" in source
    assert "visual_angle_ok" in source
    assert "self._roundabout_gyro_armed" in source


def test_v4_motion_smoothing_override_is_asymmetric_and_ring_specific():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "def _limit_command(" in source
    assert "motion_linear_accel_up" in source
    assert "motion_linear_decel" in source
    assert "motion_angular_slew_normal" in source
    assert "motion_angular_slew_roundabout" in source
    assert "motion_angular_reverse_slew" in source
    assert "motion_angular_target_alpha" in source
    assert "reversing = (" in source
    assert 'if self.phase == "ROUNDABOUT_FOLLOW"' in source


def test_race_tuning_caps_turn_and_uses_reasonable_base_slew():
    text = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "race_tuning.yaml"
    ).read_text(encoding="utf-8")
    assert "drive_max_turn_speed: 0.80" in text
    assert "drive_angular_accel: 3.00" in text
    assert "drive_min_forward_speed: 0.26" in text
    assert "drive_curve_slowdown: 0.64" in text


def test_v7_source_uses_mainline_pass_then_direct_fast_entry():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    compact = "".join(source.split())
    assert "roundabout_marker1_straight_seconds" in source
    assert "adaptive_marker1_mainline_pass_" in source
    assert "roundabout_marker1_follow_line_enabled" in source
    assert 'self._ring_branch_gate.count==1' in compact
    assert 'self._set_phase("ROUNDABOUT_FAST_ENTRY",now)' in compact
    assert "left_branch_suppressed" in source
    assert "adaptive_marker1_straight_hold" not in source



def test_good2_plus_uses_30hz_control_and_nonstall_hard_turn_cap():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    icn_text = (config_dir / "icn.yaml").read_text(encoding="utf-8")
    race_text = (config_dir / "race_tuning.yaml").read_text(encoding="utf-8")
    assert "control_rate: 30.0" in icn_text
    assert "drive_hard_turn_speed_cap: 0.20" in race_text


def test_roundabout_loss_keeps_bidirectional_trend_before_left_fallback():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "linear, angular, recovery_mode = self._trend_recovery_command(now)" in source
    assert "angular = base.clamp(angular, -max_turn, max_turn)" in source
    assert "loss_age >= tuning.line_recovery_timeout_seconds" in source
    assert "abs(angular) < tuning.line_recovery_direction_deadband" in source
    assert 'recovery_mode = "adaptive_roundabout_blind_fallback_left"' in source


def test_v7_gyro_exit_cannot_miss_full_turn_when_line_disappears():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_gyro_force_exit_after_target_enabled" in source
    assert "roundabout_gyro_force_exit_margin_deg" in source
    assert "crossed_target_latch" in source
    assert "_roundabout_gyro_force_exit_frames" in source
    assert "roundabout_gyro_require_line_valid" in source


def test_v7_completed_roundabout_latches_same_lap_reentry_off():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "_roundabout_completed_lap_index" in source
    assert "self._roundabout_completed_lap_index = int(self.lap_count)" in source
    assert "self._roundabout_completed_lap_index == int(self.lap_count)" in source


def test_v7_control_perf_debug_exposes_real_processed_fps():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "CTRLDBG " in source
    assert "vision={fps:.1f}/{base.CONTROL_RATE:.1f}Hz" in source


def test_v8_entry_handoff_clears_stale_trend():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "adaptive_roundabout_entry_handoff_loss_keep_left" in source
    assert "roundabout_follow_entry_guard_until" in source
    assert "self._line_trend.reset()" in source


def test_v10_ring_single_block_and_exit_zone_loss_have_explicit_rescue_paths():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_single_block_rescue_enabled" in source
    assert "adaptive_roundabout_single_block_rescue" in source
    assert "roundabout_exit_loss_left_arm_turn_deg" in source
    assert "adaptive_roundabout_exit_zone_loss_keep_left" in source
    assert "roundabout_single_block_error" in source


def test_v8_uses_real_processed_frame_dt_for_smoothing():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "max(2.0 / base.CONTROL_RATE, 0.12)" in source

def test_v8_debug_video_encoding_disabled_for_runtime_fps():
    text = (Path(__file__).resolve().parents[1] / "config" / "icn.yaml").read_text(encoding="utf-8")
    assert "record_debug_video: false" in text


def test_v9_first_crosswalk_delays_second_roundabout_arm_for_15_seconds():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    compact = "".join(source.split())
    assert "second_roundabout_arm_delay_after_first_crosswalk_seconds" in source
    assert "_first_crosswalk_confirmed_at" in source
    assert "_second_roundabout_enable_after" in source
    assert 'int(self.lap_count)==0' in compact
    assert 'int(self.lap_count)>=1' in compact
    assert 'now<self._second_roundabout_enable_after' in compact
    assert "第一次人行横道确认：从现在开始锁住第二次环岛识别" in source
    assert "cw2=" in source


def test_v9_yaml_exposes_left_exit_guard_and_second_ring_crosswalk_delay():
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    assert "roundabout_post_exit_min_left_turn_speed: 0.12" in text
    assert "roundabout_post_exit_max_left_turn_speed: 0.22" in text
    assert "roundabout_post_exit_max_right_turn_speed: 0.00" in text
    assert "second_roundabout_arm_delay_after_first_crosswalk_seconds: 15.0" in text


def test_good3_1_yaml_enables_single_block_and_200deg_exit_loss_left():
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    assert "roundabout_single_block_rescue_enabled: true" in text
    assert "roundabout_single_block_speed: 0.18" in text
    assert "roundabout_single_block_kp: 0.78" in text
    assert "roundabout_exit_loss_left_arm_turn_deg: 200.0" in text
    assert "roundabout_exit_loss_left_turn: 0.30" in text


def test_v11_heading_turn_converges_to_saved_entry_heading():
    assert abs(roundabout_exit_heading_turn(330.0, 360.0, 8.0, 0.010, 0.30, 0.12) - 0.30) < 1e-9
    assert abs(roundabout_exit_heading_turn(345.0, 360.0, 8.0, 0.010, 0.30, 0.12) - 0.15) < 1e-9
    assert abs(roundabout_exit_heading_turn(350.0, 360.0, 8.0, 0.010, 0.30, 0.12) - 0.10) < 1e-9
    assert roundabout_exit_heading_turn(355.0, 360.0, 8.0, 0.010, 0.30, 0.12) == 0.0
    assert roundabout_exit_heading_turn(360.0, 360.0, 8.0, 0.010, 0.30, 0.12) == 0.0
    assert abs(roundabout_exit_heading_turn(375.0, 360.0, 8.0, 0.010, 0.30, 0.12) + 0.12) < 1e-9


def test_v11_source_arms_330deg_heading_lock_and_one_second_left_branch_hold():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_exit_heading_lock_arm_deg" in source
    assert "roundabout_exit_heading_target_deg" in source
    assert "roundabout_exit_heading_visual_weight" in source
    assert "roundabout_exit_left_branch_trigger_turn" in source
    assert "roundabout_exit_left_branch_hold_seconds" in source
    assert "_roundabout_exit_heading_lock_armed" in source
    assert "_roundabout_post_exit_heading_suppress_used" in source
    assert "adaptive_roundabout_exit_heading_blend" in source
    assert "adaptive_roundabout_exit_heading_left_branch_hold" in source


def test_good3_1_200deg_exit_loss_and_heading_lock_are_preserved():
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    assert "roundabout_exit_loss_speed: 0.24" in text
    assert "roundabout_exit_heading_lock_enabled: true" in text
    assert "roundabout_exit_heading_lock_arm_deg: 330.0" in text
    assert "roundabout_exit_heading_target_deg: 360.0" in text
    assert "roundabout_exit_heading_kp: 0.010" in text
    assert "roundabout_exit_left_branch_trigger_turn: 0.18" in text
    assert "roundabout_exit_left_branch_hold_seconds: 1.50" in text
    assert "roundabout_exit_heading_post_exit_seconds: 3.00" in text


def test_good3_1_pass_window_plus_v11_4_fast_exit_are_both_present():
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_exit_loss_left_arm_turn_deg: 200.0" in text
    assert "roundabout_post_exit_loss_left_turn: 0.50" in text
    assert "adaptive_roundabout_post_exit_no_line_heading_hold" in source
    assert "roundabout_marker1_pass_loss_min_speed: 0.32" in text
    assert "roundabout_marker1_pass_loss_min_speed" in source


def test_v11_4_ring_follow_bites_line_more_strongly():
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    assert "roundabout_follow_steering_gain: 1.10" in text
    assert "roundabout_blind_recovery_max_turn_speed: 0.45" in text
    assert "roundabout_exit_loss_left_arm_turn_deg: 200.0" in text


def test_v11_4_post_exit_left_search_ramps_smoothly():
    assert abs(post_exit_search_turn(0.35, 0.55, 0.0, 0.40) - 0.35) < 1e-9
    assert abs(post_exit_search_turn(0.35, 0.55, 0.20, 0.40) - 0.45) < 1e-9
    assert abs(post_exit_search_turn(0.35, 0.55, 0.40, 0.40) - 0.55) < 1e-9
    assert abs(post_exit_search_turn(0.35, 0.55, 1.00, 0.40) - 0.55) < 1e-9


def test_v11_4_post_exit_reacquire_blends_without_snap():
    assert abs(post_exit_reacquire_turn(0.55, -0.25, 0.0, 0.30) - 0.55) < 1e-9
    assert abs(post_exit_reacquire_turn(0.55, -0.25, 0.15, 0.30) - 0.15) < 1e-9
    assert abs(post_exit_reacquire_turn(0.55, -0.25, 0.30, 0.30) + 0.25) < 1e-9


def test_v11_4_yaml_exposes_smooth_post_exit_parameters():
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "roundabout_post_exit_loss_left_turn_start: 0.35" in text
    assert "roundabout_post_exit_loss_left_turn_max: 0.55" in text
    assert "roundabout_post_exit_turn_ramp_seconds: 0.40" in text
    assert "roundabout_post_exit_reacquire_blend_seconds: 0.30" in text
    assert "roundabout_post_exit_search_speed: 0.24" in text
    assert "adaptive_roundabout_post_exit_no_line_heading_hold" in source
    assert "post_exit_reacquire_blend" in source


def test_good3_1_second_pass_reentry_guards_are_intact():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    assert "roundabout_ignore_after_exit_seconds: 25.0" in text
    assert "second_roundabout_arm_delay_after_first_crosswalk_seconds: 15.0" in text
    assert "self._roundabout_completed_lap_index == int(self.lap_count)" in source
    assert "now < self._roundabout_ignore_until" in source
    assert "now < self._second_roundabout_enable_after" in source


def test_post_exit_virtual_main_fits_separated_blocks():
    est = fit_dashed_main_line(
        [(170, 90), (165, 150), (158, 215), (150, 285)],
        320, 360, min_components=2, min_span_ratio=0.055,
    )
    assert est.valid
    assert est.count == 4
    assert est.span_ratio > 0.50
    assert est.near_error < 0.0


def test_post_exit_virtual_main_rejects_tiny_span():
    est = fit_dashed_main_line(
        [(150, 180), (154, 184)], 320, 360,
        min_components=2, min_span_ratio=0.055,
    )
    assert not est.valid


def test_good3_1_core_only_adds_post_exit_main_selection_contract():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    text = (Path(__file__).resolve().parents[1] / "config" / "adaptive_tuning.yaml").read_text(encoding="utf-8")
    # good3_1 entry timing and fast-exit tuning remain untouched.
    assert "roundabout_marker1_straight_seconds: 2.00" in text
    assert "roundabout_exit_loss_left_arm_turn_deg: 200.0" in text
    assert "roundabout_post_exit_loss_left_turn_start: 0.35" in text
    assert "roundabout_post_exit_loss_left_turn_max: 0.55" in text
    # New logic is constrained to post-exit main selection.
    assert "def _post_exit_mainline_command" in source
    assert "post_exit_virtual_main" in source
    assert "left_branch_filtered" in source
    assert "if self.phase != \"SEEK_CROSSWALK\"" in source
    # Later V11.13 timing/normal-patrol rewrites must not leak back in.
    assert "2.30s" not in source
    assert "0.78m" not in source
    assert "normal_anti_cross" not in source
    assert "line_recovery_soft_hold" not in source


def test_post_exit_reacquired_dropout_uses_fresh_trend_with_local_caps_only():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    # Initial exit search remains available until a real/virtual main road is seen.
    assert "self._post_exit_main_reacquired = False" in source
    assert source.count("self._post_exit_main_reacquired = True") >= 2
    # Once reacquired, later dropout uses the existing good3_1 trend estimator,
    # but only this post-exit path is locally limited against surge/spin.
    assert "def _post_exit_reacquired_loss_command" in source
    assert "linear, angular, mode = self._trend_recovery_command(now)" in source
    assert "linear = base.clamp(float(linear), 0.10, 0.16)" in source
    assert "angular = base.clamp(float(angular), -0.30, 0.30)" in source
    assert "adaptive_post_exit_reacquired_trend_loss/" in source
    main_idx = source.index("mainline_cmd = self._post_exit_mainline_command")
    reacq_idx = source.index("if self._post_exit_main_reacquired:", main_idx)
    heading_idx = source.index("now < self._roundabout_exit_heading_post_until", reacq_idx)
    assert main_idx < reacq_idx < heading_idx


def test_post_exit_initial_no_line_uses_saved_heading_not_fixed_left_search():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "adaptive_roundabout_post_exit_no_line_heading_hold" in source
    block_start = source.index("if not result.line_valid:", source.index("def _roundabout_exit_heading_command"))
    block_end = source.index("linear, visual_angular", block_start)
    block = source[block_start:block_end]
    assert "angular = heading_angular" in block
    assert "angular = max(heading_angular, search_angular)" not in block


def test_normal_patrol_reaccel_is_gentler_without_changing_roundabout_follow():
    source = (SCRIPTS / "icn_adaptive.py").read_text(encoding="utf-8")
    assert "NORMAL_PATROL_REACCEL_RATE = 0.22" in source
    assert 'self.phase in ("SEEK_ROUNDABOUT", "SEEK_CROSSWALK", "SEEK_FINISH")' in source
    assert "linear_rate = min(linear_rate, NORMAL_PATROL_REACCEL_RATE)" in source
    # ROUNDABOUT_FOLLOW is deliberately absent from the normal-patrol speed cap.
    normal_cap_line = 'self.phase in ("SEEK_ROUNDABOUT", "SEEK_CROSSWALK", "SEEK_FINISH")'
    assert "ROUNDABOUT_FOLLOW" not in normal_cap_line
