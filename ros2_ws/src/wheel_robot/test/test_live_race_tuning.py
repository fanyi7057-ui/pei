"""Behavioral tests for the hot-reloaded race parameter table."""

from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from race_runtime import (
    FastRoundaboutEntryGate,
    FirstTurnFollower,
    LiveRaceTuning,
    PredictiveTurnGate,
    SoftwareBuzzerEvents,
)


def test_fast_roundabout_entry_needs_three_components_and_two_on_screen_left():
    tuning = LiveRaceTuning.defaults()
    gate = FastRoundaboutEntryGate()

    assert not gate.observe(1, total_components=3, left_components=1, tuning=tuning)
    assert gate.observe(2, total_components=3, left_components=2, tuning=tuning)


def test_fast_roundabout_entry_counts_only_distinct_camera_frames():
    tuning, reason = LiveRaceTuning.defaults().with_updates(
        {"live_tuning.roundabout_fast_entry_confirm_frames": 2}
    )
    assert reason == ""
    gate = FastRoundaboutEntryGate()

    assert not gate.observe(10, total_components=3, left_components=2, tuning=tuning)
    assert not gate.observe(10, total_components=3, left_components=2, tuning=tuning)
    assert gate.observe(11, total_components=3, left_components=2, tuning=tuning)


def test_predictive_turn_requires_two_agreeing_mid_far_frames_before_left_override():
    gate = PredictiveTurnGate(confirm_frames=2, far_error_threshold=0.12)
    assert gate.observe(near_error=0.01, mid_error=0.16, far_error=0.25) == 0
    assert gate.observe(near_error=0.01, mid_error=0.17, far_error=0.28) == 1


def test_first_left_bend_tracks_each_new_frame_until_straight_exit():
    tuning = LiveRaceTuning.defaults()
    controller = FirstTurnFollower()

    assert controller.update(1, 0.0, 0.01, -0.10, -0.22, tuning).mode == "first_turn_wait"
    entered = controller.update(2, 0.1, 0.01, -0.12, -0.28, tuning)
    updated = controller.update(3, 0.2, -0.10, -0.20, -0.30, tuning)

    assert entered.mode == "first_turn_tracking"
    assert updated.mode == "first_turn_tracking"
    assert updated.angular != entered.angular
    assert not controller.done


def test_first_left_bend_exits_only_after_consecutive_straight_new_frames():
    tuning = LiveRaceTuning.defaults()
    controller = FirstTurnFollower()
    controller.update(1, 0.0, 0.01, -0.10, -0.22, tuning)
    controller.update(2, 0.1, 0.01, -0.12, -0.28, tuning)

    assert controller.update(3, 0.2, 0.01, 0.01, 0.01, tuning).mode == "first_turn_tracking"
    assert controller.update(4, 0.3, 0.01, 0.01, 0.01, tuning).mode == "first_turn_complete"
    assert controller.done


def test_live_tuning_accepts_the_user_unbounded_motion_values():
    updated, reason = LiveRaceTuning.defaults().with_updates(
        {
            "live_tuning.drive_forward_speed": 0.50,
            "live_tuning.drive_start_soft_enabled": 1,
            "live_tuning.drive_start_soft_seconds": 3.00,
            "live_tuning.drive_start_soft_speed": 0.25,
            "live_tuning.drive_max_turn_speed": 20.0,
            "live_tuning.drive_angular_accel": 20.0,
            "live_tuning.drive_curve_speed_cap": 55.0,
            "live_tuning.roundabout_entry_turn_speed": 0.98,
            "live_tuning.roundabout_entry_turn_distance": 0.66,
        }
    )
    assert reason == ""
    assert updated.drive_start_soft_enabled is True
    assert updated.drive_start_soft_seconds == pytest.approx(3.00)
    assert updated.drive_start_soft_speed == pytest.approx(0.25)
    assert updated.drive_max_turn_speed == pytest.approx(20.0)
    assert updated.drive_curve_speed_cap == pytest.approx(55.0)
    assert updated.roundabout_entry_turn_speed == pytest.approx(0.98)


def test_restored_original_schema_rejects_removed_ring_and_crosswalk_overlay_fields():
    current = LiveRaceTuning.defaults()
    for name, value in (
        ("roundabout_visual_enabled", 1),
        ("crosswalk_first_enable_after_start_seconds", 30.0),
        ("crosswalk_clear_seconds", 3.0),
        ("finish_mode", 1),
    ):
        updated, reason = current.with_updates({"live_tuning." + name: value})
        assert updated is current
        assert reason


def test_live_snapshot_updates_original_crosswalk_and_posture_atomically():
    current = LiveRaceTuning.defaults()
    updated, reason = current.with_updates(
        {
            "live_tuning.crosswalk_confirm_frames": 2,
            "live_tuning.crosswalk_stop_seconds": 3.0,
            "live_tuning.crosswalk_brake_reverse_enabled": 1,
            "live_tuning.crosswalk_brake_reverse_seconds": 0.50,
            "live_tuning.crosswalk_brake_reverse_speed": 0.06,
            "live_tuning.crosswalk_approach_distance_m": 0.30,
            "live_tuning.posture_height_m": 0.15,
            "live_tuning.posture_pitch_deg": 25.0,
        }
    )
    assert reason == ""
    assert updated.crosswalk_stop_seconds == pytest.approx(3.0)
    assert updated.crosswalk_brake_reverse_enabled is True
    assert updated.crosswalk_brake_reverse_seconds == pytest.approx(0.50)
    assert updated.crosswalk_brake_reverse_speed == pytest.approx(0.06)
    assert updated.posture_pitch_deg == pytest.approx(25.0)


def test_start_event_is_software_only_and_waits_for_complete_pattern():
    tuning, reason = LiveRaceTuning.defaults().with_updates(
        {
            "live_tuning.buzzer_enabled": 1,
            "live_tuning.buzzer_start_count": 2,
            "live_tuning.buzzer_start_on_seconds": 0.10,
            "live_tuning.buzzer_start_off_seconds": 1.00,
        }
    )
    assert reason == ""
    event = SoftwareBuzzerEvents().emit("start", tuning, now=10.0)
    assert event.delivery == "software_only"
    assert event.complete_at == pytest.approx(11.20)


@pytest.mark.parametrize(
    "changes",
    (
        {"live_tuning.step_enabled": 2},
        {"live_tuning.crosswalk_stop_seconds": 16.0},
        {"live_tuning.crosswalk_brake_reverse_enabled": 2},
        {"live_tuning.crosswalk_brake_reverse_seconds": 2.1},
        {"live_tuning.crosswalk_brake_reverse_speed": 0.31},
        {"live_tuning.drive_min_forward_speed": 0.30, "live_tuning.drive_forward_speed": 0.10},
        {"live_tuning.drive_forward_speed": -0.01},
        {"live_tuning.drive_start_soft_enabled": 2},
        {"live_tuning.drive_start_soft_seconds": -0.01},
        {"live_tuning.drive_start_soft_speed": 0.60, "live_tuning.drive_forward_speed": 0.50},
        {"live_tuning.roundabout_forward_speed": float("nan")},
        {"live_tuning.drive_max_turn_speed": float("inf")},
        {"live_tuning.not_allowed": 1},
    ),
)
def test_invalid_update_is_rejected_atomically(changes):
    current = LiveRaceTuning.defaults()
    updated, reason = current.with_updates(changes)
    assert updated is current
    assert reason
