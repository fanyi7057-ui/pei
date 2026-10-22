"""Tests for save-to-apply live tuning embedded in the real ICN YAML."""

from pathlib import Path
import sys

import pytest
import yaml


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from race_runtime import LiveRaceTuning, LiveTuningFileWatcher


VALID_FILE = """wheelleg_vision_race:
  ros__parameters:
    camera:
      width: 640
    live_tuning:
      step_enabled: 0
      crosswalk_enabled: 1
      crosswalk_confirm_frames: 2
      crosswalk_stop_seconds: 3.0
      crosswalk_delay_after_roundabout: 1.5
      crosswalk_approach_distance_m: 0.30
      drive_forward_speed: 0.28
      drive_min_forward_speed: 0.10
      drive_max_turn_speed: 0.70
      drive_linear_accel: 0.42
      drive_angular_accel: 2.50
      drive_curve_slowdown: 0.72
      drive_curve_speed_cap: 0.17
      drive_curve_strong_speed_cap: 0.145
      drive_edge_rescue_speed_cap: 0.085
      drive_edge_rescue_max_turn_speed: 0.42
      drive_hard_turn_speed_cap: 0.105
      drive_hard_turn_max_turn_speed: 0.56
      drive_big_turn_loss_seconds: 1.00
      drive_big_turn_loss_speed: 0.075
      drive_big_turn_loss_turn: 0.34
      drive_preturn_enabled: 1
      drive_preturn_confirm_frames: 2
      drive_preturn_far_error_threshold: 0.12
      drive_preturn_turn_speed: 0.45
      drive_preturn_seconds: 0.30
      drive_preturn_speed: 0.18
      roundabout_forward_speed: 0.19
      roundabout_exit_speed: 0.105
      roundabout_entry_turn_speed: 0.34
      roundabout_entry_turn_distance: 1.00
      roundabout_exit_min_turn_speed: -0.03
      roundabout_exit_max_turn_speed: 0.22
      roundabout_follow_min_turn_speed: -0.04
      roundabout_blind_recovery_speed: 0.10
      roundabout_blind_recovery_left_turn: 0.16
      roundabout_blue_guard_speed: 0.045
      roundabout_blue_guard_turn_speed: 0.30
      roundabout_approach_speed: 0.20
      roundabout_approach_distance_m: 1.20
      posture_height_m: 0.15
      posture_roll_deg: 0.0
      posture_pitch_deg: 28.0
      posture_slide_m: 0.0
      buzzer_enabled: 0
      buzzer_start_count: 2
      buzzer_start_on_seconds: 0.10
      buzzer_start_off_seconds: 1.00
      buzzer_crosswalk_count: 1
      buzzer_crosswalk_on_seconds: 1.00
      buzzer_crosswalk_off_seconds: 0.00
      buzzer_finish_count: 1
      buzzer_finish_on_seconds: 1.00
      buzzer_finish_off_seconds: 0.00
"""

# Extend this historical baseline with the newly added step fields while
# preserving the independent drive/crosswalk values exercised below.
_fixture_document = yaml.safe_load(VALID_FILE)
_fixture_values = _fixture_document["wheelleg_vision_race"]["ros__parameters"]["live_tuning"]
for _name, _value in LiveRaceTuning.parameter_defaults().items():
    _fixture_values.setdefault(_name.removeprefix("live_tuning."), _value)
VALID_FILE = yaml.safe_dump(_fixture_document, sort_keys=False)

UNBOUNDED_SPEED_FILE = (
    VALID_FILE.replace("drive_forward_speed: 0.28", "drive_forward_speed: 0.80")
    .replace("drive_min_forward_speed: 0.10", "drive_min_forward_speed: 0.70")
    .replace("drive_max_turn_speed: 0.70", "drive_max_turn_speed: 3.50")
    .replace("roundabout_forward_speed: 0.19", "roundabout_forward_speed: 0.90")
    .replace("roundabout_exit_speed: 0.105", "roundabout_exit_speed: 0.80")
)


def test_missing_icn_config_is_rejected_without_creating_a_second_file(tmp_path):
    path = tmp_path / "race_tuning.yaml"
    current = LiveRaceTuning.defaults()

    result = LiveTuningFileWatcher(path).poll(current)

    assert result.kind == "rejected"
    assert result.tuning is current
    assert not path.exists()


def test_complete_live_tuning_section_in_icn_config_applies_atomically(tmp_path):
    path = tmp_path / "race_tuning.yaml"
    path.write_text(VALID_FILE, encoding="utf-8")

    result = LiveTuningFileWatcher(path).poll(LiveRaceTuning.defaults())

    assert result.kind == "loaded"
    assert result.tuning.crosswalk_stop_seconds == pytest.approx(3.0)
    assert result.tuning.drive_forward_speed == pytest.approx(0.28)
    assert result.tuning.posture_pitch_deg == pytest.approx(28.0)
    assert result.tuning.drive_preturn_enabled is True


def test_complete_yaml_with_unbounded_speed_values_applies_atomically(tmp_path):
    path = tmp_path / "race_tuning.yaml"
    path.write_text(UNBOUNDED_SPEED_FILE, encoding="utf-8")

    result = LiveTuningFileWatcher(path).poll(LiveRaceTuning.defaults())

    assert result.kind == "loaded"
    assert result.tuning.drive_forward_speed == pytest.approx(0.80)
    assert result.tuning.roundabout_exit_speed == pytest.approx(0.80)


def test_complete_yaml_applies_original_ring_motion_controls(tmp_path):
    path = tmp_path / "race_tuning.yaml"
    path.write_text(VALID_FILE, encoding="utf-8")

    result = LiveTuningFileWatcher(path).poll(LiveRaceTuning.defaults())

    assert result.kind == "loaded"
    assert result.tuning.roundabout_blind_recovery_speed == pytest.approx(0.10)
    assert result.tuning.roundabout_entry_turn_speed == pytest.approx(0.34)
