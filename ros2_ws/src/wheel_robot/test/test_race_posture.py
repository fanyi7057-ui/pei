"""Tests for the complete normal-race posture contract."""

from pathlib import Path
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
ICN_SOURCE = SCRIPTS / "icn.py"
sys.path.insert(0, str(SCRIPTS))

from race_posture import RACE_POSTURE_NAMES, RacePosture, RacePostureTransition


def test_default_race_posture_uses_selected_low_head_down_pitch():
    posture = RacePosture.from_values()

    assert posture.names == RACE_POSTURE_NAMES
    assert posture.positions == (0.15, 0.0, 28.0, 0.0)

    config = (PACKAGE / "config" / "race_tuning.yaml").read_text(encoding="utf-8")
    assert "posture_height_m: 0.15" in config
    assert "posture_pitch_deg: 25.0" in config

    race_source = ICN_SOURCE.read_text(encoding="utf-8")
    assert "posture_height_m" in race_source
    assert "posture_pitch_deg" in race_source

    stand_launch = (PACKAGE / "launch" / "stand_only.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'default_value="0.15"' in stand_launch
    assert 'default_value="28.0"' in stand_launch


def test_invalid_values_are_clamped_to_lower_controller_bounds():
    posture = RacePosture.from_values(0.01, 99.0, -99.0, 1.0)

    assert posture.positions == (0.14, 15.0, -35.0, 0.08)


def test_posture_transition_reaches_target_in_half_a_second():
    transition = RacePostureTransition(
        RacePosture.from_values(0.15, 0.0, 0.0, 0.0),
        duration_seconds=0.5,
    )
    transition.set_target(
        RacePosture.from_values(0.15, 0.0, 28.0, 0.0),
        now=10.0,
    )

    assert transition.at(10.25).pitch_deg == pytest.approx(14.0)
    assert transition.at(10.50).pitch_deg == pytest.approx(28.0)


def test_race_posture_module_is_installed_with_the_ros_package():
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/race_posture.py" in cmake


def test_race_node_uses_one_configured_posture_for_hold_and_normal_control():
    source = ICN_SOURCE.read_text(encoding="utf-8")

    assert "_race_posture_from_tuning" in source
    assert "RacePostureTransition" in source
    assert "def _publish_race_posture" in source
    assert "self._publish_race_posture()" in source


def test_jump_auxiliary_remains_an_explicit_temporary_posture_override():
    source = ICN_SOURCE.read_text(encoding="utf-8")
    jump_section = source.split("def _publish_step_aux", 1)[1].split(
        "def _publish_stand_hold", 1
    )[0]

    assert "STEP_STAND_HEIGHT_M" in jump_section
    assert "self._publish_race_posture" not in jump_section
