"""Static contracts for live race tuning deployment and ROS integration."""

import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "icn.yaml"
TUNING_CONFIG = PACKAGE / "config" / "race_tuning.yaml"
CMAKE = PACKAGE / "CMakeLists.txt"
ICN = PACKAGE / "scripts" / "icn.py"
LAUNCH = PACKAGE / "launch" / "icn.launch.py"


def test_yaml_has_safe_live_defaults_and_chinese_notes():
    text = CONFIG.read_text(encoding="utf-8")
    values = yaml.safe_load(text)["wheelleg_vision_race"]["ros__parameters"]

    tuning_text = TUNING_CONFIG.read_text(encoding="utf-8")
    tuning = yaml.safe_load(tuning_text)["wheelleg_vision_race"]["ros__parameters"]
    assert "live_tuning" not in values
    assert tuning["live_tuning"]["step_enabled"] == 1
    assert tuning["live_tuning"]["buzzer_enabled"] == 0
    assert tuning["live_tuning"]["crosswalk_stop_seconds"] == 3.0
    assert tuning["live_tuning"]["posture_pitch_deg"] == 25.0
    assert tuning["live_tuning"]["drive_preturn_enabled"] == 0
    assert tuning["live_tuning"]["drive_forward_speed"] == 0.50
    assert tuning["live_tuning"]["drive_start_soft_enabled"] == 1
    assert tuning["live_tuning"]["drive_start_soft_seconds"] == 3.00
    assert tuning["live_tuning"]["drive_start_soft_speed"] == 0.25
    assert tuning["live_tuning"]["drive_max_turn_speed"] == 0.80
    assert tuning["live_tuning"]["drive_angular_accel"] == 3.00
    assert tuning["live_tuning"]["drive_curve_speed_cap"] == 55.0
    assert tuning["live_tuning"]["roundabout_approach_speed"] == 0.40
    assert tuning["live_tuning"]["roundabout_entry_turn_speed"] == 0.98
    assert tuning["live_tuning"]["roundabout_entry_turn_distance"] == 0.66
    assert tuning["live_tuning"]["roundabout_fast_entry_enabled"] == 1
    assert tuning["live_tuning"]["roundabout_fast_entry_min_components"] == 3
    assert tuning["live_tuning"]["roundabout_fast_entry_min_left_components"] == 2
    assert tuning["live_tuning"]["roundabout_fast_entry_left_turn_speed"] > 0.0
    assert tuning["live_tuning"]["roundabout_exit_min_turn_speed"] == 0.10
    assert tuning["live_tuning"]["first_turn_enabled"] == 1
    assert tuning["live_tuning"]["first_turn_confirm_frames"] == 2
    assert tuning["live_tuning"]["first_turn_max_left_turn_speed"] == 0.80
    assert tuning["live_tuning"]["line_loss_left_enabled"] == 1
    assert tuning["live_tuning"]["line_loss_left_speed_ratio"] == 1.0
    assert tuning["live_tuning"]["line_loss_left_turn_speed"] == 0.60
    assert "crosswalk_clear_seconds" not in tuning["live_tuning"]
    assert "roundabout_visual_enabled" not in tuning["live_tuning"]
    assert "用途、单位、范围、调节效果与运行中限制" in text
    assert "race_tuning.yaml" in text
    assert "0.5 秒自动生效" in tuning_text
    assert "台阶接管" in tuning_text
    assert "0=关闭" in tuning_text
    assert "未接物理蜂鸣器" in tuning_text


def test_cmake_installs_runtime_helper_and_registers_live_tests():
    text = CMAKE.read_text(encoding="utf-8")

    assert "scripts/race_runtime.py" in text
    assert "test_live_race_tuning.py" in text
    assert "test_live_race_contract.py" in text
    assert "test_live_tuning_file_reload.py" in text


def test_package_declares_ros_parameter_callback_dependency():
    package = (PACKAGE / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>rcl_interfaces</exec_depend>" in package
    assert "<exec_depend>python3-yaml</exec_depend>" in package


def test_cmake_accepts_the_bundled_versioned_orbbec_library_at_build_time():
    text = CMAKE.read_text(encoding="utf-8")

    assert "NAMES OrbbecSDK" in text
    assert "NAMES libOrbbecSDK.so.1.10.37" in text
    assert "Orbbec SDK library not found" in text


def test_icn_has_live_callback_start_cue_and_software_events():
    text = ICN.read_text(encoding="utf-8")

    assert "add_on_set_parameters_callback" in text
    assert "def _on_live_tuning_parameters" in text
    assert "SoftwareBuzzerEvents" in text
    assert "def _run_start_cue" in text
    assert 'self._emit_buzzer_event("crosswalk"' in text
    assert 'self._emit_buzzer_event("finish"' in text
    assert "软件事件（未驱动物理蜂鸣器）" in text
    assert "台阶动作正在接管" in text


def test_icn_uses_one_live_snapshot_for_posture_file_reload_and_original_crosswalk():
    text = ICN.read_text(encoding="utf-8")

    assert "RacePostureTransition" in text
    assert "LiveTuningFileWatcher" in text
    assert 'Path("/home/orangepi/robot/ros2_ws/src/wheel_robot/config/race_tuning.yaml")' in text
    assert "self.create_timer(0.5, self._poll_live_tuning_file)" in text
    assert "def _poll_live_tuning_file" in text
    assert "CROSSWALK_CLEAR" not in text
    assert "crosswalk_clear_forward_speed" not in text
    assert "台阶动作正在接管，不能修改比赛姿态" in text


def test_icn_maps_hidden_motion_controls_from_live_tuning():
    text = ICN.read_text(encoding="utf-8")

    for mapping in (
        "MAX_LINEAR_ACCELERATION = tuning.drive_linear_accel",
        "MAX_ANGULAR_ACCELERATION = tuning.drive_angular_accel",
        "CURVE_SLOWDOWN = tuning.drive_curve_slowdown",
        "CURVE_CENTER_SPEED_CAP = tuning.drive_curve_speed_cap",
        "CURVE_CENTER_STRONG_SPEED_CAP = tuning.drive_curve_strong_speed_cap",
        "LINE_EDGE_RESCUE_SPEED_CAP = tuning.drive_edge_rescue_speed_cap",
        "LINE_HARD_TURN_SPEED_CAP = tuning.drive_hard_turn_speed_cap",
        "ROUNDABOUT_BLIND_RECOVERY_SPEED = tuning.roundabout_blind_recovery_speed",
        "ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR = tuning.roundabout_blind_recovery_left_turn",
        "CROSSWALK_APPROACH_DISTANCE = tuning.crosswalk_approach_distance_m",
        "PREDICTIVE_TURN_SPEED = tuning.drive_preturn_speed",
        "ROUNDABOUT_APPROACH_SPEED = tuning.roundabout_approach_speed",
        "ROUNDABOUT_FAST_ENTRY_FORWARD_SPEED = tuning.roundabout_fast_entry_forward_speed",
        "ROUNDABOUT_FAST_ENTRY_LEFT_TURN_SPEED = tuning.roundabout_fast_entry_left_turn_speed",
        "LINE_LOSS_LEFT_ENABLED = tuning.line_loss_left_enabled",
        "LINE_LOSS_LEFT_SPEED_RATIO = tuning.line_loss_left_speed_ratio",
        "LINE_LOSS_LEFT_TURN_SPEED = tuning.line_loss_left_turn_speed",
        "LINE_LOSS_LEFT_TIMEOUT_SECONDS = tuning.line_loss_left_timeout_seconds",
    ):
        assert mapping in text


def test_live_motion_assignments_are_declared_global_not_local_shadow_variables():
    tree = ast.parse(ICN.read_text(encoding="utf-8"))
    method = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_apply_live_tuning"
    )
    globals_in_method = {
        name for node in ast.walk(method) if isinstance(node, ast.Global) for name in node.names
    }

    assert {
        "MAX_LINEAR_ACCELERATION",
        "MAX_ANGULAR_ACCELERATION",
        "CURVE_SLOWDOWN",
        "CURVE_CENTER_SPEED_CAP",
        "CURVE_CENTER_STRONG_SPEED_CAP",
        "LINE_EDGE_RESCUE_SPEED_CAP",
        "LINE_EDGE_RESCUE_MAX_ANGULAR",
        "LINE_HARD_TURN_SPEED_CAP",
        "LINE_HARD_TURN_MAX_ANGULAR",
        "BIG_TURN_LOSS_SECONDS",
        "BIG_TURN_LOSS_SPEED",
        "BIG_TURN_LOSS_TURN",
        "ROUNDABOUT_ENTRY_LEFT_ANGULAR",
        "ROUNDABOUT_ENTRY_TURN_DISTANCE",
        "ROUNDABOUT_EXIT_MIN_ANGULAR",
        "ROUNDABOUT_EXIT_MAX_ANGULAR",
        "ROUNDABOUT_FOLLOW_MIN_ANGULAR",
        "ROUNDABOUT_BLIND_RECOVERY_SPEED",
        "ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR",
        "ROUNDABOUT_BLUE_GUARD_SPEED",
        "ROUNDABOUT_BLUE_GUARD_ANGULAR",
        "CROSSWALK_APPROACH_DISTANCE",
        "PREDICTIVE_TURN_SPEED",
        "ROUNDABOUT_APPROACH_SPEED",
    }.issubset(globals_in_method)


def test_icn_keeps_original_yellow_line_finish_after_crosswalk_stop():
    text = ICN.read_text(encoding="utf-8")
    compact = "".join(text.split())

    assert "CrosswalkFinishPolicy" not in text
    assert "second_crosswalk_finish_distance_m" not in text
    assert 'self._set_phase("SEEK_FINISH",now)' in compact
    assert "def _handle_yellow_finish" in text


def test_icn_has_predictive_turn_and_restored_original_ring_crosswalk_flow():
    text = ICN.read_text(encoding="utf-8")

    assert "PredictiveTurnGate" in text
    assert "CrosswalkTimeGate" not in text
    assert "CROSSWALK_FIRST_ENABLE_AFTER_START_SECONDS" not in text
    assert "predictive_preturn" in text
    assert "roundabout_approach_tracking" in text
    assert "icn_entry_left" in text


def test_launch_layers_the_small_tuning_file_after_base_icn_config():
    text = LAUNCH.read_text(encoding="utf-8")

    assert '"race_tuning.yaml"' in text
    assert 'LaunchConfiguration("tuning_params_file")' in text


def test_first_lap_soft_start_is_first_lap_only():
    text = ICN.read_text(encoding="utf-8")
    assert "def _first_lap_soft_start_cap" in text
    assert "self.lap_count != 0" in text
    assert "self._race_motion_start_time = now" in text
    assert "first_lap_soft_start" in text
