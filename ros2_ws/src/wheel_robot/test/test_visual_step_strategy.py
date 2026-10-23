"""Behavior tests of step decisions and ring takeover without ROS/hardware."""

import ast
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import threading

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from race_runtime import LiveRaceTuning, VisualStepController, parse_live_tuning_file


def observe(controller, now, tuning=None, **kwargs):
    inputs = dict(pose=(0.0, 0.0, 0.30), camera_ok=True,
                  fresh_frame=True, line_valid=True, near_error=0.0, mid_error=0.0)
    inputs.update(kwargs)
    return controller.update(now, tuning or LiveRaceTuning.defaults(), **inputs)


def aligned_controller():
    controller = VisualStepController()
    controller.start(10.0, (0.0, 0.0, 0.30))
    observe(controller, 10.1)
    observe(controller, 10.2)
    assert controller.state == "VISUAL_SPRINT"
    return controller


def jumping_controller():
    controller = aligned_controller()
    observe(controller, 10.4, pose=(0.06, 0.0, 0.30), line_valid=False)
    command = observe(controller, 10.5, pose=(0.07, 0.0, 0.30), line_valid=False)
    assert command.jump
    return controller


def test_reverse_requires_center_and_heading_then_records_latest_yaw():
    controller = VisualStepController()
    controller.start(10.0, (0.0, 0.0, 0.30))
    command = observe(controller, 10.1, near_error=0.30, mid_error=0.30)
    assert command.linear == -0.06
    assert controller.state == "VISUAL_ALIGN"
    observe(controller, 10.2, near_error=-0.15, mid_error=0.15)
    assert controller.aligned_frames == 0  # centred mean alone is insufficient
    observe(controller, 10.3, pose=(-0.02, 0.0, 0.45))
    observe(controller, 10.4, pose=(-0.03, 0.0, 0.50))
    assert controller.state == "VISUAL_SPRINT"
    assert controller.target_yaw == 0.50


def test_reverse_correction_sign_and_switch():
    controller = VisualStepController()
    controller.start(0.0, (0.0, 0.0, 0.0))
    right_line = observe(controller, 0.1, near_error=0.30, mid_error=0.30)
    assert right_line.angular > 0  # reverse lateral correction, unlike forward
    tuning = replace(LiveRaceTuning.defaults(), step_reverse_align_enabled=False)
    stationary = observe(controller, 0.2, tuning, near_error=0.30, mid_error=0.30)
    assert stationary.linear == 0
    assert stationary.angular < 0


def test_alignment_timeout_exits_to_sprint_when_line_visible():
    controller = VisualStepController()
    controller.start(0.0, (0.0, 0.0, 0.0))
    observe(controller, 3.0, near_error=0.40, mid_error=0.40)
    assert controller.state == "VISUAL_SPRINT"
    assert "超时" in controller.reason


def test_alignment_timeout_without_line_stops_and_never_jumps():
    controller = VisualStepController()
    controller.start(0.0, (0.0, 0.0, 0.0))
    command = observe(controller, 3.0, line_valid=False)
    assert command.linear == 0
    assert not command.jump
    assert controller.state == "STOPPED"


def test_repeated_camera_frame_does_not_confirm_alignment():
    controller = VisualStepController()
    controller.start(0.0, (0.0, 0.0, 0.0))
    observe(controller, 0.1)
    for tick in (0.2, 0.3, 0.4):
        observe(controller, tick, fresh_frame=False)
    assert controller.aligned_frames == 1
    assert controller.state == "VISUAL_ALIGN"


def test_sprint_ramps_velocity_and_holds_recorded_heading_across_pi():
    controller = aligned_controller()
    command = observe(controller, 10.3, pose=(0.03, 0.0, 0.10))
    assert command.linear == pytest.approx(0.065)
    assert command.angular > 0
    controller.target_yaw = -math.pi + 0.01
    command = observe(controller, 10.4, pose=(0.06, 0.0, math.pi - 0.01))
    assert command.angular == pytest.approx(0.024)


def test_sprint_requires_distance_time_and_distinct_loss_frames():
    controller = aligned_controller()
    observe(controller, 10.25, line_valid=False)
    assert controller.lost_frames == 0
    observe(controller, 10.4, pose=(0.06, 0.0, 0.30), line_valid=False)
    repeated = observe(controller, 10.45, pose=(0.065, 0.0, 0.30), fresh_frame=False, line_valid=False)
    assert not repeated.jump
    assert controller.lost_frames == 1
    command = observe(controller, 10.5, pose=(0.07, 0.0, 0.30), line_valid=False)
    assert command.jump
    assert command.linear == 0.60


@pytest.mark.parametrize("failure", ["camera", "odom"])
def test_sensor_failure_is_not_line_loss(failure):
    controller = aligned_controller()
    kwargs = dict(camera_ok=False) if failure == "camera" else dict(pose=None)
    command = observe(controller, 10.4, line_valid=False, **kwargs)
    assert controller.state == "STOPPED"
    assert not command.jump
    assert command.linear == 0


def test_jump_switch_off_stops_on_loss():
    controller = aligned_controller()
    tuning = replace(LiveRaceTuning.defaults(), step_jump_on_loss_enabled=False)
    observe(controller, 10.4, tuning, pose=(0.06, 0.0, 0.30), line_valid=False)
    command = observe(controller, 10.5, tuning, pose=(0.07, 0.0, 0.30), line_valid=False)
    assert controller.state == "STOPPED"
    assert not command.jump


@pytest.mark.parametrize("now,pose", [(13.2, (0.1, 0.0, 0.30)), (10.5, (1.20, 0.0, 0.30))])
def test_sprint_time_or_distance_exit_does_not_jump(now, pose):
    controller = aligned_controller()
    command = observe(controller, now, pose=pose, line_valid=False)
    assert controller.state == "STOPPED"
    assert not command.jump


def test_pulse_ends_even_on_camera_loss_then_stops():
    controller = jumping_controller()
    assert observe(controller, 10.6, camera_ok=False, fresh_frame=False).jump
    command = observe(controller, 10.8, camera_ok=False, fresh_frame=False)
    assert not command.jump
    assert command.linear == 0
    assert controller.state == "STOPPED"


def test_landing_protection_then_two_frames_restore_following_without_repeat_jump():
    controller = jumping_controller()
    command = observe(controller, 10.8, pose=(0.10, 0.0, 0.15))
    assert controller.state == "VISUAL_REACQUIRE"
    assert not command.jump
    assert command.angular > 0
    observe(controller, 11.0, pose=(0.12, 0.0, 0.15))
    assert controller.found_frames == 0
    observe(controller, 11.4, pose=(0.16, 0.0, 0.15))
    assert controller.state == "VISUAL_REACQUIRE"
    observe(controller, 11.5, pose=(0.18, 0.0, 0.15))
    assert controller.state == "DONE"
    assert not observe(controller, 11.6, line_valid=False).jump


def test_reacquisition_timeout_holds_stopped():
    controller = jumping_controller()
    observe(controller, 10.8)
    command = observe(controller, 14.7, line_valid=False)
    assert controller.state == "STOPPED"
    assert command.linear == 0


def test_all_new_values_save_apply_and_modify_active_timeout():
    controller = VisualStepController()
    controller.start(0.0, (0.0, 0.0, 0.0))
    text = (PACKAGE / "config/race_tuning.yaml").read_text(encoding="utf-8")
    tuning, reason = parse_live_tuning_file(text, LiveRaceTuning.defaults())
    assert not reason
    changed, reason = tuning.with_updates({"live_tuning.step_reverse_align_timeout_seconds": 1.0})
    assert not reason
    observe(controller, 1.1, changed, near_error=0.40, mid_error=0.40)
    assert controller.state == "VISUAL_SPRINT"


@pytest.mark.parametrize("change", [
    {"live_tuning.step_jump_min_height": 0.34, "live_tuning.step_jump_max_height": 0.15},
    {"live_tuning.step_sprint_max_seconds": 0},
    {"live_tuning.step_sprint_accel": float("nan")},
    {"live_tuning.step_line_loss_confirm_frames": 0},
    {"live_tuning.step_landing_seconds": 5.0},
])
def test_invalid_step_snapshot_rejected_atomically(change):
    current = LiveRaceTuning.defaults()
    updated, reason = current.with_updates(change)
    assert reason
    assert updated is current


def node_method(name, namespace):
    """Run actual node decisions with simple substitutes, no ROS imports."""
    namespace.setdefault("STEP_OPERATOR_REQUIRE_TURN_GATE", False)
    namespace.setdefault(
        "STEP_TRIGGER_PHASES", ("SEEK_ROUNDABOUT", "SEEK_CROSSWALK", "SEEK_FINISH")
    )
    tree = ast.parse((PACKAGE / "scripts/icn.py").read_text(encoding="utf-8"))
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), "icn_decision", "exec"), namespace)
    return namespace[name]


@pytest.mark.parametrize("gate,score,frames,expected", [(True, 0.82, 1, "SEEK_ROUNDABOUT"), (True, 0.82, 3, "ROUNDABOUT_APPROACH"), (True, 0.81, 7, "ROUNDABOUT_APPROACH"), (False, 0.1, 10, "SEEK_ROUNDABOUT")])
def test_actual_original_ring_evidence_requires_confirmation(gate, score, frames, expected):
    constants = dict(ROUNDABOUT_GATE_MIN_TRAVEL=1.0, ROUNDABOUT_GATE_DETECT_DELAY=1.0,
        ROUNDABOUT_FAST_ENTRY_ENABLED=False,
        ROUNDABOUT_GATE_NO_ODOM_FALLBACK_SECONDS=5.0, ROUNDABOUT_SECOND_LAP_USE_FIRST_POSITION=False,
        ROUNDABOUT_GATE_STRONG_SCORE=0.82, ROUNDABOUT_GATE_WEAK_SCORE=0.78,
        ROUNDABOUT_GATE_EVIDENCE_MAX=5.0, ROUNDABOUT_GATE_EVIDENCE_STRONG_GAIN=1.0,
        ROUNDABOUT_GATE_EVIDENCE_WEAK_GAIN=0.35, ROUNDABOUT_GATE_EVIDENCE_DECAY=0.30,
        ROUNDABOUT_GATE_EVIDENCE_TRIGGER=2.4,
        ROUNDABOUT_GATE_CONFIRM_DELAY_ALLOWANCE_M=0.08, ROUNDABOUT_GATE_CONFIRM_DELAY_MAX_COMP_M=0.15,
        ROUNDABOUT_APPROACH_SPEED=0.20, ROUNDABOUT_APPROACH_LEFT_TURN=0.20,
        clamp=lambda x, lo, hi: max(lo, min(x, hi)))
    sync = node_method("_sync_course_state", constants)
    node = SimpleNamespace(phase="SEEK_ROUNDABOUT", phase_enter_time=0.0, _seek_distance=2.0,
        lap_count=0, _roundabout_gate_first_seen_seek=None, _roundabout_gate_evidence=0.0,
        _first_roundabout_gate_seek=None, _update_seek_roundabout_progress=lambda: None,
        _odom_snapshot=lambda: (0, 0, 0), _roundabout_approach_target=lambda: 1.2,
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    node._set_phase = lambda phase, now: setattr(node, "phase", phase)
    for _ in range(frames):
        sync(node, SimpleNamespace(roundabout_gate=gate, roundabout_gate_score=score), 3.0)
    assert node.phase == expected


def test_actual_approach_restores_original_bidirectional_smoothing():
    limit = node_method("_limit_command", dict(CONTROL_RATE=15.0,
        MAX_LINEAR_ACCELERATION=0.42, MAX_ANGULAR_ACCELERATION=2.50,
        clamp=lambda x, lo, hi: max(lo, min(x, hi))))
    approach = lambda current, target, step: current + max(-step, min(target - current, step))
    node = SimpleNamespace(phase="ROUNDABOUT_APPROACH", _last_command=(0.45, -0.70),
                           _last_motion_update_time=0.0, _approach=approach)
    linear, angular = limit(node, 0.20, 0.20, 0.05)
    assert angular < 0.0  # approach follows the line, including right corrections
    assert linear < 0.45
    node.phase = "SEEK_ROUNDABOUT"
    node._last_motion_update_time = 0.0
    assert limit(node, 0.20, 0.20, 0.05)[1] < 0  # ordinary smoother remains intact


def test_actual_step_master_off_never_starts_new_controller():
    method = node_method("_try_lock_step", {"STEP_ENABLED": False})
    node = SimpleNamespace(_step_done_this_lap=False, _step_state="IDLE")
    assert not method(node, None, 1.0)


@pytest.mark.parametrize("confirmed,expected", [(False, False), (True, True)])
def test_actual_step_trigger_requires_depth_confirmation(confirmed, expected):
    method = node_method("_try_start_visual_step", dict(
        math=math, STEP_RACE_CANDIDATE_MAX_DISTANCE_M=1.20,
        STEP_RACE_CANDIDATE_MIN_HEIGHT_M=0.025, STEP_RACE_CANDIDATE_MAX_HEIGHT_M=0.10,
        STEP_RACE_CANDIDATE_MIN_CONFIDENCE=0.42, STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER=2.0))
    node = SimpleNamespace(_latest_step_detection=SimpleNamespace(
        detected=confirmed, distance=0.70, height=0.05, confidence=0.90),
        _step_race_candidate_evidence=0.0, _step_detection_fresh=lambda _: True,
        _step_odom_fresh=lambda _: True, _odom_snapshot=lambda: (0.0, 0.0, 0.30),
        _visual_step=VisualStepController())
    node._set_step_state = lambda state, reason: setattr(node, "_step_state", state)
    assert method(node, None, 1.0) == expected
    assert node._visual_step.active == expected


def test_actual_ros_dry_run_never_publishes_jump_true_even_when_decision_jumps():
    method = node_method("_run_visual_step_cycle", dict(
        ENABLE_MOTION=False, IMAGE_TIMEOUT_SECONDS=0.5, STEERING_SIGN=-1.0,
        wrap_pi=lambda v: math.atan2(math.sin(v), math.cos(v))))
    outputs = []
    result = SimpleNamespace(line_valid=False, near_error=0.0, mid_error=0.0)
    frame = SimpleNamespace(copy=lambda: object())
    node = SimpleNamespace(_frame_lock=threading.Lock(), _camera_healthy=True,
        _latest_frame=frame, _latest_frame_time=10.6, _frame_sequence=1,
        _visual_step_sequence=0, _visual_step=jumping_controller(),
        _visual_step_result=None, _live_tuning=LiveRaceTuning.defaults(),
        vision=SimpleNamespace(process=lambda *args, **kw: result),
        _record_raw_frame=lambda _: None, _step_odom_fresh=lambda _: True,
        _odom_snapshot=lambda: (0.08, 0.0, 0.30),
        _publish_zero=lambda: outputs.append("zero"),
        _publish_jump_false_only=lambda: outputs.append("jump_false"),
        _draw_debug=lambda *args: None, _write_race_log=lambda *args: None)
    node._set_step_state = lambda state, reason: setattr(node, "_step_state", state)
    linear, angular, mode = method(node, 10.6)
    assert linear == 0.60  # intended decision remains visible in dry-run logs
    assert outputs == ["zero", "jump_false"]
    assert mode.startswith("DRY_RUN/")


def test_operator_step_gate_accepts_confirmed_depth_without_85deg():
    method = node_method("_try_operator_gated_step", dict(
        math=math,
        STEP_ENABLED=True,
        STEP_RACE_CANDIDATE_MAX_DISTANCE_M=1.20,
        STEP_RACE_CANDIDATE_MIN_HEIGHT_M=0.025,
        STEP_RACE_CANDIDATE_MAX_HEIGHT_M=0.085,
        STEP_RACE_CANDIDATE_MIN_CONFIDENCE=0.42,
        STEP_OPERATOR_WAIT_MAX_DISTANCE_M=1.05,
        STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER=2.40,
        STEP_PRESEEN_HOLD_SECONDS=2.5,
        STEP_PRESEEN_MAX_FORWARD_TRAVEL_M=0.80,
    ))
    det = SimpleNamespace(detected=True, distance=0.82, height=0.05, confidence=0.95)
    statuses = []
    node = SimpleNamespace(
        _step_done_this_lap=False,
        _step_state="IDLE",
        phase="SEEK_ROUNDABOUT",
        _step_arm_window_ready=lambda: (True, "ok"),
        _step_aux_confirmation_ready=lambda *_: True,
        _latest_step_detection=det,
        _step_preseen_this_lap=False,
        _step_preseen_detection=None,
        _step_preseen_distance=float("nan"),
        _step_preseen_seek_distance=0.0,
        _step_preseen_time=0.0,
        _step_turn_ready=False,
        _step_detection_fresh=lambda _: True,
        _step_race_candidate_evidence=4.0,
        _step_wait_detection=None,
        _step_wait_distance=float("nan"),
        _step_execute_requested=True,
        _step_operator_auto_execute=True,
        _step_waiting_alignment=False,
        _seek_distance=0.40,
        _step_turn_deg=lambda: 12.0,
        _publish_operator_status=lambda stage, payload="": statuses.append((stage, payload)),
        _step_wait_status_last_publish=0.0,
        lap_count=0,
    )
    node._set_step_state = lambda state, reason: setattr(node, "_step_state", state)
    assert method(node, SimpleNamespace(), 1.0)
    assert node._step_state == "WAIT_TOPIC"
    assert statuses and statuses[-1][0] == "STEP_WAIT"


def test_operator_step_gate_stops_for_topic_after_turn_and_distance_gate():
    method = node_method("_try_operator_gated_step", dict(
        math=math,
        STEP_ENABLED=True,
        STEP_RACE_CANDIDATE_MAX_DISTANCE_M=1.20,
        STEP_RACE_CANDIDATE_MIN_HEIGHT_M=0.025,
        STEP_RACE_CANDIDATE_MAX_HEIGHT_M=0.085,
        STEP_RACE_CANDIDATE_MIN_CONFIDENCE=0.42,
        STEP_OPERATOR_WAIT_MAX_DISTANCE_M=1.05,
        STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER=2.40,
        STEP_PRESEEN_HOLD_SECONDS=2.5,
        STEP_PRESEEN_MAX_FORWARD_TRAVEL_M=0.80,
    ))
    det = SimpleNamespace(detected=True, distance=0.82, height=0.05, confidence=0.95)
    statuses = []
    node = SimpleNamespace(
        _step_done_this_lap=False,
        _step_state="IDLE",
        phase="SEEK_ROUNDABOUT",
        _step_arm_window_ready=lambda: (True, "ok"),
        _step_aux_confirmation_ready=lambda *_: True,
        _latest_step_detection=det,
        _step_preseen_this_lap=False,
        _step_preseen_detection=None,
        _step_preseen_distance=float("nan"),
        _step_preseen_seek_distance=0.0,
        _step_preseen_time=0.0,
        _step_turn_ready=True,
        _step_detection_fresh=lambda _: True,
        _step_race_candidate_evidence=4.0,
        _step_wait_detection=None,
        _step_wait_distance=float("nan"),
        _step_execute_requested=True,
        _step_operator_auto_execute=True,
        _step_waiting_alignment=True,
        _seek_distance=1.40,
        _step_turn_deg=lambda: 88.0,
        _publish_operator_status=lambda stage, payload="": statuses.append((stage, payload)),
        _step_wait_status_last_publish=0.0,
        lap_count=0,
    )
    node._set_step_state = lambda state, reason: setattr(node, "_step_state", state)
    assert method(node, SimpleNamespace(), 2.0)
    assert node._step_state == "WAIT_TOPIC"
    assert not node._step_execute_requested
    assert not node._step_operator_auto_execute
    assert statuses and statuses[-1][0] == "STEP_WAIT"


def test_operator_step_gate_uses_recent_confirmed_preseen_when_depth_is_lost_at_85deg():
    method = node_method("_try_operator_gated_step", dict(
        math=math,
        STEP_ENABLED=True,
        STEP_RACE_CANDIDATE_MAX_DISTANCE_M=1.20,
        STEP_RACE_CANDIDATE_MIN_HEIGHT_M=0.025,
        STEP_RACE_CANDIDATE_MAX_HEIGHT_M=0.085,
        STEP_RACE_CANDIDATE_MIN_CONFIDENCE=0.42,
        STEP_OPERATOR_WAIT_MAX_DISTANCE_M=1.05,
        STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER=2.40,
        STEP_PRESEEN_HOLD_SECONDS=2.5,
        STEP_PRESEEN_MAX_FORWARD_TRAVEL_M=0.80,
    ))
    confirmed = SimpleNamespace(
        detected=True, distance=0.87, height=0.053, confidence=0.94
    )
    statuses = []
    node = SimpleNamespace(
        _step_done_this_lap=False,
        _step_state="IDLE",
        phase="SEEK_ROUNDABOUT",
        _step_arm_window_ready=lambda: (True, "ok"),
        _step_aux_confirmation_ready=lambda *_: True,
        _latest_step_detection=SimpleNamespace(
            detected=False, distance=float("nan"), height=float("nan"), confidence=0.0
        ),
        _step_preseen_this_lap=True,
        _step_preseen_detection=confirmed,
        _step_preseen_distance=0.87,
        _step_preseen_seek_distance=1.46,
        _step_preseen_time=10.0,
        _step_turn_ready=True,
        _step_detection_fresh=lambda _: False,
        _step_race_candidate_evidence=4.0,
        _step_wait_detection=None,
        _step_wait_distance=float("nan"),
        _step_execute_requested=False,
        _step_operator_auto_execute=False,
        _step_waiting_alignment=False,
        _seek_distance=1.70,
        _step_turn_deg=lambda: 87.0,
        _publish_operator_status=lambda stage, payload="": statuses.append((stage, payload)),
        _step_wait_status_last_publish=0.0,
        lap_count=0,
    )
    node._set_step_state = lambda state, reason: setattr(node, "_step_state", state)
    assert method(node, SimpleNamespace(), 11.0)
    assert node._step_state == "WAIT_TOPIC"
    assert node._step_wait_distance == pytest.approx(0.63)
    assert statuses and "source=latched_depth_odom_bridge" in statuses[-1][1]


def test_operator_step_gate_rejects_stale_preseen_snapshot():
    method = node_method("_try_operator_gated_step", dict(
        math=math,
        STEP_ENABLED=True,
        STEP_RACE_CANDIDATE_MAX_DISTANCE_M=1.20,
        STEP_RACE_CANDIDATE_MIN_HEIGHT_M=0.025,
        STEP_RACE_CANDIDATE_MAX_HEIGHT_M=0.085,
        STEP_RACE_CANDIDATE_MIN_CONFIDENCE=0.42,
        STEP_OPERATOR_WAIT_MAX_DISTANCE_M=1.05,
        STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER=2.40,
        STEP_PRESEEN_HOLD_SECONDS=2.5,
        STEP_PRESEEN_MAX_FORWARD_TRAVEL_M=0.80,
    ))
    confirmed = SimpleNamespace(
        detected=True, distance=0.87, height=0.053, confidence=0.94
    )
    node = SimpleNamespace(
        _step_done_this_lap=False, _step_state="IDLE", phase="SEEK_ROUNDABOUT",
        _step_arm_window_ready=lambda: (True, "ok"),
        _step_aux_confirmation_ready=lambda *_: True,
        _latest_step_detection=None,
        _step_preseen_this_lap=True, _step_preseen_detection=confirmed,
        _step_preseen_distance=0.87, _step_preseen_seek_distance=1.46,
        _step_preseen_time=5.0, _step_turn_ready=True,
        _step_detection_fresh=lambda _: False, _step_race_candidate_evidence=4.0,
        _step_wait_detection=None, _step_wait_distance=float("nan"),
        _step_execute_requested=False, _step_operator_auto_execute=False,
        _step_waiting_alignment=False, _seek_distance=1.70,
        _step_turn_deg=lambda: 87.0, _publish_operator_status=lambda *args: None,
        _step_wait_status_last_publish=0.0, lap_count=0,
    )
    node._set_step_state = lambda state, reason: setattr(node, "_step_state", state)
    assert not method(node, SimpleNamespace(), 10.0)
    assert node._step_state == "IDLE"


def test_5_2_aux_confirmation_accepts_stable_depth_with_pose_support():
    method = node_method("_step_aux_confirmation_ready", dict(
        math=math,
        STEP_FIXED_DEPTH_FILTER_MAX_AGE_SECONDS=0.80,
        STEP_AUX_DISTANCE_SAMPLES=3,
        STEP_AUX_MAX_DISTANCE_SPREAD_M=0.12,
        STEP_AUX_MAX_EDGE_ANGLE_RAD=0.30,
        STEP_AUX_MAX_LATERAL_OFFSET_M=0.16,
        STEP_AUX_MIN_LINE_CONFIDENCE=0.35,
        STEP_AUX_MAX_LINE_HEADING_DELTA=0.35,
        STEP_AUX_CONFIRM_MAX_WAIT_SECONDS=1.0,
        STEP_AUX_CONFIRM_REQUIRED_FRAMES=3,
    ))
    node = SimpleNamespace(
        _step_aux_confirm_started_at=0.0,
        _step_aux_confirm_frames=0,
        _step_aux_confirm_latched=False,
        _step_aux_confirm_reason="waiting_depth",
        _step_depth_history=[(1.0, 0.82), (1.1, 0.80), (1.2, 0.79)],
    )
    detection = SimpleNamespace(
        pose_valid=True, edge_angle_rad=0.08, lateral_offset_m=0.03
    )
    result = SimpleNamespace(
        line_valid=False, confidence=0.0, near_error=0.0, mid_error=0.0
    )
    assert not method(node, result, detection, 1.2)
    assert not method(node, result, detection, 1.3)
    assert method(node, result, detection, 1.4)
    assert "stable=1" in node._step_aux_confirm_reason
    assert "pose=1" in node._step_aux_confirm_reason


def test_5_2_aux_confirmation_depth_only_fallback_does_not_block_forever():
    method = node_method("_step_aux_confirmation_ready", dict(
        math=math,
        STEP_FIXED_DEPTH_FILTER_MAX_AGE_SECONDS=2.0,
        STEP_AUX_DISTANCE_SAMPLES=3,
        STEP_AUX_MAX_DISTANCE_SPREAD_M=0.12,
        STEP_AUX_MAX_EDGE_ANGLE_RAD=0.30,
        STEP_AUX_MAX_LATERAL_OFFSET_M=0.16,
        STEP_AUX_MIN_LINE_CONFIDENCE=0.35,
        STEP_AUX_MAX_LINE_HEADING_DELTA=0.35,
        STEP_AUX_CONFIRM_MAX_WAIT_SECONDS=1.0,
        STEP_AUX_CONFIRM_REQUIRED_FRAMES=3,
    ))
    node = SimpleNamespace(
        _step_aux_confirm_started_at=1.0,
        _step_aux_confirm_frames=0,
        _step_aux_confirm_latched=False,
        _step_aux_confirm_reason="waiting_depth",
        _step_depth_history=[(1.0, 0.82), (1.2, 0.81), (1.4, 0.80)],
    )
    detection = SimpleNamespace(
        pose_valid=False, edge_angle_rad=float("nan"), lateral_offset_m=float("nan")
    )
    result = SimpleNamespace(
        line_valid=False, confidence=0.0, near_error=0.0, mid_error=0.0
    )
    assert not method(node, result, detection, 2.1)
    assert not method(node, result, detection, 2.2)
    assert method(node, result, detection, 2.3)
    assert "fallback=1" in node._step_aux_confirm_reason


def test_execute_jump_topic_is_ignored_until_wait_state_then_accepted():
    method = node_method("_on_step_execute_jump", {})
    logs = []
    statuses = []
    node = SimpleNamespace(
        _step_state="IDLE",
        _step_execute_requested=False,
        lap_count=0,
        get_logger=lambda: SimpleNamespace(warn=lambda msg: logs.append(msg)),
        _publish_operator_status=lambda stage, payload="": statuses.append((stage, payload)),
    )
    method(node, SimpleNamespace(data=True))
    assert not node._step_execute_requested
    assert logs
    node._step_state = "WAIT_TOPIC"
    method(node, SimpleNamespace(data=True))
    assert node._step_execute_requested
    assert statuses[-1][0] == "STEP_EXECUTING"
