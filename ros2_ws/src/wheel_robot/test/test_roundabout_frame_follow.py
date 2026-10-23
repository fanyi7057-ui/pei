"""Regression checks for the restored original ring and zebra-crossing flow."""

from pathlib import Path
import sys
from types import SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

from test_visual_step_strategy import node_method


def test_original_ring_entry_is_fixed_left_turn_without_visual_overlay():
    method = node_method(
        "_planned_command",
        dict(ROUNDABOUT_FORWARD_SPEED=0.40, ROUNDABOUT_ENTRY_LEFT_ANGULAR=0.98),
    )
    node = SimpleNamespace(phase="ROUNDABOUT")

    assert method(node, SimpleNamespace(line_valid=True), 1.0) == (
        0.40,
        0.98,
        "icn_entry_left",
    )


def test_fast_roundabout_entry_keeps_forcing_left_until_inner_line_is_confirmed():
    method = node_method(
        "_planned_command",
        dict(
            ROUNDABOUT_FAST_ENTRY_FORWARD_SPEED=0.32,
            ROUNDABOUT_FAST_ENTRY_LEFT_TURN_SPEED=1.10,
        ),
    )
    node = SimpleNamespace(phase="ROUNDABOUT_FAST_ENTRY")

    assert method(node, SimpleNamespace(line_valid=False), 1.0) == (
        0.32,
        1.10,
        "roundabout_fast_entry_left_search",
    )


def test_original_ring_approach_keeps_line_following_until_entry_point():
    method = node_method("_planned_command", dict(ROUNDABOUT_APPROACH_SPEED=0.40))
    node = SimpleNamespace(
        phase="ROUNDABOUT_APPROACH",
        _normal_angular=lambda result, now: -0.15,
    )

    assert method(node, SimpleNamespace(line_valid=True), 1.0) == (
        0.40,
        -0.15,
        "roundabout_approach_tracking",
    )


def test_original_crosswalk_returns_directly_to_yellow_finish_search():
    source = (PACKAGE / "scripts" / "icn.py").read_text(encoding="utf-8")

    assert "CROSSWALK_CLEAR" not in source
    assert "CrosswalkFinishPolicy" not in source
    assert "CrosswalkTimeGate" not in source
    assert 'self._set_phase("SEEK_FINISH", now)' in source


def test_removed_visual_overlay_helper_is_not_imported_by_race_node():
    source = (PACKAGE / "scripts" / "icn.py").read_text(encoding="utf-8")

    assert "RoundaboutFrameFollower" not in source
    assert "roundabout_visual_" not in source


def test_first_turn_only_runs_in_seek_roundabout_and_never_in_ring():
    first_turn = SimpleNamespace(
        done=False,
        update=lambda *args: SimpleNamespace(
            linear=0.25,
            angular=0.60,
            mode="first_turn_tracking",
        ),
    )
    method = node_method(
        "_planned_command",
        dict(
            PREDICTIVE_TURN_ENABLED=False,
            PREDICTIVE_TURN_FAR_ERROR_THRESHOLD=0.07,
            MAX_ANGULAR_SPEED=2.0,
            MINIMUM_FORWARD_SPEED=0.1,
            FORWARD_SPEED=0.5,
            CURVE_SLOWDOWN=0.5,
            CURVE_CENTER_RIDING_ENABLED=False,
            SECOND_LAP_PRESTEP_SLOW_ENABLED=False,
            BIG_TURN_LOSS_MIN_RECENT_TURN=0.2,
            ROUNDABOUT_FORWARD_SPEED=0.40,
            ROUNDABOUT_ENTRY_LEFT_ANGULAR=0.98,
        ),
    )
    result = SimpleNamespace(
        line_valid=True,
        near_error=0.0,
        mid_error=-0.12,
        far_error=-0.28,
        valid_bands=6,
        reason="tracking",
    )
    node = SimpleNamespace(
        phase="SEEK_ROUNDABOUT",
        _first_turn_follower=first_turn,
        _live_tuning=SimpleNamespace(),
        _last_processed_sequence=4,
        _normal_angular=lambda *_: 0.0,
        _last_valid_line_time=0.0,
        _last_good_angular=0.0,
        _last_curve_turn_angular=0.0,
        _last_curve_turn_time=0.0,
        _step_waiting_alignment=False,
        _post_step_reacquire_active=False,
        _predictive_turn_until=0.0,
        _predictive_turn_wait_for_near=False,
        lap_count=0,
        _step_done_this_lap=True,
        vision=SimpleNamespace(_roundabout_blue_guard_active=False),
    )

    assert method(node, result, 1.0)[2] == "first_turn_tracking"
    node.phase = "ROUNDABOUT"
    assert method(node, result, 1.1) == (0.40, 0.98, "icn_entry_left")


def test_normal_loss_keeps_last_trusted_speed_and_turns_left():
    method = node_method(
        "_planned_command",
        dict(
            LINE_LOSS_LEFT_ENABLED=True,
            LINE_LOSS_LEFT_SPEED_RATIO=1.0,
            LINE_LOSS_LEFT_TURN_SPEED=0.60,
            LINE_LOSS_LEFT_TIMEOUT_SECONDS=1.5,
            PREDICTIVE_TURN_ENABLED=False,
            BIG_TURN_LOSS_SECONDS=0.0,
            SHORT_LOSS_SECONDS=0.0,
            SEARCH_LOSS_SECONDS=0.0,
        ),
    )
    node = SimpleNamespace(
        phase="SEEK_CROSSWALK",
        _last_trusted_linear=0.42,
        _last_valid_line_time=1.0,
        _last_curve_turn_time=0.0,
        _last_curve_turn_angular=0.0,
        _step_waiting_alignment=False,
        _post_step_reacquire_active=False,
        _predictive_turn_until=0.0,
        _predictive_turn_wait_for_near=False,
    )
    result = SimpleNamespace(line_valid=False)

    assert method(node, result, 1.1) == (0.42, 0.60, "normal_left_line_recovery")


def test_normal_loss_left_recovery_does_not_override_ring_or_timeout_stop():
    source = (PACKAGE / "scripts" / "icn.py").read_text(encoding="utf-8")

    assert 'self.phase in ("SEEK_ROUNDABOUT", "SEEK_CROSSWALK", "SEEK_FINISH")' in source
    assert "elapsed <= LINE_LOSS_LEFT_TIMEOUT_SECONDS" in source
