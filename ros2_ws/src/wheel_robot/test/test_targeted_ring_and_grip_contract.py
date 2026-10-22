from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'scripts' / 'icn_adaptive.py').read_text(encoding='utf-8')


def test_second_lap_marker_pass_loss_keeps_useful_speed_without_changing_timer():
    assert 'MARKER_PASS_LOSS_SPEED_FLOOR = 0.38' in SOURCE
    assert 'roundabout_marker1_straight_seconds' in SOURCE
    assert 'and not marker_pass_active' in SOURCE


def test_fast_entry_has_short_explicit_left_commit_when_handoff_frame_is_lost():
    assert 'FAST_ENTRY_COMMIT_SECONDS = 0.45' in SOURCE
    assert 'FAST_ENTRY_COMMIT_MIN_LEFT = 0.60' in SOURCE
    assert 'entry_min_left = max(entry_min_left, FAST_ENTRY_COMMIT_MIN_LEFT)' in SOURCE


def test_exit_zone_forced_left_is_one_shot_after_line_reacquisition():
    assert '_roundabout_exit_zone_force_left_started' in SOURCE
    assert '_roundabout_exit_zone_line_reacquired' in SOURCE
    assert 'and not self._roundabout_exit_zone_line_reacquired' in SOURCE
    assert '出口区强左找线已完成' in SOURCE


def test_reacquired_exit_zone_filters_left_branch_before_following_main():
    assert 'roundabout_exit_main_branch_filtered' in SOURCE
    assert 'post_exit_main_valid' in SOURCE
    assert 'roundabout_left_branch_raw' in SOURCE


def test_normal_patrol_grip_acts_only_while_line_is_still_visible():
    assert 'def _normal_patrol_grip_command' in SOURCE
    assert 'result.line_valid' in SOURCE
    assert 'NORMAL_GRIP_START_ERROR = 0.16' in SOURCE
    assert 'NORMAL_GRIP_MAX_TURN = 0.72' in SOURCE
    assert 'self.phase in ("SEEK_ROUNDABOUT", "SEEK_FINISH")' in SOURCE


def test_roundabout_is_hard_blocked_until_step_done_and_four_second_cooldown():
    assert 'not bool(getattr(self, "_step_done_this_lap", False))' in SOURCE
    assert '_step_roundabout_enable_after' in SOURCE
    assert 'now < step_cooldown_until' in SOURCE


def test_final_second_lap_sprint_is_double_speed_and_zero_angular():
    assert 'FINAL_SPRINT_SPEED_MULTIPLIER' in SOURCE
    assert 'final_straight_double_speed' in SOURCE
    assert 'self.phase in ("SEEK_FINISH", "FINISH_PASS")' in SOURCE
    assert 'return limited_linear, 0.0' in SOURCE
