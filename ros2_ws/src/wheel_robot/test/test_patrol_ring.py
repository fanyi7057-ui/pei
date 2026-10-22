"""Synthetic ring topology/FSM tests. No ROS, camera, serial or actuators."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
from patrol_contour_runtime import PatrolTuning, PathEstimate
from patrol_ring import PatrolRingNavigator, RingObservation, observe_ring


def fork_mask(kind='diverging'):
    mask = np.zeros((240, 320), np.uint8)
    for index, y in enumerate([220, 195, 170, 145, 120, 95]):
        cv2.rectangle(mask, (171, y-5), (179, y+5), 255, -1)
        gap = 18+12*index if kind == 'diverging' else 78-12*index
        cv2.rectangle(mask, (171-gap, y-5), (179-gap, y+5), 255, -1)
    return mask


@pytest.mark.parametrize('kind', ['converging', 'diverging'])
def test_fork_topology_uses_left_branch_separation_toward_forward(kind):
    observation = observe_ring(fork_mask(kind), PatrolTuning())
    assert observation.kind == kind
    assert observation.main.valid and observation.branch.valid
    assert observation.branch.angle_rad < 0
    assert observation.separation_change*(1 if kind == 'diverging' else -1) > .06


def test_far_fork_and_single_dashed_line_do_not_trigger():
    mask = fork_mask()
    mask[150:] = 0
    assert observe_ring(mask, PatrolTuning()).kind == 'none'
    mask = fork_mask()
    mask[:, :160] = 0
    assert observe_ring(mask, PatrolTuning()).kind == 'none'


def test_parallel_stripes_and_plain_solid_line_are_not_forks():
    mask = np.zeros((240, 320), np.uint8)
    for y in range(100, 231, 24):
        cv2.rectangle(mask, (120, y), (200, y+5), 255, -1)
    assert observe_ring(mask, PatrolTuning()).kind == 'none'
    mask[:] = 0
    cv2.line(mask, (160, 239), (160, 80), 255, 7)
    observation = observe_ring(mask, PatrolTuning())
    assert observation.kind == 'none' and observation.solid.valid
    assert not observation.dashed.valid


def test_locked_reference_keeps_curved_dashes_not_straight_alternative():
    mask = fork_mask()
    observation = observe_ring(mask, PatrolTuning())
    tracking = observe_ring(mask, PatrolTuning(), reference=observation.branch.points)
    assert tracking.dashed.valid
    assert np.max(tracking.dashed.points[:, 0]) < 165


def path(angle=0.):
    return PathEstimate(True, angle, .9, np.array([[160., 220], [160, 190], [160, 160]]))


def obs(kind='none', dashed=True, solid=False):
    return RingObservation(kind, path(), path(-.3), path(-.25) if dashed else PathEstimate(False),
                           path() if solid else PathEstimate(False), 12, .15)


def tuning(**kwargs):
    return replace(PatrolTuning(), ring_start_delay_s=0., **kwargs)


def fresh(nav, now, x=0., y=0., rate=0.):
    nav.observe_imu(rate, now)
    nav.observe_odom(x, y, now)


def enter_ring():
    nav, t = PatrolRingNavigator(), tuning()
    for i in range(3):
        now = 1+i*.04
        fresh(nav, now)
        nav.step(obs('converging'), path(), now, t)
    assert nav.phase == 'PASS_EXIT'
    for i in range(1, 16):
        now = 1.08+i*.1
        fresh(nav, now, x=.02*i)
        nav.step(obs(), path(), now, t)
    assert nav.phase == 'SEEK_ENTRY'
    for i in range(1, 4):
        now = 2.58+i*.04
        fresh(nav, now, x=.3)
        nav.step(obs('diverging'), path(), now, t)
    assert nav.phase == 'ENTER'
    return nav, t, now


def test_first_exit_is_not_an_entry_and_entry_without_exit_is_ignored():
    nav, t = PatrolRingNavigator(), tuning()
    for i in range(6):
        nav.step(obs('diverging'), path(), 1+i*.04, t)
    assert nav.phase == 'SEEK_EXIT' and nav.turn_rad == 0.
    for i in range(3):
        now = 2+i*.04
        fresh(nav, now)
        decision = nav.step(obs('converging'), path(-.4), now, t)
    assert nav.phase == 'PASS_EXIT'
    assert decision.path.angle_rad == 0.  # main path, never the curved exit
    assert decision.speed == t.ring_speed_m_s


def test_pass_exit_requires_forward_distance_and_marker_departure():
    nav, t = PatrolRingNavigator(), tuning()
    for i in range(3):
        fresh(nav, 1+i*.04)
        nav.step(obs('converging'), path(), 1+i*.04, t)
    for i in range(1, 9):
        now = 1.08+i*.1
        fresh(nav, now, x=-.04*i)  # reversing must not satisfy pass distance
        nav.step(obs('diverging'), path(), now, t)
    assert nav.phase == 'PASS_EXIT'
    for i in range(1, 8):
        now = 2+i*.1
        fresh(nav, now, x=.04*i)
        nav.step(obs('converging'), path(), now, t)
    assert nav.phase == 'PASS_EXIT'  # still seeing the same exit: do not rearm


def test_start_delay_and_single_frame_fork_do_not_arm_entry():
    nav, t = PatrolRingNavigator(), replace(PatrolTuning(), ring_start_delay_s=5.)
    for i in range(50):
        nav.step(obs('converging'), path(), 1+i*.04, t)
    assert nav.phase == 'SEEK_EXIT' and not nav.armed_notice
    nav.step(obs('converging'), path(), 7., t)
    nav.step(obs(), path(), 7.04, t)
    assert nav.phase == 'SEEK_EXIT'


def test_entry_locks_curve_and_does_not_use_normal_right_line():
    nav, t, now = enter_ring()
    fresh(nav, now+.05, x=.305, rate=.6)
    decision = nav.step(obs('diverging'), path(.6), now+.05, t)
    assert decision.path.angle_rad < 0 and decision.speed == t.ring_entry_speed_m_s
    assert not decision.stop


def test_short_dash_loss_moves_slowly_left_then_stops_not_spins_forever():
    nav, t, now = enter_ring()
    fresh(nav, now+.05, x=.305, rate=.3)
    decision = nav.step(obs(dashed=False), path(.6), now+.05, t)
    assert decision.search_turn > 0 and decision.speed == .1
    assert not decision.path.valid
    for i in range(1, 10):
        instant = now+.05+i*.1
        fresh(nav, instant, x=.305+i*.005, rate=.3)
        decision = nav.step(obs(dashed=False), path(.6), instant, t)
    assert decision.stop and nav.phase == 'FAILED'
    assert nav.step(obs(), path(), instant+.01, t).stop  # no automatic restart


def test_signed_gyro_not_absolute_turn_and_yaw_does_not_depend_on_cmd():
    nav, t, now = enter_ring()
    for i in range(1, 11):
        nav.observe_imu(.8, now+i*.05)
    positive = nav.turn_rad
    for i in range(11, 21):
        nav.observe_imu(-.8, now+i*.05)
    assert 0 < positive < .5
    assert abs(nav.turn_rad) < .08
    assert nav.travel_m == 0.


def test_stationary_360_degree_rotation_cannot_complete():
    nav, t, now = enter_ring()
    nav.phase = 'FOLLOW'
    # Continuous sensor samples, same odom pose; visual solid alone is not enough.
    for i in range(1, 141):
        instant = now+i*.05
        fresh(nav, instant, x=.3, rate=.9)
        decision = nav.step(obs(solid=True), path(), instant, t)
    assert nav.turn_rad > math.radians(330)
    assert nav.travel_m == 0 and nav.phase == 'FOLLOW' and not decision.stop


def test_exit_needs_turn_distance_time_and_continuous_solid_then_is_latched():
    nav, t, now = enter_ring()
    nav.phase = 'FOLLOW'
    # A moving circular trajectory with signed CCW gyro.
    for i in range(1, 141):
        instant = now+i*.05
        angle = .9*i*.05
        fresh(nav, instant, .3+.3*math.sin(angle), .3*(1-math.cos(angle)), .9)
        nav.step(obs(), path(.5), instant, t)
    assert nav.travel_m > 1.2 and nav.turn_rad > math.radians(330)
    nav.step(obs(solid=True), path(), instant+.01, t)
    nav.step(obs(solid=False), path(), instant+.02, t)
    assert nav.phase == 'FOLLOW'
    for i in range(1, 9):
        at = instant+.02+i*.01
        fresh(nav, at, .3+.3*math.sin(angle), .3*(1-math.cos(angle)), 0.)
        nav.step(obs(solid=True), path(.4), at, t)
    assert nav.phase == 'DONE'
    decision = nav.step(obs('diverging'), path(.1), at+.01, t)
    assert decision.phase == 'DONE' and decision.path.angle_rad == .1


def test_missing_sensors_odom_jump_and_overrotation_fail_closed():
    nav, t, now = enter_ring()
    assert nav.step(obs(), path(), now+1., t).stop
    nav, t, now = enter_ring()
    fresh(nav, now+.05, 5., 5.)
    assert nav.phase == 'FAILED'
    nav, t, now = enter_ring()
    nav.turn_rad = math.radians(421)
    assert nav.step(obs(), path(), now+.01, t).stop


def test_disable_during_ring_stops_but_disable_before_start_preserves_normal():
    disabled = tuning(ring_enabled=0)
    nav = PatrolRingNavigator()
    assert nav.step(obs(), path(.1), 1., disabled).path.angle_rad == .1
    nav, _, now = enter_ring()
    assert nav.step(obs(), path(), now+.01, disabled).stop


def test_entry_not_confirmed_by_stationary_rotation_alone():
    nav, t, now = enter_ring()
    for i in range(1, 21):
        instant = now+i*.05
        fresh(nav, instant, .3, 0., .8)
        nav.step(obs('diverging'), path(), instant, t)
    assert nav.turn_rad > math.radians(8) and nav.travel_m == 0
    assert nav.phase == 'ENTER'


def test_real_translation_and_turn_confirm_inside_ring():
    nav, t, now = enter_ring()
    for i in range(1, 16):
        instant = now+i*.05
        fresh(nav, instant, .3+i*.006, 0., .5)
        nav.step(obs('diverging'), path(), instant, t)
    assert nav.phase == 'FOLLOW'


def test_ring_loss_timeout_adapts_to_search_speed():
    nav, t, now = enter_ring()
    slow = replace(t, ring_loss_speed_m_s=.05)
    nav.phase = 'FOLLOW'
    for i in range(1, 11):
        instant = now+i*.1
        fresh(nav, instant, .3+i*.003, 0., .1)
        decision = nav.step(obs(dashed=False), path(), instant, slow)
    assert not decision.stop and decision.search_turn > 0  # 1.4s instead of .7s at half search speed


def test_ring_target_offset_is_shared_with_normal_patrol():
    centered = observe_ring(fork_mask(), PatrolTuning())
    offset = observe_ring(fork_mask(), replace(PatrolTuning(), target_x_ratio=.55))
    assert offset.main.angle_rad < centered.main.angle_rad
    assert offset.branch.angle_rad < centered.branch.angle_rad


@pytest.mark.parametrize('seed', [.25, .35, .5, .75])
@pytest.mark.parametrize('kind', ['converging', 'diverging'])
def test_main_axis_does_not_follow_ordinary_tracker_into_wrong_branch(seed, kind):
    observation = observe_ring(fork_mask(kind), PatrolTuning(), seed_x_ratio=seed)
    assert observation.kind == kind
    assert np.median(observation.main.points[:, 0]) == pytest.approx(175.)


@pytest.mark.parametrize('kind', ['converging', 'diverging'])
def test_unconfirmed_fork_is_diagnostic_only_no_target_or_speed_override(kind):
    nav, t = PatrolRingNavigator(), tuning()
    normal = path(-.4)
    for index in range(t.ring_fork_confirm_frames-1):
        decision = nav.step(obs(kind), normal, 1+index*.04, t)
        assert decision.path is normal
        assert decision.speed == -1 and decision.phase == 'SEEK_EXIT'
    if kind == 'diverging':
        for index in range(10):
            decision = nav.step(obs(kind), normal, 2+index*.04, t)
            assert decision.path is normal and decision.speed == -1


def test_flapping_fork_candidates_never_take_control():
    nav, t, normal = PatrolRingNavigator(), tuning(), path(.25)
    for index, kind in enumerate(['converging', 'none', 'diverging', 'none']*10):
        decision = nav.step(obs(kind), normal, 1+index*.04, t)
        assert decision.path is normal and decision.speed == -1


def test_invalid_branch_evidence_cannot_confirm_exit():
    nav, t, normal = PatrolRingNavigator(), tuning(), path(.25)
    observation = obs('converging')
    observation.branch.valid = False
    for index in range(10):
        decision = nav.step(observation, normal, 1+index*.04, t)
        assert decision.path is normal and decision.speed == -1


@pytest.mark.parametrize('changes', [
    {'ring_enabled': 2}, {'ring_speed_m_s': 0}, {'ring_start_delay_s': -1},
    {'ring_fork_confirm_frames': 1}, {'ring_fork_confirm_frames': 2.5},
    {'ring_min_turn_deg': 360}, {'ring_max_turn_deg': 300},
    {'ring_min_duration_s': 50}, {'ring_roi_top_ratio': .8},
    {'ring_loss_timeout_s': 0}, {'ring_search_turn_rad_s': 2},
])
def test_ring_parameters_validate(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(), **changes).validate()
