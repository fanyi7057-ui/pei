"""V17 regressions: log 929a5007, bounded acquisition not angle-only handoff."""
from dataclasses import replace
import math
import cv2
import numpy as np
import pytest

from test_patrol_contour import load_actual_vision_module,make_node,node_module
from test_patrol_vv_v16 import masks,cue,armed
from patrol_contour_runtime import PatrolTuning,RoundaboutRouter,PathEstimate,MeasuredRingTurn
from patrol_dash_entry import DashObservation,track_vv_curve


def reliable():
    return PathEstimate(True,-.1,.8,route_locked=True,route_observed=True)


def test_short_pass_defaults_and_no_generic_dash_five_marker():
    r,t=armed()
    assert t.roundabout_first_marker_cooldown_seconds==.5
    assert t.roundabout_first_marker_pass_distance_m==.12
    for i in range(20):r.observe(True,1.,i*.03,t,(.3,0.),dash_cue=DashObservation(valid=True,blocks=5))
    assert r.phase=="SEEK" and r.first_marker_at is None


def test_explicit_four_is_direct_entry_not_an_exit_to_ignore():
    r,t=armed()
    r.observe(False,0.,0.,t,(.3,0.))
    r.observe(True,1.,.04,t,(.3,0.),dash_cue=cue(t=t))
    assert r.phase=="ENTRY" and not r.course.rings[0]


def test_five_pass_is_about_fifteen_cm_not_sixtyfive_cm():
    r,t=armed()
    for i in range(8):r.observe(True,1.,i*.1,t,(.3,0.),(i*.03,0.,0.),dash_cue=cue("converging",t))
    assert .12<=r.distance-r.first_marker_distance<=.21
    r.observe(True,1.,.8,t,(.3,0.),(.24,0.,0.),dash_cue=cue(t=t))
    assert r.phase=="ENTRY"


@pytest.mark.parametrize("degrees",[54,56,101,164])
def test_angle_without_current_route_never_hands_back_to_full_speed(degrees):
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r.measured_turn.source="imu";r.measured_turn.stamp=1.;r.measured_turn.angle=math.radians(degrees)
    v,w,mode=r.motion_override(1.,t,path=PathEstimate(False))
    assert 0<v<=.3 and 0<w<=1.2 and mode=="roundabout_left_acquire"
    if degrees>=55:assert v<=.12 and w<=.35


def test_real_owned_curve_gets_visual_control_before_fiftyfive_degrees():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r.left_yaw=.45
    assert r.motion_override(.7,t,path=reliable()) is None


def test_curved_blocks_continue_without_straight_fork():
    dark,_=masks()
    dark[:,170:181]=0  # only curved branch remains, no main four-block column
    points=np.array([[157.,210.],[145.,185.],[133.,160.],[121.,135.]])
    guide=track_vv_curve(dark,np.where(dark==0,255,0).astype(np.uint8),PatrolTuning(),points)
    assert guide.valid and guide.route_locked and guide.route_observed
    assert guide.reason=="vv_owned_curve"
    empty=np.zeros_like(dark)
    assert not track_vv_curve(empty,255-empty,PatrolTuning(),points).valid


def test_actual_vision_continues_current_entry_curve(monkeypatch):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision();t=PatrolTuning()
    v.router._phase("ENTRY",0.);v.ring_phase="ENTRY"
    v.previous_points=np.array([[157.,210.],[145.,185.],[133.,160.],[121.,135.]])
    v.last_path_at=.1
    dark,_=masks();dark[:,170:181]=0
    result=v.estimate(cv2.cvtColor(255-dark,cv2.COLOR_GRAY2BGR),t,.2)
    assert result.valid and result.reason=="vv_owned_curve" and result.route_observed


def test_failure_recovery_cannot_keep_high_speed_or_persist_forever():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r._release_ring(3.,t,"entry_no_route_abort")
    assert r.motion_override(3.1,t,path=PathEstimate(False))==(.12,.35,"ring_failure_recovery")
    assert r.motion_override(4.1,t,path=PathEstimate(False))==(0.,0.,"ring_recovery_stop")
    assert r.fault and not any(r.course.rings)


def test_entry_without_imu_also_has_bounded_route_acquisition():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r.observe(False,0.,3.1,t,(.12,.35))
    assert r.phase=="SEEK" and r.ring_recovery_until is not None


def test_failure_recovery_requires_consecutive_real_route_to_resume():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r._release_ring(3.,t,"entry_no_route_abort")
    for now in (3.1,3.2):assert r.motion_override(now,t,path=reliable())[0]<=.12
    assert r.motion_override(3.3,t,path=reliable()) is None
    assert r.ring_recovery_until is None


def test_repeated_recovery_timestamp_does_not_prefill_confirmations():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r._release_ring(3.,t,"entry_no_route_abort")
    for _ in range(10):r.motion_override(3.1,t,path=reliable())
    assert r.ring_recovery_streak==1 and r.ring_recovery_until is not None


def test_explicit_operator_resume_does_not_retain_expired_recovery_deadline():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r._release_ring(3.,t,"entry_no_route_abort")
    r.motion_override(4.1,t,path=PathEstimate(False));r.pause(4.2)
    r.resume(10.,clear_fault=True)
    assert not r.fault and r.ring_recovery_until is None


def test_actual_failure_output_does_not_leak_previous_point65(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter();r._phase("ENTRY",8.);r._release_ring(9.9,node.patrol_tuning,"entry_no_route_abort")
    node.vision.router=r
    node.vision.estimate=lambda *_a,**_k:PathEstimate(False)
    node.patrol_controller.last_command=(.65,-1.8)
    node.patrol_controller.last_time=9.97
    node._run_control_cycle()
    assert commands[-1][0]<=.12
    assert node.patrol_controller.last_command==commands[-1]


def gyro_ring(degrees=370,travel=.03,enabled=1):
    r,t=RoundaboutRouter(),replace(PatrolTuning(),ring_gyro_exit_enabled=enabled)
    r._phase("FOLLOW",0.);r.branch_acquired=r.ring_route_acquired=True
    for i,deg in enumerate(range(0,degrees+1,10)):
        now=i*.1;yaw=(math.radians(deg)+math.pi)%(2*math.pi)-math.pi
        r.observe(False,0.,now,t,(.3,1.),(i*travel,0.,yaw),heading=("imu",yaw,now))
    return r,t,now


@pytest.mark.parametrize("degrees,travel,enabled,want",[(350,.03,1,False),(370,0.,1,False),(370,.03,0,False),(370,.03,1,True)])
def test_gyro_full_turn_and_actual_distance_only_nominate_exit(degrees,travel,enabled,want):
    r,t,now=gyro_ring(degrees,travel,enabled)
    assert (r.phase=="EXIT")==want
    assert r.completed==0 and not any(r.course.rings)
    if want:
        assert r.motion_override(now,t,path=PathEstimate(False))==(.12,0.,"gyro_exit_seek")
        assert r.motion_override(now,t,path=reliable()) is None


def test_gyro_exit_seek_cannot_expire_into_fast_blind_search():
    r,t,now=gyro_ring()
    assert r.phase=="EXIT"
    assert r.motion_override(now+1.1,t,path=PathEstimate(False))==(0.,0.,"ring_recovery_stop")
    assert r.completed==0 and r.fault


@pytest.mark.parametrize("changes",[{"roundabout_speed_m_s":0},{"ring_gyro_exit_enabled":2},
    {"ring_gyro_exit_degrees":100},{"ring_gyro_min_distance_m":0},{"ring_gyro_exit_seek_seconds":4},
    {"ring_recovery_speed_m_s":.5},{"ring_recovery_turn_rad_s":0},{"ring_recovery_seconds":0}])
def test_bad_ring_parameters_reject_save(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()
