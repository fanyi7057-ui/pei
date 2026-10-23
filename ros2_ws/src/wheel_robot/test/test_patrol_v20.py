"""V20 distance-bounded exit, owned-current-block recovery, SBUS ownership."""
from dataclasses import replace
from types import SimpleNamespace
import math
import cv2
import numpy as np
import pytest
from test_patrol_contour import make_node,node_module,load_actual_vision_module
from test_patrol_v17 import gyro_ring
from test_patrol_vv_v16 import masks
from patrol_contour_runtime import PatrolTuning,RoundaboutRouter,RemoteAuthorityGuard,PathEstimate
from patrol_dash_entry import track_vv_curve


def seek():
    r,t,now=gyro_ring()
    r.loop_seek_pose=(0.,0.,0.)
    return r,t,now


def test_exit_search_adds_ten_cm_without_changing_any_speed():
    r,t,now=seek()
    assert t.ring_gyro_exit_seek_distance_m==.83  # V22 approved distance extension
    assert (t.fixed_speed_m_s,t.roundabout_speed_m_s,t.ring_recovery_speed_m_s)==(.65,.30,.12)
    r.measured_turn.stamp=now+1.5
    assert r.motion_override(now+1.5,t,(.18,0.,0.),PathEstimate(False))==(.12,0.,"gyro_exit_seek")
    r.measured_turn.stamp=now+1.9
    assert r.motion_override(now+1.9,t,(.831,0.,0.),PathEstimate(False))==(0.,0.,"ring_recovery_stop")
    assert r.fault=="gyro_exit_seek_distance_no_reliable_path" and r.completed==0


@pytest.mark.parametrize("pose",[(0.,.3,0.),(-.1,0.,0.),None])
def test_sideways_reverse_or_no_odom_never_claim_forward_distance(pose):
    r,t,now=seek()
    assert r.motion_override(now,t,pose,PathEstimate(False))[2]=="gyro_exit_seek"
    assert r.loop_seek_distance==0


def test_pose_projection_follows_actual_start_heading():
    r,t,now=seek();r.loop_seek_pose=(1.,2.,math.pi/2)
    assert r.motion_override(now,t,(1.,2.84,math.pi/2),PathEstimate(False))[2]=="ring_recovery_stop"


def test_stalled_search_still_has_hard_time_limit_with_fresh_imu():
    r,t,now=seek();r.measured_turn.stamp=now+8.01
    assert r.motion_override(now+8.01,t,(0.,0.,0.),PathEstimate(False))[2]=="ring_recovery_stop"
    assert r.fault=="gyro_exit_seek_no_reliable_path"


def test_heading_departure_cannot_extend_search():
    r,t,now=seek();r.measured_turn.last_yaw=r.loop_seek_yaw+math.radians(40)
    assert r.motion_override(now,t,(.05,0.,0.),PathEstimate(False))[2]=="ring_recovery_stop"
    assert r.fault=="gyro_exit_seek_heading_departure"


def test_real_route_can_take_over_before_distance_stop_but_not_fake_receipt():
    r,t,now=seek()
    assert r.motion_override(now,t,(.23,0.,0.),PathEstimate(True,route_observed=True)) is None
    assert not any(r.course.rings)


def test_follow_only_reacquisition_uses_current_shifted_blocks():
    dark,_=masks();dark[:,170:181]=0
    dark=cv2.warpAffine(dark,np.float32([[1,0,-55],[0,1,0]]),(320,240))
    white=255-dark
    reference=np.array([[157.,210.],[145.,185.],[133.,160.],[121.,135.]])
    assert not track_vv_curve(dark,white,PatrolTuning(),reference).valid
    result=track_vv_curve(dark,white,PatrolTuning(),reference,reacquire=True)
    assert result.valid and result.reason=="vv_owned_curve_reacquired" and result.route_locked
    empty=np.zeros_like(dark)
    assert not track_vv_curve(empty,255-empty,PatrolTuning(),reference,reacquire=True).valid


def test_reacquire_cannot_use_single_blob_or_unrelated_straight_chain():
    t=PatrolTuning();ref=np.array([[90.,100.],[115.,150.],[145.,210.]])
    one=np.zeros((240,320),np.uint8);cv2.rectangle(one,(85,180),(94,190),255,-1)
    assert not track_vv_curve(one,255-one,t,ref,reacquire=True).valid
    straight=np.zeros_like(one)
    for y in (110,150,190,210):cv2.rectangle(straight,(50,y-4),(58,y+4),255,-1)
    assert not track_vv_curve(straight,255-straight,t,ref,reacquire=True).valid


def test_two_equally_supported_shifted_routes_are_not_guessed():
    ref=np.array([[157.,210.],[145.,185.],[133.,160.],[121.,135.]])
    dark=np.zeros((240,320),np.uint8)
    for shift in (-55,55):
        for x,y in ref:cv2.rectangle(dark,(int(x+shift-4),int(y-5)),(int(x+shift+4),int(y+5)),255,-1)
    result=track_vv_curve(dark,255-dark,PatrolTuning(),ref,reacquire=True)
    assert not result.valid and result.reason=="vv_curve_reacquire_ambiguous"


def test_actual_vision_expanded_recovery_never_overrides_healthy_center_line(monkeypatch):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision();t=PatrolTuning()
    v.router._phase("FOLLOW",0.);v.router.branch_acquired=v.router.ring_route_acquired=True
    v.ring_phase="FOLLOW";v.previous_points=np.array([[160.,100.],[160.,220.]]);v.last_path_at=.1
    fallback=PathEstimate(True,-.5,.65,points=np.array([[80.,100.],[90.,220.]]),
                          route_locked=True,reason="vv_owned_curve_reacquired")
    monkeypatch.setattr(m,"track_vv_curve",lambda *a,**kw:fallback)
    frame=np.full((240,320,3),255,np.uint8);cv2.line(frame,(160,60),(160,239),(0,0,0),8)
    result=v.estimate(frame,t,.2)
    assert result.valid and result.reason!="vv_owned_curve_reacquired" and abs(result.error)<.15


def channels(raw):
    result=[992]*10;result[4]=raw;return result


def test_switch_uses_confirmed_firmware_mapping_and_timeout():
    g,t=RemoteAuthorityGuard(),PatrolTuning()
    assert g.state(0,t)=="unknown"
    g.update(channels(992),0.,t);assert g.state(0,t)=="manual_switch"
    g.update(channels(1792),.1,t);assert g.state(.1,t)=="auto_switch"
    assert g.state(.5,t)=="stale"
    g.update([1,2],.6,t);assert g.state(.6,t)=="unknown"


def test_manual_takeover_invalidates_only_unfinished_ring_and_never_restarts(node_module,monkeypatch):
    n,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter();r._phase("FOLLOW",9.);r.branch_acquired=r.ring_route_acquired=True
    n.vision.router=r;n.vision.reset_path=lambda:None
    n.remote_guard=RemoteAuthorityGuard();n.remote_guard.update(channels(992),10.,n.patrol_tuning)
    n._run_control_cycle()
    assert r.phase=="SEEK" and not r.ring_route_acquired and r.measured_turn.angle==0
    assert n._localization_wait and not any(r.course.rings) and commands[-1]==(0.,0.)
    n._on_race_start_confirm(SimpleNamespace(data=True));assert n._localization_wait
    n.remote_guard.update(channels(1792),10.,n.patrol_tuning)
    assert n._localization_wait  # switch alone never starts ROS motion
    n._on_race_start_confirm(SimpleNamespace(data=True));assert not n._localization_wait


def test_remote_takeover_preserves_already_completed_lap(node_module,monkeypatch):
    n,_=make_node(node_module,monkeypatch)
    r=RoundaboutRouter();r.course.confirm_ring();r.course.confirm_zebra();r.course.next_lap();r._phase("ENTRY",9.)
    n.vision.router=r;n.vision.reset_path=lambda:None
    n.remote_guard=RemoteAuthorityGuard();n.remote_guard.update(channels(992),10.,n.patrol_tuning)
    n._run_control_cycle()
    assert r.course.rings==[True,False] and r.course.zebras==[True,False] and r.course.lap==1


def test_manual_switch_stops_even_without_new_camera_frame(node_module,monkeypatch):
    n,commands=make_node(node_module,monkeypatch);n.vision.reset_path=lambda:None
    n._last_processed_sequence=n._frame_sequence
    n.remote_guard=RemoteAuthorityGuard();n.remote_guard.update(channels(992),10.,n.patrol_tuning)
    n._run_control_cycle()
    assert commands[-1]==(0.,0.) and n._localization_wait


def test_stale_auto_switch_cannot_start_or_continue(node_module,monkeypatch):
    n,commands=make_node(node_module,monkeypatch);n.vision.reset_path=lambda:None
    n.remote_guard=RemoteAuthorityGuard();n.remote_guard.update(channels(1792),9.,n.patrol_tuning)
    n._run_control_cycle()
    assert commands[-1]==(0.,0.) and n._localization_wait
    n._on_race_start_confirm(SimpleNamespace(data=True));assert n._localization_wait


def test_dry_run_does_not_require_remote_motion_authority(node_module,monkeypatch):
    n,commands=make_node(node_module,monkeypatch);n.remote_guard=RemoteAuthorityGuard()
    monkeypatch.setattr(node_module.base,"ENABLE_MOTION",False)
    n._run_control_cycle()
    assert not getattr(n,"_localization_wait",False) and commands[-1]==(0.,0.)


@pytest.mark.parametrize("changes",[{"ring_gyro_exit_seek_distance_m":0},{"ring_gyro_exit_seek_distance_m":1.1},
    {"remote_guard_enabled":2},{"remote_auto_channel_index":10},{"remote_auto_channel_min":99},
    {"remote_signal_timeout_seconds":0},{"ring_curve_reacquire_enabled":2},
    {"ring_curve_reacquire_max_shift_ratio":.3}])
def test_new_parameters_reject_unsafe_values(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()
