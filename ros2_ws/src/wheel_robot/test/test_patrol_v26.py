"""V26 synthetic masks, bounded grace, posture and read-only HTTP preview."""
from dataclasses import replace
import http.client
import json
import time
import cv2
import numpy as np
import pytest
from test_patrol_contour import cruise_controller,load_actual_vision_module,make_node,node_module
from patrol_contour_runtime import PatrolTuning,PathEstimate,FixedSpeedController,PathSelector
from patrol_stability import stabilize_masks
from patrol_preview import FirstPersonPreview


def test_posture_and_baseline_speeds():
    t=PatrolTuning();t.validate()
    assert (t.posture_height_m,t.posture_pitch_deg)==(.25,28.)
    assert (t.fixed_speed_m_s,t.roundabout_speed_m_s)==(.65,.30)
    assert t.roundabout_first_marker_pass_distance_m==.12

def test_dark_blue_removed_without_removing_neutral_black():
    bgr=np.full((120,160,3),240,np.uint8)
    bgr[40:100,20:35]=(11,3,1)  # below original V=12 blue floor
    bgr[40:100,75:85]=(8,8,8)
    mask=np.zeros((120,160),np.uint8);mask[40:100,20:35]=255;mask[40:100,75:85]=255
    a,e=stabilize_masks(mask,mask,cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV),PatrolTuning())
    assert not a[50,25] and not e[50,25]
    assert a[50,80] and e[50,80] and mask[50,25]  # does not mutate input

def test_wide_black_blob_not_tracking_target_but_zebra_event_retained():
    mask=np.zeros((120,160),np.uint8);mask[40:90,20:140]=255
    hsv=np.zeros((120,160,3),np.uint8)
    a,e=stabilize_masks(mask,mask,hsv,PatrolTuning())
    assert not a.any() and np.array_equal(e,mask)

def test_neutral_curved_thin_and_dashed_lines_survive():
    m=np.zeros((240,320),np.uint8)
    cv2.ellipse(m,(160,200),(70,150),0,180,300,255,7)
    for y in range(80,220,30):cv2.rectangle(m,(200,y),(208,y+10),255,-1)
    a,e=stabilize_masks(m,m,np.zeros((240,320,3),np.uint8),PatrolTuning())
    assert np.array_equal(a,m) and np.array_equal(e,m)

def test_nearly_horizontal_real_curve_not_erased_for_width_alone():
    m=np.zeros((120,160),np.uint8);m[65:72,20:140]=255
    a,_=stabilize_masks(m,m,np.zeros((120,160,3),np.uint8),PatrolTuning())
    assert np.array_equal(a,m)

def test_short_loss_keeps_issued_speed_and_turn_then_caps():
    c,t=cruise_controller(-.1,maintain_cruise_during_search=0)
    before=c.last_command
    v,w,mode=c.command(PathEstimate(False),2.03,t)
    assert v==pytest.approx(before[0]) and w==pytest.approx(before[1]) and mode=='short_loss_bridge'
    assert c.command(PathEstimate(False),2.13,t)[0]<=.2

def test_short_valid_reacquisition_does_not_pulse_brake():
    c,t=cruise_controller(-.1,maintain_cruise_during_search=0)
    c.command(PathEstimate(False),2.03,t)
    for now in [2.06,2.09,2.12]:
        assert c.command(PathEstimate(True,-.1,.9,source='black'),now,t)[0]==pytest.approx(.65)

@pytest.mark.parametrize('kind',['weak','rescue','handoff'])
def test_unreliable_tracking_never_gets_blind_cruise_grace(kind):
    c=FixedSpeedController();t=PatrolTuning()
    p=PathEstimate(True,-.1,.2 if kind=='weak' else .9,rescue=kind=='rescue',handoff=kind=='handoff')
    for i in range(40):c.command(p,i*.05,t)
    assert c.command(PathEstimate(False),1.98,t)[0]<=.2

def test_distance_budget_and_explicit_ring_search_override_grace():
    c,t=cruise_controller(-.1,maintain_cruise_during_search=0)
    assert c.command(PathEstimate(False),2.03,t,search_direction=1)[0]<=.2
    c,t=cruise_controller(-.1,maintain_cruise_during_search=0,loss_grace_distance_m=.01)
    assert c.command(PathEstimate(False),2.03,t)[0]<=.2

@pytest.mark.parametrize('changes',[{'loss_grace_seconds':.2},{'loss_grace_seconds':float('nan')},
    {'loss_grace_distance_m':0},{'dark_blue_s_min':1},{'dark_blue_br_min':100},
    {'black_max_run_ratio':.8},{'dark_blue_guard_enabled':2}])
def test_new_invalid_parameters_rejected(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()

def test_preview_loopback_read_only_and_closes():
    stream=FirstPersonPreview(lambda:(np.zeros((48,64,3),np.uint8),time.monotonic()),port=0)
    try:
        assert stream.server.server_address[0]=='127.0.0.1'
        c=http.client.HTTPConnection('127.0.0.1',stream.port,timeout=2)
        c.request('GET','/status');r=c.getresponse();assert r.status==200
        assert not json.loads(r.read())['fresh'];c.close()
        c=http.client.HTTPConnection('127.0.0.1',stream.port,timeout=2)
        c.request('GET','/stream');r=c.getresponse();assert r.status==200
        assert b'--frame' in r.read(40);c.close()
        c=http.client.HTTPConnection('127.0.0.1',stream.port,timeout=2)
        c.request('GET','/cmd_vel');r=c.getresponse();assert r.status==404;r.read();c.close()
    finally:stream.close()
    assert not stream.encoder.is_alive() and not stream.http_thread.is_alive()

def test_guard_can_be_disabled_without_replacing_current_pixels():
    m=np.full((40,80),255,np.uint8)
    a,e=stabilize_masks(m,m,np.zeros((40,80,3),np.uint8),replace(PatrolTuning(),mask_stability_enabled=0))
    assert np.array_equal(a,m) and np.array_equal(e,m)

def test_single_edge_road_cannot_jump_to_opposite_side_in_one_frame():
    s=PathSelector();t=PatrolTuning()
    assert s.select(PathEstimate(True,-.6,.9),PathEstimate(False),0.,t)[0].valid
    road=PathEstimate(True,.6,.5,single_edge=True,source='road')
    p,_=s.select(PathEstimate(False),road,.03,t)
    assert not p.valid and p.reason=='path_jump_confirm'
    assert s.select(PathEstimate(False),road,.06,t)[0].valid

def test_optional_mask_guard_failure_falls_back_to_current_base_masks(monkeypatch):
    module=load_actual_vision_module(monkeypatch);v=module.PatrolVision()
    def broken(*args):raise ValueError('synthetic guard failure')
    monkeypatch.setattr(module,'stabilize_masks',broken)
    image=np.full((240,320,3),240,np.uint8);image[:,155:164]=0
    v.estimate(image,PatrolTuning(),0.)
    assert v.mask_guard_fallback and v.black_estimate.mask.any()

def test_grace_can_be_disabled_and_reset_clears_old_evidence():
    c,t=cruise_controller(-.1,maintain_cruise_during_search=0,loss_grace_seconds=0)
    assert c.command(PathEstimate(False),2.03,t)[0]<=.2
    c.reset();assert c.last_reliable_at is None and not c.grace_active

def test_stale_preview_reports_not_fresh(monkeypatch):
    s=FirstPersonPreview(lambda:(None,0.),port=0)
    try:
        with s.condition:s.jpeg=b'old';s.stamp=time.monotonic()-2.
        c=http.client.HTTPConnection('127.0.0.1',s.port,timeout=2)
        c.request('GET','/status');r=c.getresponse();assert not json.loads(r.read())['fresh'];c.close()
    finally:s.close()

def test_requested_posture_reaches_existing_transition_interface(node_module,monkeypatch):
    from types import SimpleNamespace
    from race_posture import RacePosture
    monkeypatch.setattr(node_module.base,'RacePosture',RacePosture,raising=False)
    node,_=make_node(node_module,monkeypatch)
    targets=[]
    node._posture_transition=SimpleNamespace(set_target=lambda pose,stamp:targets.append(pose))
    node._set_patrol_posture()
    assert targets[-1].height_m==.25 and targets[-1].pitch_deg==28.
