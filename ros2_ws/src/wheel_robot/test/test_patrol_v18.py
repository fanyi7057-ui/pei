"""V18 current-pixel preview and non-authorizing candidate slowdown."""
from dataclasses import replace
import numpy as np
import pytest
from test_patrol_contour import make_node, node_module
from patrol_contour_runtime import PatrolTuning, PathEstimate
from patrol_dash_entry import DashObservation, detect_vv_dashes, track_vv_curve
from test_patrol_vv_v16 import armed, masks


def test_candidate_is_short_lived_and_never_authorizes_entry():
    r,t=armed()
    r.observe(False,0.,0.,t,(.65,0.))
    r.observe(True,1.,.1,t,(.65,0.),dash_cue=DashObservation(valid=True,blocks=4))
    assert r.candidate_active(.2,t) and r.phase=="SEEK"
    assert not r.candidate_active(.8,t)
    assert r.first_marker_at is None and not any(r.course.rings)
    r.pause(.2)
    assert not r.candidate_active(.3,t)


@pytest.mark.parametrize("error,sign",[(-.6,1),(.6,-1),(0.,0)])
def test_current_owned_pixels_steer_before_full_proof(error,sign):
    r,t=armed();r._phase("ENTRY",0.)
    p=PathEstimate(True,error,.8,route_locked=True,route_observed=False)
    v,w,mode=r.motion_override(.2,t,path=p)
    assert v==.12 and abs(w)<=.6 and mode=="ring_curve_preview"
    assert np.sign(w)==sign
    r.accept_path(p,.2,t)
    assert r.phase=="ENTRY" and r.path_streak==0


def test_no_pixels_still_bounded_acquisition_not_visual_preview():
    r,t=armed();r._phase("ENTRY",0.)
    assert r.motion_override(.2,t,path=PathEstimate(False))[2]=="roundabout_left_acquire"


def test_detection_diagnostics_explain_filter_stage():
    dark,white=masks()
    hit=detect_vv_dashes(dark,white,PatrolTuning())
    assert hit.raw_blocks>=hit.white_blocks>=hit.blocks>=4
    assert hit.column_blocks>=4 and hit.branch_blocks>=2
    rejected=detect_vv_dashes(dark,np.zeros_like(white),PatrolTuning())
    assert rejected.raw_blocks>0 and rejected.white_blocks==0 and not rejected.valid


@pytest.mark.parametrize("field,value",[("ring_candidate_hold_seconds",0),("ring_candidate_turn_limit_rad_s",3)])
def test_bad_candidate_parameters_rejected(field,value):
    with pytest.raises(ValueError):replace(PatrolTuning(),**{field:value}).validate()


def test_current_curve_near_threshold_matches_entry_not_unrelated_72_percent():
    dark,_=masks();dark[:,170:181]=0
    points=np.array([[157.,210.],[145.,185.],[133.,160.],[121.,135.]])
    guide=track_vv_curve(dark,255-dark,PatrolTuning(),points)
    assert guide.valid and guide.route_observed and guide.route_locked


def test_actual_candidate_publish_caps_previous_fast_right_search(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r,t=armed();r.started_at=0.;r.vv_candidate_at=9.9
    node.vision.router=r
    node.vision.estimate=lambda *_a,**_k:PathEstimate(False)
    node.patrol_controller.last_command=(.65,-2.)
    node.patrol_controller.last_time=9.97
    node._run_control_cycle()
    assert commands[-1][0]<=.30 and abs(commands[-1][1])<=.6
    assert node.patrol_controller.last_command==commands[-1]


def test_actual_preview_publish_caps_stale_speed_and_keeps_right_correction(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r,t=armed();r._phase("ENTRY",9.9);node.vision.router=r
    node.vision.estimate=lambda *_a,**_k:PathEstimate(True,.5,.8,route_locked=True)
    node.patrol_controller.last_command=(.65,0.)
    node.patrol_controller.last_time=9.97
    node._run_control_cycle()
    assert commands[-1][0]<=.12 and commands[-1][1]<0
