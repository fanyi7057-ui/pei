from types import SimpleNamespace
from dataclasses import replace
from patrol_contour_runtime import PatrolTuning, RoundaboutRouter, PathEstimate

def left_cue():
    return SimpleNamespace(valid=True, branch=PathEstimate(True))


def fixture():
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    r=RoundaboutRouter();r.started_at=0.
    r._observe_vv_entry(SimpleNamespace(valid=True),.1,t,.03,.01,(.3,0.))
    return r,t

def test_one_frame_latches_advance_without_second_cue():
    r,t=fixture()
    assert r.phase=="APPROACH"
    r.distance=.119
    r._observe_vv_entry(left_cue(),.4,t,.03,.01,(.3,0.))
    assert r.phase=="APPROACH" and r.motion_override(.4,t) is None
    r.distance=.121
    r._observe_vv_entry(None,.5,t,.03,.002,(.3,0.))
    assert r.phase=="ENTRY"
    assert r.motion_override(.51,t)[1]>0

def test_probe_timeout_does_not_award_ring_or_continue_forced_turn():
    r,t=fixture();r.distance=.121
    r._observe_vv_entry(left_cue(),.5,t,.03,.01,(.3,0.))
    assert r.motion_override(.81,t) == (0., 0., "roundabout_entry_release")
    assert r.phase=="SEEK" and not any(r.course.rings)
    assert r.ring_release_source=="entry_probe_no_route"

def test_probe_hands_to_real_owned_route_before_timeout():
    r,t=fixture();r.distance=.121
    r._observe_vv_entry(left_cue(),.5,t,.03,.01,(.3,0.))
    p=PathEstimate(True,route_locked=True,route_observed=True)
    assert r.motion_override(.56,t,path=p)[2] == "roundabout_left_acquire"
    r.entry_dash_proven = True
    assert r.motion_override(.60,t,path=p)[2] == "roundabout_left_acquire"
    assert r.motion_override(.64,t,path=p)[2] == "roundabout_left_acquire"
    assert r.motion_override(.68,t,path=p) is None
    assert r.motion_override(.70,t,path=PathEstimate(False))[2] == "roundabout_left_acquire"
    assert r.motion_override(.74,t,path=p) is None
    assert r.motion_override(.80,t,path=PathEstimate(False))[2] == "roundabout_left_acquire"
    assert r.motion_override(1.10,t,path=PathEstimate(False)) == (0., 0., "roundabout_entry_release")
    assert r.phase == "SEEK" and not any(r.course.rings)

def test_stationary_detection_cannot_latch_entry():
    r=RoundaboutRouter();t=PatrolTuning();r.started_at=0.
    r._observe_vv_entry(SimpleNamespace(valid=True),.1,t,.03,0.,(0.,0.))
    assert r.phase=="SEEK"


def test_locked_ordinary_line_cannot_confirm_follow_without_dash_evidence():
    r,t=fixture();r.distance=.121
    r._observe_vv_entry(left_cue(),.5,t,.03,.01,(.3,0.))
    r.phase_distance=1.;r.left_yaw=1.2
    p=PathEstimate(True,.1,.9,near_error=.1,mid_error=.1,route_locked=True,route_observed=True)
    for now in (.6,.63,.66,.69,.72,.75,.78,.81):
        r.accept_path(p,now,t)
    assert r.phase=="ENTRY" and not r.ring_route_acquired

def test_entry_probe_extension_needs_current_left_evidence_and_keeps_hard_deadline():
    r, t = fixture()
    r.distance = .121
    r._observe_vv_entry(left_cue(), .5, t, .03, .01, (.3, 0.))
    assert r.motion_override(.81, t, entry_branch_visible=True)[2] == "roundabout_left_acquire"
    assert r.route_status == "entry_left_branch_route_rejected"
    assert r.motion_override(1.06, t, entry_branch_visible=True) == (0., 0., "roundabout_entry_release")
    assert r.ring_release_source == "entry_confirm_timeout" and not any(r.course.rings)

def test_converging_exit_cancels_generic_approach_without_ring_receipt():
    r, t = fixture()
    r.distance = .2
    exit_cue = SimpleNamespace(valid=True, exit_branch=PathEstimate(True), branch=PathEstimate(False))
    r._observe_vv_entry(exit_cue, .2, t, .03, .01, (.3, 0.))
    assert r.phase == "SEEK" and not any(r.course.rings)
    r._observe_vv_entry(exit_cue, .3, t, .03, .01, (.3, 0.))
    assert r.phase == "SEEK"
