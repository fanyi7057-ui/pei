from types import SimpleNamespace
from patrol_contour_runtime import PatrolTuning, RoundaboutRouter
from dataclasses import replace

def test_single_frame_needs_no_branch_or_first_marker():
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    r=RoundaboutRouter();r.started_at=0.
    r._observe_vv_entry(SimpleNamespace(valid=True),.1,t,.03,.01,(.3,0.))
    assert r.phase=="ENTRY" and r.entry_detection_reason=="vv_single_frame_left_entry"

def test_invalid_or_stationary_cue_does_not_enter():
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    for valid,travel,cmd in [(False,.01,(.3,0.)),(True,0.,(0.,0.))]:
        r=RoundaboutRouter();r.started_at=0.
        r._observe_vv_entry(SimpleNamespace(valid=valid),.1,t,.03,travel,cmd)
        assert r.phase=="SEEK"
