"""V24 measures the pass after marker 5, NOT the post-loop exit search."""
from dataclasses import replace
import pytest
from test_patrol_vv_v16 import cue, armed
from patrol_contour_runtime import PatrolTuning

def marker():
    r,t=armed()
    t=replace(t,roundabout_first_marker_pass_distance_m=.73)
    r.observe(False,0.,0.,t,(.3,0.),(0.,0.,0.))
    r.observe(True,1.,.1,t,(.3,0.),(.03,0.,0.),dash_cue=cue('converging',t))
    assert r.phase=='APPROACH'
    return r,t

def advance(r,t,distance,kind='diverging'):
    # Small real forward odom increments, fresh camera frames.
    steps=round(distance/.01)
    for i in range(1,steps+1):
        r.observe(True,1.,.1+i/30,t,(.3,0.),(.03+i*.01,0.,0.),dash_cue=cue(kind,t))
    return .1+steps/30

def test_defaults_distinguish_entry_distance_from_exit_budget():
    t=PatrolTuning();t.validate()
    assert t.roundabout_first_marker_pass_distance_m==.30  # V25 default; V24 scenarios explicitly pin .73
    assert t.roundabout_first_marker_cooldown_seconds==.5
    assert (t.ring_gyro_exit_seek_distance_m,t.ring_gyro_exit_seek_seconds,t.roundabout_exit_acquire_max_seconds)==(.73,7.,8.)
    assert .73/.12<t.roundabout_approach_max_seconds

def test_current_entry_cannot_trigger_before_pass_distance():
    r,t=marker();now=advance(r,t,.72)
    assert r.phase=='APPROACH' and r.entry_detection_reason=='vv_cooling_keep_tracking'
    assert r.motion_override(now,t) is None  # ordinary tracking, not stop/forced yaw
    r.observe(True,1.,now+1/30,t,(.3,0.),(.771,0.,0.),dash_cue=cue(t=t))
    assert r.phase=='ENTRY' and not any(r.course.rings)

@pytest.mark.parametrize('kind',['converging','missing'])
def test_pass_distance_is_not_itself_entry_authorization(kind):
    r,t=marker();now=advance(r,t,.75,'converging')
    current=cue('converging',t) if kind=='converging' else None
    r.observe(current is not None,1.,now+.04,t,(.3,0.),(.79,0.,0.),dash_cue=current)
    assert r.phase=='APPROACH' and not any(r.course.rings)

def test_stationary_car_cannot_consume_distance_gate():
    r,t=marker()
    for i in range(1,100):
        r.observe(True,1.,.1+i/30,t,(.3,0.),(.03,0.,0.),dash_cue=cue(t=t))
    assert r.phase=='APPROACH' and r.distance-r.first_marker_distance==0
