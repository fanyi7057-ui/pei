"""V25 shorten entrance pass only; do not silently modify loss control."""
from dataclasses import replace
from test_patrol_v24 import marker, advance
from test_patrol_vv_v16 import cue
from patrol_contour_runtime import PatrolTuning

def test_default_entrance_pass_changes_not_exit_or_search_speed():
    t=PatrolTuning();t.validate()
    assert t.roundabout_first_marker_pass_distance_m==.30
    assert t.roundabout_first_marker_cooldown_seconds==.5
    assert (t.ring_gyro_exit_seek_distance_m,t.ring_gyro_exit_seek_seconds)==(.73,7.)
    assert (t.search_speed_m_s,t.maintain_cruise_during_search)==(.20,0)

def test_current_entry_can_trigger_after_point_three_not_point_seven_three():
    r,t=marker();t=replace(t,roundabout_first_marker_pass_distance_m=.30)
    now=advance(r,t,.29)
    assert r.phase=='APPROACH' and r.motion_override(now,t) is None
    r.observe(True,1.,now+1/30,t,(.3,0.),(.341,0.,0.),dash_cue=cue(t=t))
    assert r.phase=='ENTRY' and not any(r.course.rings)

def test_only_distance_without_current_fork_stays_tracking():
    r,t=marker();t=replace(t,roundabout_first_marker_pass_distance_m=.30)
    advance(r,t,.4,'converging')
    assert r.phase=='APPROACH'
