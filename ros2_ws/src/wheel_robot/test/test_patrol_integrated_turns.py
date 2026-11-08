import sys
from pathlib import Path
from dataclasses import replace
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from patrol_contour_runtime import FixedSpeedController, PathEstimate, PatrolTuning, RoundaboutRouter
from patrol_dash_entry import DashObservation


def test_heading_preview_builds_turn_without_changing_cruise_or_straight():
    t=PatrolTuning()
    curve=PathEstimate(True,-.15,.9,near_error=-.15,mid_error=-.4,route_observed=True)
    c=FixedSpeedController()
    turn=c._steer(curve,1.,t,.03)
    baseline=FixedSpeedController()._steer(replace(curve,route_observed=False),1.,t,.03)
    assert turn > baseline > 0
    straight=replace(curve,near_error=-.15,mid_error=-.15)
    assert FixedSpeedController()._steer(straight,1.,t,.03)==baseline
    assert t.fixed_speed_m_s == .65 and t.max_turn_rad_s == 2.
    opposite=replace(curve,mid_error=.4)
    assert FixedSpeedController()._steer(opposite,1.,t,.03)==baseline


def test_search_deadband_does_not_reverse_at_degradation():
    c,t=FixedSpeedController(),PatrolTuning()
    c.last_valid_at=0.; c.last_time=0.; c.last_command=(.2,0.)
    c.last_good_turn=-.01
    absent=PathEstimate(False)
    for now in np.arange(.1,3.5,.1):
        _,w,_=c.command(absent,float(now),t)
        assert w >= 0


def test_first_visible_left_fork_cannot_bypass_marker_pass():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.gate_detection_allowed=lambda *_:True
    cue=DashObservation(valid=True,branch=PathEstimate(True))
    r._observe_vv_entry(cue,1.,t,.1,.02,(.2,0.))
    assert r.phase=='APPROACH'
    assert r.entry_detection_reason=='vv_first_marker_pass_5'
    r._observe_vv_entry(cue,1.1,t,.1,.02,(.2,0.))
    assert r.phase=='APPROACH'
