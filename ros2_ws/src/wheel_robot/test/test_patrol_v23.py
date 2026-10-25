"""Bounded blind search without changing valid-route cruise."""
from dataclasses import replace
import pytest
from test_patrol_v20 import seek
from patrol_contour_runtime import PatrolTuning, FixedSpeedController, PathEstimate

def test_exit_budget_073():
    r,t,now=seek()
    assert (t.ring_gyro_exit_seek_distance_m,t.ring_gyro_exit_seek_seconds,t.roundabout_exit_acquire_max_seconds)==(.73,7.,8.)
    r.measured_turn.stamp=now+6.2
    assert r.motion_override(now+6.2,t,(.731,0.,0.),PathEstimate(False))[0]==0

def test_blind_speed_is_capped_immediately_and_during_confirmation():
    t=replace(PatrolTuning(),maintain_cruise_during_search=0);c=FixedSpeedController()
    p=PathEstimate(True,-.5,.9,near_error=-.5,source='black')
    for i in range(40): c.command(p,i*.04,t)
    assert c.last_command[0]>.6
    assert c.command(PathEstimate(False),1.61,t)[0]<=.2
    assert c.command(p,1.65,t)[0]<=.2
    assert c.command(p,1.69,t)[0]<=.2
    assert c.command(p,1.73,t)[0]>.2

def test_reliable_opposite_observation_cancels_wrong_way_output():
    t=replace(PatrolTuning(),maintain_cruise_during_search=0);c=FixedSpeedController()
    c.command(PathEstimate(True,.8,.9,near_error=.8,source='black'),0,t)
    c.command(PathEstimate(False),.04,t)
    c.last_command=(.2,-1.5)
    v,w,_=c.command(PathEstimate(True,-.8,.9,near_error=-.8,source='black'),.08,t)
    assert w==0 and v<=.2

@pytest.mark.parametrize('speed',[0,-1,.31,float('nan'),float('inf')])
def test_invalid_search_speed(speed):
    with pytest.raises(ValueError): replace(PatrolTuning(),search_speed_m_s=speed).validate()
