"""Gentle true-loss search and no one-frame direction-memory rewrites."""
from dataclasses import replace
import numpy as np
import pytest
from test_patrol_contour import cruise_controller
from patrol_contour_runtime import PatrolTuning,PathEstimate
from patrol_stability import stabilize_masks

def test_active_search_cannot_inherit_two_rad_turn():
    c,t=cruise_controller(-.9,maintain_cruise_during_search=0,loss_grace_seconds=0)
    assert c.last_command[1]>1.5
    for now in [2.03,2.06,2.10,2.50,3.]:
        v,w,_=c.command(PathEstimate(False),now,t)
        assert abs(w)<=.6 and v<=.2

def test_invalid_memory_bridge_is_also_gentle_after_grace():
    c,t=cruise_controller(.1,maintain_cruise_during_search=0,loss_grace_seconds=0)
    c.last_command=(.65,-1.8)
    v,w,mode=c.command(PathEstimate(False),2.03,t)
    assert mode=='path_memory_bridge' and abs(w)<=.6

def test_one_opposite_flash_cannot_rewrite_search_memory_or_reverse():
    c,t=cruise_controller(-.8,maintain_cruise_during_search=0,loss_grace_seconds=0)
    c.command(PathEstimate(False),2.03,t)
    chosen=c.search_turn_goal
    for now in [2.06,2.12,2.18]:
        v,w,mode=c.command(PathEstimate(True,.8,.9,near_error=.8,source='black'),now,t)
        assert mode=='reacquire_confirm' and w==0 and c.search_turn_goal==chosen
        v,w,_=c.command(PathEstimate(False),now+.03,t)
        assert 0<=w<=.6 and c.search_turn_goal==chosen

def test_three_reliable_frames_can_reacquire_other_direction():
    c,t=cruise_controller(-.8,maintain_cruise_during_search=0,loss_grace_seconds=0)
    c.command(PathEstimate(False),2.03,t)
    p=PathEstimate(True,.8,.9,near_error=.8,source='black')
    assert c.command(p,2.06,t)[1]==0
    assert c.command(p,2.09,t)[1]==0
    assert c.command(p,2.12,t)[2]=='fixed_speed'
    assert c.search_turn_goal<0 and c.loss_started_at is None

def test_weak_frames_cannot_prefill_reliable_direction_confirmation():
    c,t=cruise_controller(-.8,maintain_cruise_during_search=0,loss_grace_seconds=0)
    c.command(PathEstimate(False),2.03,t);chosen=c.search_turn_goal
    for now in (2.06,2.09):c.command(PathEstimate(True,.8,.3,near_error=.8),now,t)
    c.command(PathEstimate(True,.8,.9,near_error=.8),2.12,t)
    assert c.turn_memory_hits==1 and c.search_turn_goal==chosen

@pytest.mark.parametrize('kind',['weak','rescue','handoff'])
def test_untrusted_observation_does_not_replace_direction_memory(kind):
    c,t=cruise_controller(-.8,maintain_cruise_during_search=0,loss_grace_seconds=0)
    c.command(PathEstimate(False),2.03,t);chosen=c.search_turn_goal
    p=PathEstimate(True,.8,.3 if kind=='weak' else .9,near_error=.8,rescue=kind=='rescue',handoff=kind=='handoff')
    c.command(p,2.06,t)
    assert c.search_turn_goal==chosen

def test_valid_normal_corner_keeps_original_speed_and_turn_authority():
    c,t=cruise_controller(-.9)
    assert c.last_command[0]==pytest.approx(.65) and c.last_command[1]==pytest.approx(2.)

def test_wide_near_black_survives_default_low_posture_comparison():
    t=PatrolTuning();assert not t.black_wide_guard_enabled
    m=np.zeros((120,160),np.uint8);m[50:115,30:130]=255
    a,e=stabilize_masks(m,m,np.zeros((120,160,3),np.uint8),t)
    assert np.array_equal(a,m) and np.array_equal(e,m)
    b,_=stabilize_masks(m,m,np.zeros((120,160,3),np.uint8),replace(t,black_wide_guard_enabled=1))
    assert b.sum()<a.sum()

@pytest.mark.parametrize('changes',[{'search_max_turn_rad_s':0},{'search_max_turn_rad_s':float('nan')},
    {'search_max_turn_rad_s':2.1},{'black_wide_guard_enabled':2}])
def test_invalid_new_parameters(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()
