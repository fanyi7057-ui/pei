"""V14 regressions from log 57f0d083: chronic slowdown and rapid lost-line spin."""
from dataclasses import replace
import math
import numpy as np
import pytest

from test_patrol_contour import make_node, node_module
from patrol_contour_runtime import PatrolTuning, FixedSpeedController, PathEstimate, RoundaboutRouter


def straight(conf=.63):
    return PathEstimate(True,.02,conf,source="black",near_error=.02,mid_error=.03,far_error=.04)


def run_cruise(controller,tuning,confidence=.63):
    for now in np.arange(0.,3.,1/30):
        result=controller.command(straight(confidence),float(now),tuning)
    return result


@pytest.mark.parametrize("conf",[.40,.45,.52,.63,.64,.90])
def test_normal_black_confidence_no_longer_latches_startup_limit(conf):
    c,t=FixedSpeedController(),PatrolTuning()
    v,_,_=run_cruise(c,t,conf)
    assert v==pytest.approx(.65)
    assert c.speed_reason=="stable_cruise"


def test_confidence_changes_do_not_restart_startup_clock():
    c,t=FixedSpeedController(),PatrolTuning()
    for i in range(150):
        conf=(.45,.63,.64,.52)[i%4]
        v,_,_=c.command(straight(conf),i/30,t)
    assert v==pytest.approx(.65)
    assert c.startup_until==t.speed_stable_seconds


def test_startup_is_brief_and_true_s_bend_still_brakes():
    c,t=FixedSpeedController(),PatrolTuning()
    for i in range(15):
        v,_,_=c.command(straight(),i/30,t)
        assert v<=t.startup_speed_m_s
    run_cruise(c,t)
    for i in range(30):
        e=.6 if i%2 else -.6
        p=PathEstimate(True,.1,.9,source="black",near_error=0.,mid_error=e,far_error=e)
        v,_,_=c.command(p,3.+i/30,t)
    assert v==pytest.approx(.45) and c.speed_reason=="bend_preview_limit"


def test_ring_slowdown_starts_at_approach_not_before_detection():
    c,t=FixedSpeedController(),PatrolTuning()
    assert run_cruise(c,t)[0]==pytest.approx(.65)
    for i in range(30): v,_,_=c.command(straight(.8),3.+i/30,t,ring_approach=True)
    assert v==pytest.approx(.4) and c.speed_reason=="ring_detected_approach_limit"
    for i in range(30): v,_,_=c.command(straight(.8),4.+i/30,t,ring_tracking=True)
    assert v==pytest.approx(.3)


@pytest.mark.parametrize("old_turn",[-2.,-1.51,1.46,2.])
@pytest.mark.parametrize("mode",["bridge","active","ring"])
def test_first_lost_frame_has_hard_final_yaw_limit(old_turn,mode):
    c,t=FixedSpeedController(),PatrolTuning()
    run_cruise(c,t)
    c.last_command=(.65,old_turn)
    c.last_good_turn=c.last_curve_turn=old_turn
    c.last_curve_at=2.99 if mode!="bridge" else -math.inf
    if mode=="bridge": c.last_curve_turn=0.
    v,w,status=c.command(PathEstimate(False),3.,t,search_direction=1 if mode=="ring" else None,
                          ring_tracking=mode=="ring")
    assert v<=.12 and abs(w)<=.35
    assert status in ("path_memory_bridge","active_search")
    assert c.last_command==(v,w)


def test_search_guard_is_independent_of_automatic_speed_planning_switch():
    c=FixedSpeedController()
    t=replace(PatrolTuning(),adaptive_speed_enabled=0)
    run_cruise(c,t)
    c.last_command=(.65,2.)
    c.last_good_turn=2.
    assert abs(c.command(PathEstimate(False),3.,t)[1])<=.35


@pytest.mark.parametrize("conf,rescue",[(.31,True),(.31,False),(.39,False)])
def test_weak_or_single_band_cannot_command_full_turn(conf,rescue):
    c,t=FixedSpeedController(),PatrolTuning()
    run_cruise(c,t)
    c.last_command=(.12,-1.51)
    weak=PathEstimate(True,.95,conf,source="black",rescue=rescue)
    _,w,_=c.command(weak,3.,t)
    assert abs(w)<=.45


def test_weak_flicker_cannot_flip_search_direction_or_refresh_budget():
    c,t=FixedSpeedController(),PatrolTuning()
    run_cruise(c,t)
    c.last_good_turn=.8
    c.last_curve_turn=.8
    c.last_curve_at=2.99
    c.command(PathEstimate(False),3.,t)
    goal=c.search_turn_goal
    started=c.loss_started_at
    for i in range(1,30):
        p=PathEstimate(True,.9 if i%2 else -.9,.31,source="black",rescue=True) if i%3 else PathEstimate(False)
        c.command(p,3.+i/30,t)
        assert c.search_turn_goal==goal and c.loss_started_at==started
        assert abs(c.last_command[1])<=.45
    assert c.reacquire_streak==0


def test_reliable_reacquisition_restores_normal_turn_authority():
    c,t=FixedSpeedController(),PatrolTuning()
    run_cruise(c,t)
    c.command(PathEstimate(False),3.,t)
    p=PathEstimate(True,-.8,.9,source="black",near_error=-.8,mid_error=-.8)
    for i in range(1,15): v,w,mode=c.command(p,3.+i/30,t)
    assert mode=="fixed_speed" and c.loss_started_at is None
    assert w>t.weak_path_turn_max_rad_s and v<=.45


def test_gyro_seek_speed_does_not_increase_with_startup_speed():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("EXIT",0.)
    r.gyro_exit_requested=True
    assert r.motion_override(.1,t)==(.2,0.,"gyro_exit_seek")
    assert t.roundabout_entry_turn_rad_s==1.2 and t.roundabout_follow_min_seconds==2.
    assert t.roundabout_follow_max_seconds==28.


def test_actual_node_publishes_limited_search_not_only_raw_goal(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    node.patrol_controller.last_valid_at=9.97
    node.patrol_controller.last_time=9.97
    node.patrol_controller.last_command=(.65,-1.51)
    node.patrol_controller.last_good_turn=-1.51
    node.vision.estimate=lambda *_a,**_k:PathEstimate(False)
    node._run_control_cycle()
    assert 0<=commands[-1][0]<=.12 and abs(commands[-1][1])<=.35
    assert node.patrol_controller.last_command==commands[-1]


@pytest.mark.parametrize("changes",[{"search_turn_max_rad_s":0},{"weak_path_turn_max_rad_s":-1},
    {"recovery_angular_accel_rad_s2":0},{"recovery_turn_guard_enabled":2},
    {"roundabout_approach_speed_m_s":-1},{"ring_gyro_exit_speed_m_s":-1}])
def test_new_snapshot_values_are_validated(changes):
    with pytest.raises(ValueError): replace(PatrolTuning(),**changes).validate()
