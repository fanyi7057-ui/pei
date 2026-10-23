"""Synthetic vision and non-actuating node tests; no ROS or motors needed."""
from dataclasses import asdict, replace
import importlib.util
import math
import os
from pathlib import Path
import sys
import threading
import time
import types

import cv2
import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from patrol_contour_runtime import (
    FixedSpeedController, PathEstimate, PathSelector, PatrolFileWatcher,
    PatrolTuning, RoundaboutRouter, GateChainTracker, PartialGateTracker, EntryApproachProof, ApproachRouteMemory, CourseProgress, ExitRouteCommit, RingRouteMemory, MeasuredRingTurn, observed_route,
    _repair_short_gaps, center_error, road_centerline, road_masks, detect_crosswalk,
)


@pytest.mark.parametrize("degrees",[0,30,60,90,180,-90])
def test_v12_actual_driver_assignments_preserve_firmware_yaw(degrees):
    import ast
    import re
    cpp=SCRIPTS.parents[1]/"wheeled_legged_pkg/src/wl_base_node.cpp"
    assignments=dict(re.findall(r"imu_msg.orientation\.([xyzw]) = ROS_body.ImuData\.(q[0-3]);",cpp.read_text(encoding="utf-8-sig")))
    a=math.radians(degrees)/2
    firmware={"q0":math.cos(a),"q1":0.,"q2":0.,"q3":math.sin(a)}
    tree=ast.parse((SCRIPTS/"icn.py").read_text(encoding="utf-8-sig"))
    helper=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="quaternion_to_yaw")
    namespace={"math":math}
    exec(compile(ast.Module(body=[helper],type_ignores=[]),"actual yaw helper","exec"),namespace)
    actual=namespace["quaternion_to_yaw"](*(firmware[assignments[k]] for k in "xyzw"))
    assert actual==pytest.approx(math.radians(degrees))
    assert assignments=={"w":"q0","x":"q1","y":"q2","z":"q3"}


def partial_chain(x=135.):
    return [(x-3,110.,10,12,100),(x,140.,11,13,110),(x+3,170.,12,14,120)]


def test_v12_partial_chain_requires_fresh_longitudinal_left_evidence():
    tracker,t=PartialGateTracker(),PatrolTuning()
    seen=[tracker.update(partial_chain(),320,240,i/30,t) for i in range(8)]
    assert not any(seen[:3]) and any(seen[3:])
    assert len(tracker.chain)==3 and tracker.reason=="partial_tracked"


@pytest.mark.parametrize("bad",["horizontal","two","right","short","duplicate","stale","jump","disabled","wrong_slope","shape"])
def test_v12_partial_chain_rejects_non_entry_patterns(bad):
    tr,t=PartialGateTracker(),PatrolTuning()
    if bad=="disabled":t=replace(t,roundabout_partial_gate_enabled=0)
    seen=[]
    for i in range(10):
        c=partial_chain()
        now=i/30
        if bad=="horizontal":c=[(70+j*40,150.,10,12,100) for j in range(5)]
        elif bad=="two":c=c[:2]
        elif bad=="right":c=partial_chain(250.)
        elif bad=="short":c=[(130.,130.+j*10,10,12,100) for j in range(3)]
        elif bad=="duplicate":now=0.
        elif bad=="stale":now=i*.3
        elif bad=="jump":c=partial_chain(70. if i%2 else 165.)
        elif bad=="wrong_slope":c=[(160.,110.,10,12,100),(130.,140.,10,12,100),(100.,170.,10,12,100)]
        elif bad=="shape":c=[(70.,110.,10,12,100),(165.,140.,10,12,100),(75.,170.,10,12,100)]
        seen.append(tr.update(c,320,240,now,t))
    assert not any(seen)


def test_v14_static_partial_chain_without_approach_geometry_cannot_start_entry(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    frame=np.full((240,320,3),245,np.uint8)
    for x,y,w,h,area in partial_chain():
        cv2.rectangle(frame,(int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),(0,0,0),-1)
    for i in range(18):
        v.motion_command=(.65,0.)
        v.estimate(frame,t,i/30)
    assert v.router.phase=="SEEK" and v.gate_score<.70  # three blocks below VV minimum
    assert v.router.completed==0 and not v.entry_ready


def test_v12_partial_entry_is_not_used_before_start_delay_or_in_exit(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    frame=np.full((240,320,3),245,np.uint8)
    v._small_neutral_components=lambda *_args:pytest.fail("entry components ran before arming")
    v.estimate(frame,PatrolTuning(),.1)
    assert not v.dash_observation.valid and v.router.phase=="SEEK"


def test_v12_sensor_disagreement_invalidates_no_turn_and_loop_proof():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("ENTRY",0.)
    for i in range(24):
        now=i*.1
        r.observe(False,0.,now,t,(.65,1.2),pose=(i*.04,0.,i*.1),heading=("imu",0.,now))
    assert r.measured_turn.reason=="imu_odom_turn_conflict"
    assert not r.measured_turn.available
    assert r.phase=="ENTRY" and r.aborted_rings==0  # not a false no-turn abort


def loop_candidate_router():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    r.branch_acquired=True
    r.ring_route_acquired=True  # fixture for already-acquired FOLLOW
    for i,deg in enumerate(np.arange(0,331,10)):
        now=i*.1
        yaw=(math.radians(deg)+math.pi)%(2*math.pi)-math.pi
        r.observe(False,0.,now,t,(.65,1.),pose=(i*.05,0.,yaw),heading=("imu",yaw,now))
    return r,t,now


def test_v12_full_turn_is_exit_candidate_not_immediate_success():
    r,t,now=loop_candidate_router()
    assert r.phase=="EXIT" and r.loop_candidate_at is not None
    assert r.completed==0 and r.yaw_releases==0 and not r.crosswalk_armed(now,t)
    p=PathEstimate(True,-.1,.9,near_error=-.1,mid_error=-.1,route_locked=True,route_observed=True)
    r.phase_distance=.6
    for delta in (.1,.2,.3):r.accept_path(p,now+delta,t)
    assert r.phase=="SEEK_CROSSWALK" and r.completed==1
    assert r.ring_release_source=="visual_exit"


def test_v12_unconfirmed_exit_has_bounded_non_successful_release():
    r,t,now=loop_candidate_router()
    for dt in np.arange(.1,2.9,.1):
        r.observe(False,0.,now+float(dt),t,(.65,.1))
    assert r.phase=="SEEK" and r.completed==0
    assert not r.crosswalk_armed(now+3.,t)
    assert r.ring_release_source=="yaw_exit_unconfirmed"


def route_path(e=0.,source="road",**kw):
    return PathEstimate(True,e,.9,near_error=e,mid_error=e,source=source,route_observed=True,**kw)


def test_v12_exit_commit_prefers_contour_but_follows_its_bend():
    lock,t=ExitRouteCommit(),PatrolTuning()
    assert lock.begin(route_path(),0.,t)
    p=lock.select(route_path(.1,"black"),route_path(-.2),.1,.65,t)
    assert p.source=="road" and p.error==-.2  # NOT forced zero yaw
    c=FixedSpeedController()
    assert c.command(p,.1,t)[1]>0


def test_v12_exit_commit_same_route_black_fallback_and_disconnected_rejection():
    lock,t=ExitRouteCommit(),PatrolTuning()
    lock.begin(route_path(-.2),0.,t)
    assert lock.select(route_path(-.15,"black"),PathEstimate(False),.1,.65,t).source=="black"
    p=lock.select(route_path(.65,"black"),route_path(.65),.2,.65,t)
    assert not p.valid and lock.reason=="exit_route_missing"
    assert lock.path.near_error<0


def test_v12_exit_commit_expiry_not_renewed_by_observations_or_distance():
    t=replace(PatrolTuning(),roundabout_exit_commit_distance_m=.15)
    lock=ExitRouteCommit();lock.begin(route_path(),0.,t)
    for i in range(1,5):lock.select(route_path(),route_path(),i*.1,.65,t)
    assert not lock.active(.4,t)
    lock.begin(route_path(),0.,PatrolTuning())
    for i in range(1,49):lock.select(route_path(),route_path(),i*.1,.1,PatrolTuning())
    assert lock.started_at==0. and not lock.active(5.,PatrolTuning())


def test_v12_exit_commit_does_not_manufacture_geometry_and_reset_clears(monkeypatch):
    lock,t=ExitRouteCommit(),PatrolTuning()
    assert not lock.begin(PathEstimate(False),0.,t)
    assert not lock.begin(route_path(rescue=True),0.,t)
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision();v.exit_commit.begin(route_path(),0.,t)
    v.reset_course()
    assert not v.exit_commit.active(.1,t)


def test_v13_normal_black_return_is_v11_not_v12_extra_road_veto():
    selector,t=PathSelector(),PatrolTuning()
    absent=PathEstimate(False)
    selector.select(absent,route_path(-.4),0.,t)
    for i in range(1,10):
        p,source=selector.select(route_path(.4,"black"),route_path(-.4),i*.03,t)
    assert source=="black"  # V12 extra road agreement veto intentionally removed
    for i in range(10,14):p,source=selector.select(route_path(-.38,"black"),route_path(-.4),i*.03,t)
    assert source=="black"


def test_v13_normal_reverse_turn_restored_without_v12_guard():
    c,t=cruise_controller(-.9)
    c.command(route_path(-.85,"black"),2.03,t)
    c.command(route_path(.55,"black"),2.06,t)
    assert not hasattr(c,"unwind_guard") and c.raw_angular<-.8
    assert c.last_command[0]==pytest.approx(t.fixed_speed_m_s)
    c.command(route_path(.55,"black"),2.09,t)
    c.command(route_path(.55,"black"),2.12,t)
    assert c.raw_angular<-.8  # V11 response, not the withdrawn V12 guard


def test_v12_actual_visual_exit_arms_commit_and_prefers_same_contour(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v,t=module.PatrolVision(),PatrolTuning()
    v.router._phase("EXIT",0.)
    v.router.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    v.router.branch_acquired=True
    v.router.phase_distance=1.
    frame=cv2.cvtColor(scene(),cv2.COLOR_HSV2BGR)
    cv2.line(frame,(160,235),(160,65),(0,0,0),6)
    for i in range(5):
        v.motion_command=(.65,0.)
        p=v.estimate(frame,t,.1+i*.04)
    assert v.router.ring_release_source=="visual_exit" and v.router.completed==1
    assert v.exit_commit.active(.30,t)
    p=v.estimate(frame,t,.32)
    assert p.valid and p.source=="road" and v.exit_commit.reason.startswith("exit_contour_priority")
    assert v.router.phase=="SEEK_CROSSWALK"  # black blocks cannot rearm entry


def test_v12_branch_frames_cannot_drift_exit_anchor_back_to_island():
    lock,t=ExitRouteCommit(),PatrolTuning()
    lock.begin(route_path(0.),0.,t)
    for i in range(1,7):
        p=lock.select(PathEstimate(False),route_path(.2,branch_rows=2),i*.03,.65,t)
        assert p.valid and lock.path.near_error==0.
    assert not lock.select(PathEstimate(False),route_path(.4,branch_rows=2),.22,.65,t).valid


def test_v12_exit_commit_straight_status_does_not_disable_next_bend():
    lock,t=ExitRouteCommit(),PatrolTuning()
    lock.begin(route_path(),0.,t)
    for i in range(1,4):lock.select(PathEstimate(False),route_path(),i*.03,.65,t)
    assert lock.reason.endswith("_straight")
    p=lock.select(PathEstimate(False),route_path(.2),.15,.65,t)
    assert p.valid and p.error==.2 and not lock.reason.endswith("_straight")


@pytest.mark.parametrize("changes",[
    {"corner_unwind_enabled":2},{"corner_unwind_seconds":0},
    {"corner_unwind_straight_error":0},{"corner_unwind_straight_frames":1.5},
    {"corner_reverse_confirm_frames":0},{"black_return_agreement":1.1},
    {"roundabout_partial_gate_enabled":2},{"roundabout_partial_gate_min_blocks":2},
    {"roundabout_partial_gate_min_left_blocks":4},{"roundabout_partial_gate_left_ratio":0},
    {"roundabout_partial_gate_min_span_ratio":0},{"roundabout_partial_gate_frames":1.5},
    {"roundabout_partial_gate_min_seconds":0},{"roundabout_heading_disagreement_deg":0},
    {"roundabout_require_exit_route":2},{"roundabout_exit_acquire_max_seconds":0},
    {"roundabout_exit_commit_enabled":2},{"roundabout_exit_commit_seconds":0},
    {"roundabout_exit_commit_distance_m":0},{"roundabout_exit_commit_max_delta":0},
    {"roundabout_exit_commit_straight_frames":0},
])
def test_v12_new_fields_reject_invalid_snapshot(changes):
    retired={"corner_unwind_enabled","corner_unwind_seconds","corner_unwind_straight_error","corner_unwind_straight_frames","corner_reverse_confirm_frames","black_return_agreement"}
    expected=TypeError if set(changes)&retired else ValueError
    with pytest.raises(expected):replace(PatrolTuning(),**changes).validate()


@pytest.mark.parametrize("name,expected",[
    ("PathSelector","8e5d914aff1fe47fd4039d05214dae936fbf56fd79d36ade94128af131015224"),
    ("FixedSpeedController","dbc1fcb9ee7d518d5d5197fd3b77b0d488b1c6437e674a0e029c8a840d0fbe2f"),
])
def test_v13_ordinary_classes_exact_v11_source(name,expected):
    import ast
    import hashlib
    source=(SCRIPTS/"patrol_contour_runtime.py").read_text(encoding="utf-8-sig")
    node=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name==name)
    assert hashlib.sha256(ast.get_source_segment(source,node).strip().encode()).hexdigest()==expected


@pytest.mark.parametrize("error,confidence",[(-.819,.31),(.819,.31),(-.7,.55),(.7,.55)])
def test_v13_actual_rescue_not_suppressed_by_recent_opposite_corner(error,confidence):
    c,t=cruise_controller(-math.copysign(.9,error))
    p=PathEstimate(True,error,confidence,source="black",rescue=True)
    w=c._steer(p,2.03,t,.03)
    assert w*error<0 and abs(w)>1.
    assert not hasattr(c,"unwind_guard")


@pytest.mark.parametrize("owned",[False,True])
def test_v13_27_degree_candidate_cannot_release_entry_or_enable_boost(owned):
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("ENTRY",0.)
    r.phase_distance=.7;r.branch_acquired=owned
    r.measured_turn.source="imu";r.measured_turn.angle=math.radians(27.5)
    for i in range(15):
        now=.7+i*.03;r.measured_turn.stamp=now
        p=route_path(-.18,route_locked=True)
        r.accept_path(p,now,t)
        assert r.phase=="ENTRY"
        assert r.motion_override(now,t)[2]=="roundabout_left_acquire"
        assert not r.follow_boost_allowed(p,now,t)


def test_v13_after_55_degrees_stable_route_still_required():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r.phase_distance=.7;r.left_yaw=math.radians(60.)
    p=route_path(-.15,route_locked=True)
    r.accept_path(p,1.,t)
    assert r.phase=="ENTRY"
    for i in range(1,9):r.accept_path(p,1.+i*.03,t)
    assert r.phase=="FOLLOW" and r.branch_acquired


@pytest.mark.parametrize("bad",["early","little_turn","stale","rescue","disagree","wrong_phase","not_owned"])
def test_v13_follow_multiplier_requires_actual_stable_progress(bad):
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("FOLLOW",0.);r.branch_acquired=True
    r.measured_turn.source="imu";r.measured_turn.angle=math.radians(120.);r.measured_turn.stamp=2.
    now=2.;p=route_path(-.1)
    r.follow_stable_since=0.
    assert r.follow_boost_allowed(p,now,t)
    if bad=="early":r.phase_at=1.5
    elif bad=="little_turn":r.measured_turn.angle=math.radians(33.)
    elif bad=="stale":r.measured_turn.stamp=1.
    elif bad=="rescue":p=replace(p,rescue=True)
    elif bad=="disagree":p=replace(p,mid_error=.7)
    elif bad=="wrong_phase":r._phase("ENTRY",0.)
    elif bad=="not_owned":r.branch_acquired=False
    assert not r.follow_boost_allowed(p,now,t)


@pytest.mark.parametrize("kind",["road","black"])
def test_v13_exit_accepts_bounded_real_same_route_weak_path(kind):
    lock,t=ExitRouteCommit(),PatrolTuning();lock.begin(route_path(),0.,t)
    p=PathEstimate(True,.05,.31,points=np.array([[165.,210.]]),near_error=.05,mid_error=.05,
                   route_observed=False,rescue=kind=="black",source=kind)
    result=lock.select(p if kind=="black" else PathEstimate(False),p if kind=="road" else PathEstimate(False),.1,.65,t)
    assert result.valid and result.source==kind
    assert lock.observed_at==0. and lock.path.near_error==0.
    assert lock.select(p,PathEstimate(False),.41,.65,t) is None
    assert lock.reason=="exit_unobservable_release"


def test_v13_exit_weak_wrong_branch_still_rejected():
    lock,t=ExitRouteCommit(),PatrolTuning();lock.begin(route_path(),0.,t)
    p=PathEstimate(True,.8,.9,points=np.array([[280.,220.]]),near_error=.8,rescue=True)
    assert not lock.select(p,PathEstimate(False),.1,.65,t).valid


def test_v13_follow_loss_restarts_stability_before_boost():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("FOLLOW",0.);r.branch_acquired=True
    r.measured_turn.source="imu";r.measured_turn.angle=math.radians(150.)
    p=route_path()
    r.accept_path(p,0.,t);r.measured_turn.stamp=1.1
    assert r.follow_boost_allowed(p,1.1,t)
    r.accept_path(PathEstimate(False),1.2,t)
    r.accept_path(p,1.3,t);r.measured_turn.stamp=1.3
    assert not r.follow_boost_allowed(p,1.3,t)
    r.measured_turn.stamp=2.4
    assert r.follow_boost_allowed(p,2.4,t)


@pytest.mark.parametrize("changes",[
    {"roundabout_entry_commit_enabled":2},{"roundabout_follow_boost_delay_seconds":-1},
    {"roundabout_follow_boost_min_turn_deg":361},{"roundabout_exit_recovery_seconds":0},
    {"roundabout_exit_recovery_seconds":2.1},
])
def test_v13_additional_hot_parameters_reject_invalid_values(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()


def scene(center=160, width=150, black_hole=False):
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (255, 0, 0)  # blue background
    polygon = np.array([[center-width//3, 0], [center+width//3, 0], [center+width//2, 239], [center-width//2, 239]], np.int32)
    cv2.fillPoly(image, [polygon], (245, 245, 245))
    if black_hole:
        cv2.rectangle(image, (center-5, 100), (center+5, 180), (0, 0, 0), -1)
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


@pytest.mark.parametrize("center,sign", [(160, 0), (120, -1), (200, 1)])
def test_contour_midpoints_and_direction(center, sign):
    estimate = road_centerline(scene(center), PatrolTuning())
    assert estimate.valid
    if sign:
        assert estimate.error * sign > 0
    else:
        assert abs(estimate.error) < .02
    assert len(estimate.points) > 100  # dense row centerline, not 8 bands


def test_black_hole_is_not_new_road_branch():
    normal = road_centerline(scene(), PatrolTuning())
    hole = road_centerline(scene(black_hole=True), PatrolTuning())
    assert hole.valid
    assert abs(hole.error-normal.error) < .005


@pytest.mark.parametrize("color", [(255, 255, 255), (255, 0, 0), (0, 0, 0)])
def test_uniform_images_cannot_invent_road(color):
    image = np.full((240, 320, 3), color, np.uint8)
    result = road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), PatrolTuning())
    assert not result.valid


def test_white_fragment_not_connected_to_near_seed_is_rejected():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (255, 0, 0)
    cv2.rectangle(image, (100, 70), (220, 160), (255, 255, 255), -1)
    assert not road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), PatrolTuning()).valid


def test_single_visible_edge_requires_measured_width_not_image_border():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (255, 0, 0)
    image[:, :220] = 255
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    assert not road_centerline(hsv, PatrolTuning()).valid
    widths = np.array([[150., 0], [150., 239]])
    result = road_centerline(hsv, PatrolTuning(), previous_widths=widths)
    assert result.valid and result.error < 0
    assert result.single_edge and result.dual_fraction == 0
    assert result.confidence <= .5
    assert np.mean(result.points[:, 0]) == pytest.approx(219.-75.)


def test_gap_repair_cannot_bridge_long_unknown_region():
    original = np.array([1., np.nan, 3., np.nan, np.nan, np.nan, 7.])
    repaired = _repair_short_gaps(original, 1)
    assert repaired[1] == 2.
    assert np.isnan(repaired[3:6]).all()
    assert np.isnan(original[1])  # no mutation


def test_road_error_uses_near_geometry_not_remote_row_average():
    points = np.array([[180, 60], [200, 120], [220, 180], [319, 230]])
    result = center_error(points, 320, 240, PatrolTuning())
    near_x = 220+(319-220)*(240*.86-180)/(230-180)
    mid_x = 200+(220-200)*(240*.60-120)/(180-120)
    near, mid = (near_x-160)/160, (mid_x-160)/160
    assert result == pytest.approx(.96*near+.04*mid+.02*(mid-near))


def test_source_handoff_and_confirmed_return():
    selector, tuning = PathSelector(), PatrolTuning()
    black = PathEstimate(True, -.2, .9)
    road = PathEstimate(True, .2, .9)
    absent = PathEstimate(False)
    assert selector.select(black, absent, 1., tuning)[1] == "black"
    first, source = selector.select(absent, road, 1.1, tuning)
    assert source == "road" and first.error == pytest.approx(-.2)
    assert selector.select(black, road, 1.2, tuning)[1] == "road"
    assert selector.select(black, road, 1.3, tuning)[1] == "road"
    assert selector.select(black, road, 1.4, tuning)[1] == "black"


@pytest.mark.parametrize("angle", [-1., -.1, 0., .1, 1.])
def test_visible_path_always_same_speed_even_with_large_turn(angle):
    controller, tuning = FixedSpeedController(), PatrolTuning()
    for now in np.arange(0, 3., .05):
        linear, angular, mode = controller.command(PathEstimate(True, angle, .9), now, tuning)
    assert linear == pytest.approx(tuning.fixed_speed_m_s)
    assert mode == "fixed_speed"
    assert abs(angular) <= tuning.max_turn_rad_s
    if angle:
        assert angular * angle < 0  # right image error -> negative ROS yaw


def test_short_loss_keeps_issued_speed_then_final_timeout_stops():
    controller, tuning = FixedSpeedController(), replace(PatrolTuning(),search_degraded_enabled=0)
    for now in np.arange(0, 2.01, .05):
        controller.command(PathEstimate(True, -.1, .9), now, tuning)
    v1, w1, mode = controller.command(PathEstimate(False), 2.1, tuning)
    v2, _, _ = controller.command(PathEstimate(False), 2.4, tuning)
    assert mode == "path_memory_bridge"
    assert v2 == v1 == pytest.approx(tuning.fixed_speed_m_s)
    assert w1 > 0
    assert controller.command(PathEstimate(False), 2.81, tuning)[2] == "active_search"
    assert controller.command(PathEstimate(False), 5.01, tuning) == (0., 0., "localization_fault_latched")
    assert controller.command(PathEstimate(True, 0, 1), 5.05, tuning)[2] == "localization_fault_latched"


def test_never_move_blind_at_start_or_when_disabled():
    controller, tuning = FixedSpeedController(), PatrolTuning()
    assert controller.command(PathEstimate(False), 1., tuning)[:2] == (0., 0.)
    assert controller.command(PathEstimate(True, .3, 1), 2., replace(tuning, enabled=0))[:2] == (0., 0.)


@pytest.mark.parametrize("changes", [
    {"fixed_speed_m_s": math.nan}, {"fixed_speed_m_s": -1}, {"enabled": True},
    {"enabled": 2}, {"roi_top_ratio": .99}, {"close_kernel_px": 4},
    {"black_enabled": 0, "contour_enabled": 0}, {"blue_h_max": 200},
    {"posture_height_m": .4}, {"posture_pitch_deg": -40}, {"black_return_frames": 1.5},
    {"max_turn_rad_s": math.inf}, {"linear_accel_m_s2": 0},
])
def test_invalid_snapshot_rejected(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(), **changes).validate()


def test_no_hidden_numeric_speed_ceiling():
    assert replace(PatrolTuning(), fixed_speed_m_s=8., max_turn_rad_s=10.).validate().fixed_speed_m_s == 8.


def write_config(path, tuning):
    path.write_text(yaml.safe_dump({"patrol_test": asdict(tuning)}), encoding="utf-8")


def test_file_reload_atomic_replace_reject_and_recover(tmp_path):
    path = tmp_path / "patrol_test.yaml"
    tuning = PatrolTuning()
    write_config(path, tuning)
    watcher = PatrolFileWatcher(path)
    assert watcher.poll()[0] == tuning
    assert watcher.poll() == (None, "")
    replacement = tmp_path / "saved.yaml"
    changed = replace(tuning, fixed_speed_m_s=.8)
    write_config(replacement, changed)
    # Windows scanners may briefly hold the destination without delete share.
    # Retry only this TEST save operation, never production loading or motion.
    # Persistent denial still fails; Linux permissions are never suppressed.
    for attempt in range(20):
        try:
            replacement.replace(path)
            break
        except PermissionError as error:
            if os.name != "nt" or error.winerror != 5 or attempt == 19:
                raise
            time.sleep(.05)
    assert watcher.poll()[0] == changed
    path.write_text("patrol_test: [broken", encoding="utf-8")
    assert watcher.poll()[0] is None
    assert watcher.poll() == (None, "")
    write_config(path, tuning)
    assert watcher.poll()[0] == tuning
    path.unlink()
    assert watcher.poll()[0] is None


def test_shipped_config_has_all_documented_values():
    path = SCRIPTS.parent / "config" / "patrol_test.yaml"
    tuning, message = PatrolFileWatcher(path).poll()
    assert tuning == PatrolTuning(), message


@pytest.fixture
def node_module(monkeypatch):
    # Import the actual experiment module with only ROS/base class definitions
    # stubbed, then execute its real control cycle without any publishers.
    fake_base = types.ModuleType("icn")
    fake_base.CourseVision = object
    fake_base.WheelLegRaceNode = object
    fake_base.SHOW_DEBUG = False
    fake_base.ENABLE_MOTION = True
    monkeypatch.setitem(sys.modules, "icn", fake_base)
    spec = importlib.util.spec_from_file_location("patrol_node_under_test", SCRIPTS / "patrol_contour_test.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_node(module, monkeypatch, now=10.):
    monkeypatch.setattr(module.time, "monotonic", lambda: now)
    node = module.PatrolTestNode.__new__(module.PatrolTestNode)
    node._patrol_ready = True
    node.patrol_tuning = PatrolTuning()
    node.patrol_controller = FixedSpeedController()
    node._frame_lock = threading.Lock()
    node._camera_healthy = True
    node._latest_frame_time = now
    node._frame_sequence = 1
    node._last_processed_sequence = -1
    node._latest_frame = np.zeros((240, 320, 3), np.uint8)
    node._last_command = (.5, .2)
    node._run_stand_gate = lambda _: False
    node._run_start_cue = lambda _: False
    node._publish_race_posture = lambda: None
    commands = []
    node._publish_command = lambda v, w: commands.append((v, w))
    def zero():
        node._last_command = (0., 0.)
        commands.append((0., 0.))
    node._publish_zero = zero
    node._last_display_at = 0.
    node._stats_at = now
    node._stats_sequence = 0
    node._processed_count = node._displayed_count = 0
    node._capture_fps = node._process_fps = node._display_fps = 0.
    node._last_patrol_log = now
    node.vision = types.SimpleNamespace(source="black", estimate=lambda *_args, **_kwargs: PathEstimate(True, -.1, .9))
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda _: None, info=lambda _: None, error=lambda _: None)
    def forbidden(*_args):
        pytest.fail("experiment must not execute competition/step/old speed code")
    node._try_lock_step = forbidden
    node._sync_course_state = forbidden
    node._planned_command = forbidden
    node._limit_command = forbidden
    return node, commands


def test_real_node_cycle_does_not_enter_original_task_or_speed_branches(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node._run_control_cycle()
    assert len(commands) == 1
    assert node._last_processed_sequence == 1


def test_real_node_gate_blocks_vision(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node._run_stand_gate = lambda _: True
    node.vision.estimate = lambda *_args, **_kwargs: pytest.fail("gate bypassed")
    node._run_control_cycle()
    assert not commands


def test_real_node_stale_image_stops_not_republishes(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node._last_processed_sequence = 1
    node._latest_frame_time = 9.
    node._run_control_cycle()
    assert commands == [(0., 0.)]


def test_real_node_duplicate_fresh_frame_only_republishes(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node._last_processed_sequence = 1
    node.vision.estimate = lambda *_args, **_kwargs: pytest.fail("old image processed twice")
    node._run_control_cycle()
    assert commands == [(.5, .2)]


def test_real_node_slow_processing_stops(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    times = iter([10., 10.8])
    monkeypatch.setattr(node_module.time, "monotonic", lambda: next(times))
    node._run_control_cycle()
    assert commands == [(0., 0.)]


def test_real_node_exception_clears_controller_state(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node.patrol_controller.last_command = (.5, .3)
    def broken_cycle():
        raise RuntimeError("synthetic vision failure")
    node._run_control_cycle = broken_cycle
    node._control_timer()
    assert commands == [(0., 0.)]
    assert node.patrol_controller.last_command == (0., 0.)


def test_blue_island_mask_and_path_remain_forbidden():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (255, 0, 0)
    cv2.ellipse(image, (160, 160), (150, 150), 0, 0, 360, (245, 245, 245), -1)
    cv2.circle(image, (160, 150), 40, (255, 0, 0), -1)
    tuning = PatrolTuning()
    result = road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), tuning)
    assert result.valid
    assert result.mask[150-int(240*tuning.roi_top_ratio), 160] == 0
    xy = np.round(result.points).astype(int)
    assert not np.any((image[xy[:, 1], xy[:, 0]] == (255, 0, 0)).all(axis=1))
    drawn = np.zeros((240, 320), np.uint8)
    cv2.polylines(drawn, [xy], False, 255, 1)
    blue = (image == (255, 0, 0)).all(axis=2)
    assert not np.any((drawn > 0) & blue)


@pytest.mark.parametrize("seed,expected_side", [(.30, "left"), (.70, "right")])
def test_connected_two_channel_road_never_averages_across_blue(seed, expected_side):
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (255, 0, 0)
    image[:, 50:130] = 245
    image[:, 190:270] = 245
    image[205:, 50:270] = 245  # channels connect near the car
    previous = np.array([[seed*320, 0], [seed*320, 239]])
    result = road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), PatrolTuning(), seed, previous)
    assert result.valid
    upper = result.points[result.points[:, 1] < 200]
    assert len(upper) > 20
    if expected_side == "left":
        assert np.all(upper[:, 0] < 130)
    else:
        assert np.all(upper[:, 0] >= 190)


def test_thin_blue_separator_cannot_be_filled_like_black_paint():
    image = np.full((240, 320, 3), (255, 0, 0), np.uint8)
    image[:, 60:260] = 245
    image[100:180, 158:162] = (255, 0, 0)
    result = road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), replace(PatrolTuning(), close_kernel_px=9, black_gap_fill_px=40))
    assert result.mask[120-60, 160] == 0


def test_nested_white_island_is_not_joined_to_near_road_component():
    image = np.full((240, 320, 3), (255, 0, 0), np.uint8)
    cv2.ellipse(image, (160, 160), (150, 150), 0, 0, 360, (245, 245, 245), -1)
    cv2.circle(image, (160, 150), 45, (255, 0, 0), -1)
    cv2.circle(image, (160, 150), 15, (245, 245, 245), -1)
    result = road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), PatrolTuning())
    assert result.valid and result.mask[150-60, 160] == 0


def test_black_and_road_share_identical_full_image_geometry():
    tuning = replace(PatrolTuning(), road_history_weight=0)
    image = np.full((240, 320, 3), (255, 0, 0), np.uint8)
    image[:, 80:181] = 245  # exact row center x=130
    road = road_centerline(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), tuning, 130/320)
    black_angle = center_error(np.array([[130, 60], [130, 120], [130, 180]]), 320, 240, tuning)
    assert road.valid
    assert road.error == pytest.approx(black_angle, abs=1e-9)


def test_sparse_and_dense_equivalent_paths_have_same_row_weighting():
    tuning = PatrolTuning()
    sparse = np.array([[110, 60], [150, 120], [130, 180]])
    ys = np.arange(60, 181)
    dense = np.column_stack((np.interp(ys, sparse[:, 1], sparse[:, 0]), ys))
    assert center_error(sparse, 320, 240, tuning) == pytest.approx(center_error(dense, 320, 240, tuning))


def test_many_single_frame_dropouts_never_chop_or_restart_speed():
    controller, tuning = FixedSpeedController(), replace(PatrolTuning(),reacquire_confirm_frames=1)
    for now in np.arange(0, 2., .02):
        controller.command(PathEstimate(True, -.1, 1), now, tuning)
    for index, now in enumerate(np.arange(2., 5., .02)):
        path = PathEstimate(index % 2 == 0, -.1, 1)
        speed, _, mode = controller.command(path, now, tuning)
        assert speed == pytest.approx(tuning.fixed_speed_m_s)
        assert mode in ("fixed_speed", "path_memory_bridge")


def test_loss_holds_actual_turn_not_unreached_requested_turn():
    controller, tuning = FixedSpeedController(), replace(PatrolTuning(),search_enabled=0,maintain_cruise_during_search=0)
    controller.command(PathEstimate(True, -1, 1), 1., tuning)
    before = controller.command(PathEstimate(True, -1, 1), 1.05, tuning)
    lost = controller.command(PathEstimate(False), 1.10, tuning)
    assert before[1] < tuning.max_turn_rad_s
    assert lost[:2] == pytest.approx(before[:2])  # no blind acceleration, either


def test_repeated_invalid_frames_do_not_extend_memory_deadline():
    controller, tuning = FixedSpeedController(), replace(PatrolTuning(),search_max_seconds=.8,search_degraded_enabled=0)
    for now in np.arange(0, 2.01, .05):
        controller.command(PathEstimate(True, -.1, 1), now, tuning)
    for now in [2.1, 2.2, 2.4, 2.79]:
        assert controller.command(PathEstimate(False), now, tuning)[2] == "path_memory_bridge"
    assert controller.command(PathEstimate(False), 2.81, tuning)[:2] == (0., 0.)


def test_zero_memory_window_still_allows_explicit_final_protection():
    controller, tuning = FixedSpeedController(), replace(PatrolTuning(), loss_hold_seconds=0,search_enabled=0)
    controller.command(PathEstimate(True, 0, 1), 1., tuning)
    assert controller.command(PathEstimate(False), 1.01, tuning)[2] == "localization_fault_latched"


def test_reducing_turn_limit_is_not_delayed_by_previous_command():
    controller, tuning = FixedSpeedController(), PatrolTuning()
    for now in np.arange(0, 2.01, .05):
        controller.command(PathEstimate(True, -1, 1), now, tuning)
    changed = replace(tuning, max_turn_rad_s=.3)
    assert abs(controller.command(PathEstimate(True, -1, 1), 2.05, changed)[1]) <= .3


def test_hot_speed_save_keeps_vision_history(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    updated = replace(node.patrol_tuning, fixed_speed_m_s=.8)
    node.patrol_watcher = types.SimpleNamespace(poll=lambda: (updated, "loaded"))
    node.vision.reset_path = lambda: pytest.fail("speed-only save discarded path")
    node._set_patrol_posture = lambda: pytest.fail("speed-only save restarted pose transition")
    node._poll_live_tuning_file()
    assert node.patrol_tuning.fixed_speed_m_s == .8 and not commands


def test_true_camera_fault_bypasses_nonempty_path_memory(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node.patrol_controller.last_valid_at = 10.
    node.patrol_controller.last_command = (.5, 1.)
    node._camera_healthy = False
    node._run_control_cycle()
    assert commands == [(0., 0.)]
    assert node.patrol_controller.last_valid_at is None


def test_launcher_has_no_depth_or_step_nodes():
    launch = (SCRIPTS.parent / "launch" / "patrol_test.launch.py").read_text(encoding="utf-8")
    assert 'default_value="false"' in launch
    assert "step_detector_node" not in launch and "astra_camera_node" not in launch
    assert '"runtime.require_stand_confirmation:=true"' in launch
    assert '"DISPLAY": ":0"' in launch


@pytest.mark.parametrize("black_x,source,sign", [(None, "road", 0), (130, "black", -1), (160, "black", 0), (190, "black", 1)])
def test_actual_baseline_black_extractor_and_new_fallback(monkeypatch, black_x, source, sign):
    actual_node = load_actual_vision_module(monkeypatch)
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[:] = (255, 0, 0)
    frame[:, 110:530] = 245
    if black_x is not None:
        cv2.line(frame, (black_x*2, 0), (black_x*2, 479), (0, 0, 0), 12)
    vision = actual_node.PatrolVision()
    result = vision.estimate(frame, PatrolTuning(), 1., draw=True)
    assert result.valid and vision.source == source
    assert vision.last_debug.shape == frame.shape
    if sign:
        assert result.error * sign > 0
    else:
        assert abs(result.error) < .02


def load_actual_vision_module(monkeypatch):
    # Use the actual icn.py masks/extractor. Only unavailable ROS imports are
    # stubbed; the tested image-processing implementation is not mocked.
    stubs = {
        "rclpy": {}, "geometry_msgs.msg": {"Twist": object},
        "nav_msgs.msg": {"Odometry": object},
        "rcl_interfaces.msg": {"SetParametersResult": object},
        "rclpy.node": {"Node": object},
        "rclpy.signals": {"SignalHandlerOptions": types.SimpleNamespace(NO=0)},
        "sensor_msgs.msg": {"Imu": object, "JointState": object},
        "std_msgs.msg": {"Bool": object, "Float32MultiArray": object, "String": object},
        "wheel_robot.msg": {"StepDetection": object},
    }
    for name, attrs in stubs.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("icn", SCRIPTS / "icn.py")
    actual_base = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, "icn", actual_base)
    spec.loader.exec_module(actual_base)
    node_spec = importlib.util.spec_from_file_location("patrol_real_vision", SCRIPTS / "patrol_contour_test.py")
    actual_node = importlib.util.module_from_spec(node_spec)
    node_spec.loader.exec_module(actual_node)
    return actual_node


def test_expired_path_history_is_not_extended_by_invalid_frames(monkeypatch):
    actual_node = load_actual_vision_module(monkeypatch)
    vision = actual_node.PatrolVision()
    tuning = replace(PatrolTuning(), black_enabled=0)
    visible = cv2.cvtColor(scene(), cv2.COLOR_HSV2BGR)
    invisible = np.full((240, 320, 3), 255, np.uint8)
    assert vision.estimate(visible, tuning, 1.).valid
    assert vision.previous_points is not None
    assert not vision.estimate(invisible, tuning, 1.2).valid
    assert vision.last_path_at == 1.
    assert not vision.estimate(invisible, tuning, 1.81).valid
    assert vision.last_path_at is None and vision.previous_points is None


def test_manual_stop_cancels_path_memory_and_rearms_standing_gate(node_module, monkeypatch):
    node, commands = make_node(node_module, monkeypatch)
    node_module.base.STAND_SETTLE_SECONDS = 2.
    node_module.base.StandGate = lambda **kw: types.SimpleNamespace(**kw)
    reset = []
    node.vision.reset_path = lambda: reset.append(True)
    node.patrol_controller.last_valid_at = 10.
    node.patrol_controller.last_command = (.5, 1.)
    node._on_patrol_stop(types.SimpleNamespace(data=True))
    assert commands == [(0., 0.)] and reset == [True]
    assert node._stand_gate.enabled and node._stand_gate.settle_seconds == 2.
    assert node.patrol_controller.last_valid_at is None


def test_invalid_short_save_never_applies_deleted_loss_speed_ratio(tmp_path):
    path = tmp_path / "patrol_test.yaml"
    old = {**asdict(PatrolTuning()), "loss_speed_ratio": .4}
    path.write_text(yaml.safe_dump({"patrol_test": old}), encoding="utf-8")
    tuning, reason = PatrolFileWatcher(path).poll()
    assert tuning is None and "loss_speed_ratio" in reason


@pytest.mark.parametrize("changes", [{"black_gap_fill_px": 41}, {"road_max_row_step_ratio": 0}, {"road_history_weight": 1.1}])
def test_new_contour_parameters_are_validated(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(), **changes).validate()


def test_v3_handoff_is_bounded_when_new_error_changes_sign():
    selector, tuning = PathSelector(), PatrolTuning()
    absent = PathEstimate(False)
    selector.select(PathEstimate(True, -.4, 1), absent, 0., tuning)
    selector.select(absent, PathEstimate(True, .4, 1), 1., tuning)
    result, _ = selector.select(absent, PathEstimate(True, -.2, 1), 1.03, tuning)
    assert result.error == pytest.approx(-.36)  # V2 produced -.84
    assert -.4 <= result.error <= -.2
    for now, incoming in zip(np.linspace(1.04, 1.15, 20), np.linspace(.7, -.8, 20)):
        result, _ = selector.select(absent, PathEstimate(True, incoming, 1), now, tuning)
        assert min(-.4, incoming)-1e-12 <= result.error <= max(-.4, incoming)+1e-12


def test_near_single_band_rescue_confirms_opposite_jump_not_old_return_delay():
    selector, tuning = PathSelector(), PatrolTuning()
    selector.select(PathEstimate(False), PathEstimate(True, .4, 1), 1., tuning)
    rescue = PathEstimate(True, -.7, .31, rescue=True)
    chosen, source = selector.select(rescue, PathEstimate(True, .3, 1), 1.01, tuning)
    assert not chosen.valid and source == "none"  # V5 rejects a one-frame opposite flash
    chosen, source = selector.select(rescue, PathEstimate(True, .3, 1), 1.04, tuning)
    assert source == "black" and chosen.error == -.7 and not chosen.handoff


@pytest.mark.parametrize("cutoff,expected_bands", [(190, 2), (200, 2), (210, 1)])
def test_actual_near_only_black_remains_usable(monkeypatch, cutoff, expected_bands):
    module = load_actual_vision_module(monkeypatch)
    frame = np.full((240, 320, 3), (255, 0, 0), np.uint8)
    frame[:, 40:280] = 245
    cv2.line(frame, (70, cutoff), (70, 239), (0,0,0), 7)
    vision = module.PatrolVision()
    dark, _, hsv, _ = vision._make_dark_mask(frame)
    extracted = vision._extract_line(frame, dark, hsv, False, use_dynamic_offset=False)
    assert extracted[0] and extracted[6] == expected_bands
    vision = module.PatrolVision()
    result = vision.estimate(frame, PatrolTuning(), 1.)
    assert result.valid and vision.source == "black"
    assert result.error == pytest.approx(extracted[1])
    assert result.rescue == (expected_bands == 1)


def test_reset_path_clears_all_black_and_road_history(monkeypatch):
    module = load_actual_vision_module(monkeypatch)
    vision = module.PatrolVision()
    vision._last_good_band_x = [90.] * len(module.base.LOOKAHEAD_Y_RATIOS)
    vision._previous_band_x = [95.] * len(module.base.LOOKAHEAD_Y_RATIOS)
    vision.road_widths = np.array([[100,100],[100,220]])
    vision.last_dual_at = 1.
    vision.last_debug = np.zeros((2,2,3), np.uint8)
    vision.reset_path()
    assert all(x is None for x in vision._last_good_band_x)
    assert all(x is None for x in vision._previous_band_x)
    assert vision.road_widths is None and vision.last_dual_at is None
    assert vision.last_debug is None and vision.selector.source == "none"


@pytest.mark.parametrize("center", [.49, .50, .51])
@pytest.mark.parametrize("use_black", [True, False])
def test_center_parameter_is_shared_by_actual_black_and_road(monkeypatch, center, use_black):
    module = load_actual_vision_module(monkeypatch)
    frame = np.full((240,320,3), (255,0,0), np.uint8)
    frame[:,60:261] = 245
    if use_black:
        cv2.line(frame,(160,0),(160,239),(0,0,0),7)
    tuning = replace(PatrolTuning(), target_x_ratio=center)
    result = module.PatrolVision().estimate(frame, tuning, 1.)
    assert result.valid
    assert result.error == pytest.approx(-2*(center-.5), abs=.005)


def test_road_near_side_wins_over_opposite_far_curve():
    points = np.array([[50,80],[90,144],[210,207],[210,235]])
    error = center_error(points,320,240,PatrolTuning())
    assert error > .2  # do not turn left early just because far road is left


def test_road_width_single_edge_window_is_not_renewed(monkeypatch):
    module = load_actual_vision_module(monkeypatch)
    vision = module.PatrolVision()
    tuning = replace(PatrolTuning(), black_enabled=0)
    dual = np.full((240,320,3),(255,0,0),np.uint8)
    dual[:,60:261] = 245
    single = np.full((240,320,3),(255,0,0),np.uint8)
    single[:,:261] = 245
    assert vision.estimate(dual,tuning,1.).valid
    assert vision.estimate(single,tuning,1.1).valid
    assert vision.last_dual_at == 1.
    assert vision.estimate(single,tuning,1.3).valid
    assert not vision.estimate(single,tuning,1.41).valid
    assert vision.last_dual_at == 1.


def test_spatially_unrelated_road_cannot_take_over_recent_black_path(monkeypatch):
    module = load_actual_vision_module(monkeypatch)
    vision = module.PatrolVision()
    tuning = replace(PatrolTuning(), black_enabled=0, road_max_near_jump_ratio=.05)
    left = cv2.cvtColor(scene(130),cv2.COLOR_HSV2BGR)
    right = cv2.cvtColor(scene(180),cv2.COLOR_HSV2BGR)
    assert vision.estimate(left,tuning,1.).valid
    assert not vision.estimate(right,tuning,1.03).valid
    assert vision.road_estimate.reason == "road_near_disagrees_with_recent_path"


def test_pd_matches_baseline_default_coefficients_without_old_speed_caps():
    tuning = PatrolTuning()
    assert (tuning.steering_gain,tuning.steering_kd,tuning.error_filter_alpha,tuning.max_error_derivative) == (1.08,.018,.48,3.)
    controller = FixedSpeedController()
    controller.command(PathEstimate(True,.1,1,source="black"),1.,tuning)
    controller.command(PathEstimate(True,.2,1,source="black"),1.05,tuning)
    filtered = .48*.2+.52*.1
    assert controller.raw_angular == pytest.approx(-(1.08*filtered+.018*((filtered-.1)/.05)))


@pytest.mark.parametrize("kind", ["source_change", "reacquire", "handoff"])
def test_pd_has_no_derivative_kick_on_handoff_or_reacquisition(kind):
    controller = FixedSpeedController()
    tuning = replace(PatrolTuning(), steering_kd=100.)
    controller.command(PathEstimate(True,-.7,1,source="black"),1.,tuning)
    if kind == "reacquire":
        controller.command(PathEstimate(False),1.03,tuning)
    path = PathEstimate(True,.6,1,source="road" if kind == "source_change" else "black",handoff=kind == "handoff")
    controller.command(path,1.06,tuning)
    assert abs(controller.raw_angular) <= tuning.max_turn_rad_s


@pytest.mark.parametrize("changes", [{"target_x_ratio":.51},{"posture_pitch_deg":26.}])
def test_geometry_and_posture_saves_clear_history_without_speed_restart(node_module,monkeypatch,changes):
    node, commands = make_node(node_module,monkeypatch)
    node.patrol_controller.last_command = (.5,.2)
    node.patrol_controller.filtered_error = .2
    resets = []
    node.vision.reset_path = lambda: resets.append(True)
    node._set_patrol_posture = lambda: None
    node.patrol_watcher = types.SimpleNamespace(poll=lambda:(replace(node.patrol_tuning,**changes),"loaded"))
    node._poll_live_tuning_file()
    assert resets == [True] and node.patrol_controller.filtered_error is None
    assert node.patrol_controller.last_command[0] == .5 and not commands


@pytest.mark.parametrize("changes", [{"steering_kd":-1},{"error_filter_alpha":0},{"max_error_derivative":-1},{"road_near_y_ratio":.5},{"road_single_edge_hold_seconds":-1},{"road_max_near_jump_ratio":1.1}])
def test_v3_parameter_ranges(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_road_observations_cannot_renew_expired_black_template(monkeypatch):
    module = load_actual_vision_module(monkeypatch)
    frame = np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,60:261] = 245
    black = frame.copy()
    cv2.line(black,(160,0),(160,239),(0,0,0),7)
    vision = module.PatrolVision()
    tuning = PatrolTuning()
    assert vision.estimate(black,tuning,1.).valid
    assert any(x is not None for x in vision._last_good_band_x)
    for now in (1.1,1.5,1.81):
        assert vision.estimate(frame,tuning,now).valid
    assert vision.source == "road" and vision.last_path_at == 1.81
    assert vision.last_black_at is None
    assert all(x is None for x in vision._last_good_band_x)


def cruise_controller(error=-.5, **changes):
    controller, tuning = FixedSpeedController(), replace(PatrolTuning(), **changes)
    for now in np.arange(0, 2.01, 1/30):
        controller.command(PathEstimate(True,error,1,source="black"),now,tuning)
    return controller, tuning


def test_v4_no_stop_at_old_point_eight_second_deadline():
    c,t = cruise_controller()
    for now in np.arange(2.03,3.2,.03):
        v,w,mode=c.command(PathEstimate(False),now,t)
        assert v == pytest.approx(t.fixed_speed_m_s) and w > 0
        assert mode == "active_search"


def test_v4_single_reacquired_frame_never_memorizes_zero_speed():
    c,t=cruise_controller()
    c.command(PathEstimate(False),2.03,t)
    regained=c.command(PathEstimate(True,-.7,1,source="black"),2.86,t)
    lost=c.command(PathEstimate(False),2.90,t)
    assert regained[0] == lost[0] == pytest.approx(t.fixed_speed_m_s)
    assert regained[2] == "reacquire_confirm"
    assert lost[2] == "active_search"
    assert c.loss_started_at is not None and c.reacquire_streak == 0


def test_v4_flickers_cannot_reset_time_budget():
    c,t=cruise_controller(search_max_seconds=1.2,search_max_distance_m=10,search_degraded_enabled=0)
    for i,now in enumerate(np.arange(2.03,3.15,.03)):
        path=PathEstimate(i%3!=0,-.5,1,source="black")
        assert c.command(path,now,t)[0] == pytest.approx(t.fixed_speed_m_s)
    assert c.command(PathEstimate(False),3.21,t)[2] == "localization_fault_latched"
    assert c.fault_reason == "time_limit"


def test_v4_distance_budget_and_fault_do_not_auto_restart():
    c,t=cruise_controller(search_max_distance_m=.20,search_max_seconds=10,search_degraded_enabled=0)
    for now in [2.1,2.2,2.3]:
        assert c.command(PathEstimate(False),now,t)[0] == pytest.approx(t.fixed_speed_m_s)
    assert c.command(PathEstimate(False),2.5,t)[2] == "localization_fault_latched"
    assert c.fault_reason == "distance_limit"
    for now in [2.6,2.7,3.0]:
        assert c.command(PathEstimate(True,0,1),now,t) == (0.,0.,"localization_fault_latched")
    c.reset()  # operator gate is responsible for authorizing this reset
    assert c.command(PathEstimate(True,0,1),3.1,t)[0] > 0


def test_v4_consecutive_new_observations_restore_tracking_without_ramp_restart():
    c,t=cruise_controller()
    c.command(PathEstimate(False),2.1,t)
    for i,now in enumerate([2.2,2.23,2.26]):
        v,w,mode=c.command(PathEstimate(True,-.3,1,source="black"),now,t)
        assert v == pytest.approx(t.fixed_speed_m_s)
        assert mode == ("fixed_speed" if i==2 else "reacquire_confirm")
    assert c.loss_started_at is None and c.loss_distance == 0


@pytest.mark.parametrize("error,sign", [(-.875,1),(.875,-1)])
def test_v4_large_error_can_use_configured_turn_not_just_one_point_one(error,sign):
    c,t=cruise_controller(error)
    assert c.raw_angular == pytest.approx(sign*t.max_turn_rad_s)
    assert c.last_command[1] == pytest.approx(sign*t.max_turn_rad_s)
    assert c.last_command[0] == pytest.approx(t.fixed_speed_m_s)


def test_v4_sharp_reversal_does_not_wait_for_normal_slow_slew():
    c,t=cruise_controller(-.9)
    for now in np.arange(2.03,2.28,.03):
        c.command(PathEstimate(True,.9,1,source="black",rescue=True),now,t)
    assert c.last_command[0] == pytest.approx(t.fixed_speed_m_s)
    assert c.last_command[1] < 0


def test_v4_partial_near_road_uses_real_observed_boundary_only():
    image=np.full((240,320,3),(255,0,0),np.uint8)
    image[90:201,80:241]=245
    image[201:] = (30,30,30)  # neutral occlusion, NOT known blue forbidden ground
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    old=road_centerline(hsv,replace(PatrolTuning(),road_near_gap_ratio=0))
    new=road_centerline(hsv,PatrolTuning())
    assert not old.valid and new.valid
    assert new.near_gap_ratio > 0 and not new.single_edge
    assert np.max(new.points[:,1]) <= 200  # not an invented extension to row206
    assert abs(new.error) < .02


def test_v4_partial_road_does_not_allow_remote_white_island_or_all_white():
    image=np.full((240,320,3),(255,0,0),np.uint8)
    image[60:130,80:241]=245
    assert not road_centerline(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),PatrolTuning()).valid
    assert not road_centerline(np.full((240,320,3),(0,0,255),np.uint8),PatrolTuning()).valid


def test_v4_node_lost_localization_returns_to_manual_standing_gate(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    node_module.base.STAND_SETTLE_SECONDS=2.
    node_module.base.StandGate=lambda **kw:types.SimpleNamespace(**kw)
    node.vision.reset_path=lambda:None
    node.vision.estimate=lambda *_args,**_kw:PathEstimate(False)
    node.patrol_controller.fault_latched=True
    node.patrol_controller.fault_reason="time_limit"
    node._run_control_cycle()
    assert commands and all(cmd == (0.,0.) for cmd in commands)
    assert node._stand_gate.enabled and node._start_cue_until == 0


@pytest.mark.parametrize("changes", [
    {"corner_start_error":.8},{"corner_full_error":1.1},{"corner_angular_accel_rad_s2":0},
    {"search_enabled":2},{"search_max_seconds":0},{"search_max_distance_m":-1},
    {"search_turn_rad_s":-1},{"search_default_direction":0},{"reacquire_confirm_frames":1.5},
    {"road_near_gap_ratio":.4},
])
def test_v4_parameters_are_finite_and_consistent(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v4_search_direction_does_not_flip_when_curve_timestamp_ages():
    c,t=cruise_controller(.5,search_max_seconds=3.,search_max_distance_m=10.)
    for now in np.arange(2.03,2.81,.03):
        c.command(PathEstimate(True,0,1,source="black"),now,t)
    for now in [3.1,4.,5.2]:
        v,w,mode=c.command(PathEstimate(False),now,t)
        assert mode == "active_search" and v == pytest.approx(t.fixed_speed_m_s) and w < 0


def test_v4_partial_near_gap_never_crosses_known_blue_foreground():
    image=np.full((240,320,3),(255,0,0),np.uint8)
    image[90:201,80:241]=245
    result=road_centerline(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),PatrolTuning())
    assert not result.valid and result.reason == "blue_blocks_near_gap"


def test_v5_unwinds_old_corner_without_chopping_forward_speed():
    c, t = cruise_controller(-.8)
    for now in np.arange(2.033333, 2.234, 1/30):
        v, w, _ = c.command(PathEstimate(True, 0., .9, source="black"), now, t)
        assert v == pytest.approx(t.fixed_speed_m_s) and w >= 0
    assert abs(w) < .1  # V4 still had about 1.3rad/s at this point


def test_v5_unwind_never_spends_braking_budget_on_reverse_acceleration():
    c, t = cruise_controller(-.9, unwind_angular_accel_rad_s2=100.)
    _, w, _ = c.command(PathEstimate(True, .9, .9, source="black"), 2.1, t)
    assert w == 0  # not instant -2 from leftover braking allowance
    _, w, _ = c.command(PathEstimate(True, .9, .9, source="black"), 2.13, t)
    assert -.31 <= w < 0


def test_v5_low_confidence_rescue_retains_tracking_not_full_corner():
    c, t = cruise_controller(0.)
    for now in np.arange(2.03, 2.8, .03):
        v, w, _ = c.command(PathEstimate(True, -.85, .31, source="black", rescue=True), now, t)
    assert v == pytest.approx(t.fixed_speed_m_s) and .5 < w < 1.8


def test_v5_single_opposite_path_flash_needs_consistent_confirmation():
    s, t = PathSelector(), PatrolTuning()
    absent = PathEstimate(False)
    s.select(PathEstimate(True, -.6, .9), absent, 0., t)
    assert not s.select(PathEstimate(True, .6, .9), absent, .03, t)[0].valid
    chosen, _ = s.select(PathEstimate(True, .61, .9), absent, .06, t)
    assert chosen.valid and chosen.error == .61


def test_v5_task_branch_can_intentionally_change_error_without_flash_gate():
    s, t = PathSelector(), PatrolTuning()
    s.select(PathEstimate(True, .7, .9), PathEstimate(False), 0., t)
    result, _ = s.select(PathEstimate(True, -.7, .9, route_locked=True), PathEstimate(False), .03, t)
    assert result.valid and result.error == -.7


def test_v5_weak_blue_and_warm_white_are_not_confused():
    hsv = np.full((240,320,3), (94,42,120), np.uint8)
    hsv[:,80:241] = (21,100,184)
    blue, _ = road_masks(hsv, PatrolTuning())
    assert np.mean(blue[:, :70] > 0) > .99
    assert np.mean(blue[:,100:220] > 0) == 0
    road = road_centerline(hsv, PatrolTuning())
    assert road.valid and abs(road.error) < .03


def test_v5_gray_glare_inside_blue_is_not_a_new_road():
    hsv = np.full((240,320,3), (94,42,90), np.uint8)
    hsv[:,140:181] = (0,0,105)
    assert road_centerline(hsv, replace(PatrolTuning(), road_blue_contrast_gray=0)).valid
    assert not road_centerline(hsv, PatrolTuning()).valid


def fork_scene():
    image = np.full((240,320,3), (255,0,0), np.uint8)
    image[:,40:281] = 245
    image[:161,145:176] = (255,0,0)
    return image


def test_v5_left_fork_selection_does_not_average_across_island():
    hsv = cv2.cvtColor(fork_scene(), cv2.COLOR_BGR2HSV)
    left = road_centerline(hsv, PatrolTuning(), seed_x_ratio=.64, branch="left")
    assert left.valid and left.route_locked and left.branch_rows
    assert left.mid_error < -.25 and left.near_error > -.1
    blue, _ = road_masks(hsv, PatrolTuning())
    points = np.rint(left.points).astype(int)
    assert np.all(blue[points[:,1],points[:,0]] == 0)


def test_v5_left_task_rejects_right_black_even_if_it_is_confident(monkeypatch):
    module = load_actual_vision_module(monkeypatch)
    vision = module.PatrolVision()
    vision.router._phase("ENTRY", 0.)
    frame = fork_scene()
    cv2.line(frame, (225,0), (225,239), (0,0,0), 7)
    path = vision.estimate(frame, PatrolTuning(), .1)
    assert path.valid and vision.source == "road"
    assert path.error < 0 and path.route_locked
    assert not vision.black_estimate.valid
    assert vision.black_estimate.reason == "black_outside_locked_route"


def test_v5_left_route_still_allows_right_correction(monkeypatch):
    module = load_actual_vision_module(monkeypatch)
    vision = module.PatrolVision()
    vision.router._phase("EXIT", 0.)
    vision.router.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    vision.router.branch_acquired = True
    frame = np.full((240,320,3), (255,0,0), np.uint8)
    frame[:,130:281] = 245
    path = vision.estimate(frame, PatrolTuning(), .1)
    assert path.valid and path.route_locked and path.error > .1
    c = FixedSpeedController()
    assert c.command(path,.1,PatrolTuning(),search_direction=1)[1] < 0


def vv_test_cue(entry=False):
    """State-only cue; actual image geometry is tested in test_patrol_vv_v16."""
    from patrol_dash_entry import DashObservation
    return DashObservation(valid=True,blocks=4,branch=PathEstimate(entry,route_locked=entry))


def enter_router():
    t = replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    r = RoundaboutRouter()
    from patrol_dash_entry import DashObservation
    # These older tests isolate post-entry acquisition/exit contracts. Enter
    # APPROACH through the actual VV interface, not retired raw score evidence.
    cue=DashObservation(valid=True,blocks=4)
    for now in (0.,.03,.06):
        r.observe(True,.9,now,t,(.5,0.),dash_cue=cue)
    assert r.phase == "APPROACH"
    r._phase("ENTRY",.06)  # remaining tests isolate post-approach ring stages
    return r,t


def test_v5_gate_evidence_is_phase_gated_not_any_three_dark_blocks():
    r,t = RoundaboutRouter(),PatrolTuning()
    for i in range(20):
        r.observe(True,.95,i*.03,t)
    assert r.phase == "SEEK" and r.evidence == 0
    for i in range(20,220):
        r.observe(True,.95,i*.03,t,(.5,0.))
    assert r.phase=="SEEK"  # retired raw scores cannot bypass VV
    r.observe(True,1.,6.6,t,(.5,0.),dash_cue=vv_test_cue())
    assert r.phase == "APPROACH"


def test_v5_entry_requires_travel_turn_and_stable_locked_path():
    r,t = enter_router()
    for i in range(1,12):
        now=.06+i*.1
        r.observe(True,.95,now,t,(.5,0.))
        r.accept_path(PathEstimate(True, route_locked=True, route_observed=True),now,t)
    assert r.phase == "ENTRY"  # no left yaw progress
    for i in range(12,27):  # V13 requires >=55 degrees THEN stable path observations
        now=.06+i*.1
        r.observe(True,.95,now,t,(.5,1.))
        r.accept_path(PathEstimate(True, route_locked=True, route_observed=True),now,t)
    assert r.phase == "FOLLOW"


def test_v5_exit_needs_gate_clear_reappearance_and_minimum_ring_time():
    r,t = enter_router()
    r._phase("FOLLOW",1.)
    r.ring_yaw=2.
    r.phase_distance=1.
    for now in np.arange(1.1,10.,.1):
        r.observe(True,.95,now,t,(.5,.5))
    assert r.phase == "FOLLOW"  # original gate never disappeared
    for now in (10.,10.1,10.2):
        r.observe(False,0.,now,t,(.5,.5))
    for now in (10.3,10.4,10.5):
        r.observe(True,.95,now,t,(.5,.5))
    assert r.phase == "EXIT"


def test_v5_timeout_is_not_assumed_exit_and_never_moves_again_by_itself():
    r,t = enter_router()
    t=replace(t,roundabout_timeout_enabled=0)  # retain the explicit legacy mode
    r.observe(False,0.,11.,t)
    assert r.fault and r.phase == "ENTRY"
    r.accept_path(PathEstimate(True,route_locked=True),12.,t)
    assert r.phase == "ENTRY"
    r._phase("FOLLOW",20.)
    r.fault = ""
    r.observe(False,0.,49.,t)
    assert r.fault and r.phase == "FOLLOW"


def test_v5_exit_reacquires_selected_path_before_unlock_and_cooldown():
    r,t = enter_router()
    r._phase("EXIT",1.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    r.last_time=1.
    for now in np.arange(1.1,2.2,.1):
        r.observe(False,0.,now,t,(.5,0.))
        r.accept_path(PathEstimate(True,route_locked=True,route_observed=True),now,t)
    assert r.phase == "SEEK_CROSSWALK" and r.completed == 1
    for now in np.arange(2.2,20.,.1):
        r.observe(True,.95,now,t,(.5,0.))
    assert r.phase == "SEEK_CROSSWALK"  # exit marker cannot become a new entrance


def test_v5_fresh_stationary_odom_does_not_gain_command_distance():
    r,t = RoundaboutRouter(),PatrolTuning()
    for now in np.arange(0,4.,.03):
        r.observe(True,.95,now,t,(.5,1.),(0.,0.,0.))
    assert r.phase == "SEEK" and r.distance == 0 and r.left_yaw == 0
    assert r.progress_source == "odom"


def test_v5_locked_lost_path_searches_left_not_old_right():
    c,t = cruise_controller(.6)
    _,w,mode=c.command(PathEstimate(False),2.03,t,search_direction=1)
    assert mode == "active_search" and c.raw_angular > 0
    for now in np.arange(2.06,2.4,.03):
        _,w,_=c.command(PathEstimate(False),now,t,search_direction=1)
    assert w > 0 and c.last_command[0] == pytest.approx(t.fixed_speed_m_s)


@pytest.mark.parametrize("changes", [
    {"unwind_angular_accel_rad_s2":0}, {"rescue_corner_confidence":0},
    {"path_jump_confirm_frames":1.5}, {"path_jump_error":0},
    {"blue_weak_s_min":256}, {"road_blue_contrast_gray":-1},
    {"blue_context_radius_ratio":0}, {"roundabout_enabled":2},
    {"roundabout_gate_weak_score":.99}, {"roundabout_gate_evidence":6},
    {"roundabout_follow_min_seconds":29}, {"roundabout_heading_gain":1.1},
    {"roundabout_path_confirm_frames":0}, {"roundabout_route_max_seconds":.1},
])
def test_v5_new_parameters_reject_invalid_complete_snapshot(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v5_exit_lock_is_not_released_while_fork_is_visible():
    r,t=enter_router()
    r._phase("EXIT",1.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    r.last_time=1.
    for now in np.arange(1.1,3.,.1):
        r.observe(False,0.,now,t,(.5,.5))
        r.accept_path(PathEstimate(True,route_locked=True,branch_rows=2),now,t)
    assert r.phase == "EXIT" and r.completed == 0


def test_v16_vv_replaces_old_entry_extractor(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    vision=module.PatrolVision()
    frame=np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,50:271]=245
    for y in (85,110,135,160,185,210):
        cv2.rectangle(frame,(171,y-5),(179,y+5),(0,0,0),-1)
    tuning=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    vision._detect_roundabout_gate=lambda *_:pytest.fail("retired detector called")
    vision.motion_command=(.65,0.)
    for now in (0.,.03,.06):
        vision.estimate(frame,tuning,now)
    assert vision.gate_score > .82 and vision.router.phase == "APPROACH"


def test_v5_node_route_fault_stops_and_rearms_operator_gate(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    node_module.base.STAND_SETTLE_SECONDS=2.
    node_module.base.StandGate=lambda **kw:types.SimpleNamespace(**kw)
    router=RoundaboutRouter()
    router.fault="roundabout_route_timeout"
    node.vision.router=router
    node.vision.reset_path=lambda:None
    node._run_control_cycle()
    assert commands == [(0.,0.)]
    assert node._stand_gate.enabled


def test_v6_manual_stop_preserves_route_and_does_not_reset_course(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    node_module.base.STAND_SETTLE_SECONDS=2.
    node_module.base.StandGate=lambda **kw:types.SimpleNamespace(**kw)
    reset=[]
    node.vision.reset_path=lambda:None
    node.vision.reset_course=lambda:reset.append(True)
    node.vision.router=RoundaboutRouter()
    node.vision.router._phase("SEEK_CROSSWALK",0.)
    node.vision.router.completed=1
    node._on_patrol_stop(types.SimpleNamespace(data=True))
    assert not reset and commands == [(0.,0.)]
    assert node.vision.router.phase=="SEEK_CROSSWALK" and node.vision.router.completed==1
    assert node.vision.router.paused_at is not None


def test_v5_disabling_ring_mid_task_rejects_whole_live_save(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    router,_=enter_router()
    node.vision.router=router
    old=node.patrol_tuning
    node.patrol_watcher=types.SimpleNamespace(poll=lambda:(replace(old,roundabout_enabled=0,fixed_speed_m_s=.8),"loaded"), signature="saved")
    node._poll_live_tuning_file()
    assert node.patrol_tuning == old and not commands
    assert node.patrol_watcher.signature is None


def test_v5_left_right_oscillation_does_not_fake_net_entry_turn():
    r,t=enter_router()
    for i in range(1,31):
        now=.06+i*.1
        r.observe(False,0.,now,t,(.5,1. if i%2 else -1.))
        r.accept_path(PathEstimate(True,route_locked=True),now,t)
    assert abs(r.left_yaw) < .11 and r.phase == "ENTRY"


def test_v6_far_gate_only_approaches_does_not_turn_early():
    r=RoundaboutRouter()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    for now in np.arange(0.,1.5,.03):
        r.observe(True,.95,now,t,(.5,0.),gate_near=.45,dash_cue=vv_test_cue())
    assert r.phase=="APPROACH" and not r.left_lock
    for now in (1.5,1.53,1.56):
        r.observe(True,.95,now,t,(.5,0.),gate_near=.75,dash_cue=vv_test_cue(entry=True))
    assert r.phase=="ENTRY" and r.left_lock


def test_v7_first_six_seconds_skip_actual_gate_extractor(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    t=PatrolTuning()
    v.router.started_at=10.
    v.router.distance=2.
    calls=[]
    from patrol_dash_entry import DashObservation
    monkeypatch.setattr(module,"detect_vv_dashes",lambda *_:(calls.append(1) or DashObservation()))
    frame=cv2.cvtColor(scene(),cv2.COLOR_HSV2BGR)
    for now in (10.,12.,15.99):
        v.estimate(frame,t,now)
    assert not calls and v.router.evidence==0 and v.router.phase=="SEEK"
    v.estimate(frame,t,16.)
    assert len(calls)==1


def test_v16_missing_vv_cue_cannot_launch_entry_but_new_cue_can():
    r,t=RoundaboutRouter(),replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    for i in range(15):
        r.observe(False,0.,i*.1,t,(.5,0.),dash_cue=vv_test_cue())
    assert r.phase=="APPROACH"
    r.observe(False,0.,1.5,t,(.5,0.))
    assert r.phase=="APPROACH"
    r.observe(False,0.,1.6,t,(.5,0.),dash_cue=vv_test_cue(entry=True))
    assert r.phase=="ENTRY"


def test_v7_near_evidence_expires_and_pause_clears_it():
    r=RoundaboutRouter()
    t=PatrolTuning()
    r._phase("APPROACH",0.)
    r.observe(False,.8,.03,t,(.5,0.),gate_near=.75,entry_ready=True)
    r.observe(False,.8,.06,t,(.5,0.),gate_near=.75,entry_ready=True)
    r.observe(False,0.,.5,t,(.5,0.))
    assert r.phase=="APPROACH" and r.near_frames==0
    r.observe(False,.8,.53,t,(.5,0.),gate_near=.75,entry_ready=True)
    r.pause(.54)
    assert r.near_frames==0 and r.near_seen_at is None


def test_v7_left_entry_cancels_stale_right_command_without_dropping_cruise():
    r=RoundaboutRouter()
    t=PatrolTuning()
    r._phase("ENTRY",1.)
    c=FixedSpeedController()
    c.last_command=(t.fixed_speed_m_s,-1.8)
    c.last_time=1.
    v,w,mode=r.motion_override(1.03,t)
    assert v==t.fixed_speed_m_s and w>0 and mode=="roundabout_left_acquire"
    for i in range(1,6):
        output=c.task_command(v,w,1.+i*.03,t,left_only=True)
        assert output[0]==t.fixed_speed_m_s and output[1]>=0
    assert output[1]>0
    r.left_yaw=math.radians(t.roundabout_entry_acquire_yaw_deg)
    r.branch_acquired=True  # an observed owned route, not angle alone
    assert r.motion_override(1.2,t) is None  # real route may correct right
    r._phase("APPROACH",1.3)
    assert r.motion_override(1.3,t) is None


def test_v7_actual_node_entry_overrides_right_path_not_normal_patrol(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter()
    r._phase("ENTRY",9.9)
    node.vision.router=r
    node.patrol_controller.last_command=(node.patrol_tuning.fixed_speed_m_s,-1.5)
    node.patrol_controller.last_time=9.97
    node.vision.estimate=lambda *_args,**_kwargs: PathEstimate(True,.6,.9)
    node._run_control_cycle()
    assert commands[-1]==(node.patrol_tuning.fixed_speed_m_s,0.)  # cancel stale right
    assert node.patrol_controller.last_command==commands[-1]
    r._phase("SEEK",10.)
    monkeypatch.setattr(node_module.time,"monotonic",lambda:10.03)
    node._frame_sequence+=1
    node._run_control_cycle()
    assert commands[-1][1]<0  # ordinary right correction not globally reversed


def test_v7_zebra_preview_does_not_stop_before_ring_exit(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    frame,_,_=zebra_frame()
    v.estimate(frame,PatrolTuning(),1.,draw=True)
    assert v.crosswalk_score>.8
    assert v.router.crosswalk_block_reason(1.,PatrolTuning())=="lap1_ring_not_confirmed"
    assert v.router.crosswalk_count==0 and v.router.phase=="SEEK"


@pytest.mark.parametrize("changes",[
    {"roundabout_entry_confirm_frames":0},
    {"roundabout_entry_confirm_frames":1.5},
    {"roundabout_entry_near_hold_seconds":0},
])
def test_v7_near_gate_parameters_are_validated(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v6_near_gate_without_approach_travel_cannot_turn():
    r=RoundaboutRouter()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    for now in np.arange(0.,2.,.03):
        r.observe(True,.95,now,t,(0.,0.),gate_near=.95,dash_cue=vv_test_cue(entry=True))
    assert r.phase=="SEEK" and r.phase_distance==0.


def test_v6_false_approach_times_out_to_normal_not_blind_left():
    r=RoundaboutRouter()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    for now in (0.,.03,.06):
        r.observe(True,.95,now,t)
    r.observe(False,0.,9.,t,(.5,0.),gate_near=0.,entry_ready=True)
    assert r.phase=="SEEK" and not r.fault and not r.left_lock
    for now in (9.03,9.06,9.09):
        r.observe(True,.95,now,t,(.5,0.),gate_near=.9,entry_ready=True)
    assert r.phase=="SEEK"


def test_v6_approach_preserves_straight_black_line_not_left_filter(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    v.router._phase("APPROACH",0.)
    frame=np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,60:261]=245
    cv2.line(frame,(180,0),(180,239),(0,0,0),7)
    p=v.estimate(frame,PatrolTuning(),.1)
    assert p.valid and v.source=="black" and p.error>0 and not v.router.left_lock


def test_v6_ring_follow_does_not_require_contour_to_accept_real_black(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    v.router._phase("FOLLOW",0.)
    frame=np.full((240,320,3),245,np.uint8)
    cv2.line(frame,(160,0),(160,239),(0,0,0),7)
    p=v.estimate(frame,PatrolTuning(),.1)
    assert p.valid and v.source=="black" and not v.road_estimate.valid


def test_v6_solid_exit_is_gated_by_progress_not_fixed_eight_seconds():
    r,t=enter_router()
    r._phase("FOLLOW",1.)
    r.ring_yaw=2.
    r.phase_distance=1.
    for now in (3.1,3.2,3.3,3.4,3.5):
        r.observe(False,0.,now,t,(.5,.1),exit_solid=True)
    assert r.phase=="EXIT"


def test_v6_solid_entry_line_cannot_pretend_to_be_exit_before_net_turn():
    r,t=enter_router()
    r._phase("FOLLOW",1.)
    r.phase_distance=1.
    for now in np.arange(1.1,6.,.1):
        r.observe(False,0.,now,t,(.5,0.),exit_solid=True)
    assert r.phase=="FOLLOW"


def zebra_frame(stripes=6):
    frame=np.full((240,320,3),245,np.uint8)
    if stripes:
        for x in np.linspace(40,265,stripes).astype(int):
            cv2.rectangle(frame,(x,85),(x+13,180),(0,0,0),-1)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    dark=cv2.inRange(hsv,(0,0,0),(179,255,80))
    return frame,hsv,dark


def test_v6_zebra_detection_has_regular_bars_bright_gaps_not_single_line():
    for stripes,expected in ((6,True),(4,True),(1,False),(0,False)):
        _,hsv,dark=zebra_frame(stripes)
        assert bool(detect_crosswalk(dark,hsv,PatrolTuning())[0]) is expected
    hsv=np.zeros((240,320,3),np.uint8)
    assert not detect_crosswalk(np.full((240,320),255,np.uint8),hsv,PatrolTuning())[0]


def test_v6_actual_vision_detects_zebra_only_after_exit(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    t=PatrolTuning()
    frame,_,_=zebra_frame()
    for now in (1.,1.03,1.06):
        v.estimate(frame,t,now)
    assert v.router.crosswalk_count==0
    v.router._phase("SEEK_CROSSWALK",2.)
    v.router.after_ring_at=2.
    assert v.router.course.confirm_ring()  # task-only fixture: confirmed first exit
    for now in (3.6,3.63):
        v.estimate(frame,t,now)
    assert v.crosswalk_score>.8 and v.router.crosswalk_count==1 and v.router.phase=="CW_APPROACH"


def test_v6_exit_token_blocks_reentry_until_crosswalk_and_clear_done():
    r,t=enter_router()
    r._phase("EXIT",1.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    r.last_time=1.
    for now in np.arange(1.1,2.2,.1):
        r.observe(False,0.,now,t,(.5,0.))
        r.accept_path(PathEstimate(True,route_locked=True,route_observed=True),now,t)
    assert r.phase=="SEEK_CROSSWALK"
    for now in np.arange(2.2,20.,.1):
        r.observe(True,.95,now,t,(.5,.1))
    assert r.phase=="SEEK_CROSSWALK" and r.completed==1
    r.observe(False,0.,20.,t,(.5,0.),crosswalk=True)
    r.observe(False,0.,20.1,t,(.5,0.),crosswalk=True)
    assert r.phase=="CW_APPROACH" and r.crosswalk_count==1
    for now in np.arange(20.2,20.9,.1):
        r.observe(False,0.,now,t,(.5,0.))
    assert r.phase=="CW_STOP"
    stop_at=r.phase_at
    r.observe(True,.95,stop_at+3.01,t)
    assert r.phase=="CW_CLEAR"
    r.observe(True,.95,stop_at+6.02,t,(.5,0.))
    assert r.phase=="SEEK"


def test_v6_second_zebra_requires_its_own_ring_and_then_finishes_distance():
    r=RoundaboutRouter()
    t=PatrolTuning()
    r.started_at=0.
    assert r.course.confirm_ring() and r.course.confirm_zebra() and r.course.next_lap()
    r.crosswalk_count=1
    r.last_crosswalk_at=2.
    r._phase("SEEK",10.)
    for now in (10.1,10.2,10.3):
        r.observe(False,0.,now,t,(.5,0.),crosswalk=True)
    assert r.crosswalk_count==1
    assert r.course.confirm_ring()  # second ring receipt, not a count shortcut
    r.completed=2
    r.after_ring_at=10.
    r._phase("SEEK_CROSSWALK",10.)
    for now in (12.,12.1):
        r.observe(False,0.,now,t,(.5,0.),crosswalk=True)
    assert r.phase=="FINISH_RUN" and r.crosswalk_count==2
    for now in np.arange(12.2,14.4,.1):
        r.observe(True,.95,now,t,(.5,0.))
    assert r.phase=="FINISHED" and r.motion_override(14.4,t)==(0.,0.,"finished")


def test_v6_zebra_optional_thirty_second_gates_are_not_task_substitutes():
    t=replace(PatrolTuning(),crosswalk_first_min_seconds=30.,crosswalk_repeat_min_seconds=30.)
    r=RoundaboutRouter()
    r.started_at=0.
    assert r.course.confirm_ring()
    r.after_ring_at=2.
    r._phase("SEEK_CROSSWALK",2.)
    assert not r.crosswalk_armed(29.,t) and r.crosswalk_armed(30.,t)
    assert r.course.confirm_zebra() and r.course.next_lap() and r.course.confirm_ring()
    r.crosswalk_count=1
    r.last_crosswalk_at=30.
    r.after_ring_at=50.
    assert not r.crosswalk_armed(59.,t) and r.crosswalk_armed(60.,t)


def test_v6_task_heading_ignores_zebra_lines_with_same_cruise_target():
    r=RoundaboutRouter()
    t=PatrolTuning()
    r._phase("CW_APPROACH",0.)
    r.heading_target=.5
    v,w,mode=r.motion_override(.1,t,(0.,0.,.7))
    assert v==t.fixed_speed_m_s and w<0 and mode=="crosswalk_approach"
    assert r.motion_override(.1,t)[1]==0  # no stale heading without fresh odom
    c,_=cruise_controller(-.8)
    for now in np.arange(2.03,2.4,.03):
        v,w=c.task_command(v,0.,now,t)
    assert v==t.fixed_speed_m_s and abs(w)<.1
    assert c.task_command(.5,1.,2.5,t,immediate_stop=True)==(0.,0.)


def test_v6_search_can_reach_cruise_not_freeze_low_start_speed():
    c=FixedSpeedController()
    t=PatrolTuning()
    c.command(PathEstimate(True,-.1,.9,source="black"),0.,t)
    for now in np.arange(.03,1.2,.03):
        v,_,_=c.command(PathEstimate(False),now,t)
    assert v==pytest.approx(t.fixed_speed_m_s)


def test_v6_no_second_lap_speed_branch_after_task_stop():
    c,t=cruise_controller(-.3)
    c.task_command(0.,0.,2.1,t,immediate_stop=True)
    for now in np.arange(2.13,3.5,.03):
        v,_,_=c.command(PathEstimate(True,-.1,.9,source="black"),now,t)
    assert v==t.fixed_speed_m_s


def test_v6_pause_preserves_ring_and_crosswalk_order_not_restart_entry():
    r,t=enter_router()
    r._phase("SEEK_CROSSWALK",1.)
    r.after_ring_at=1.
    r.completed=1
    r.pause(2.)
    r.resume(102.)
    assert r.completed==1 and r.phase=="SEEK_CROSSWALK" and r.phase_at==101.
    assert not r.crosswalk_armed(102.,t)


def test_v6_zero_command_odom_drift_does_not_arm_entry():
    r,t=RoundaboutRouter(),PatrolTuning()
    for i in range(200):
        r.observe(True,.95,i*.03,t,(0.,0.),(i*.01,0.,0.))
    assert r.phase=="SEEK" and r.distance==0.


def test_v6_reset_topic_is_separate_from_pause(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    node_module.base.STAND_SETTLE_SECONDS=2.
    node_module.base.StandGate=lambda **kw:types.SimpleNamespace(**kw)
    reset=[]
    node.vision.reset_path=lambda:None
    node.vision.reset_course=lambda:reset.append(True)
    node.patrol_watcher=types.SimpleNamespace(signature="old")
    node._on_patrol_reset_course(types.SimpleNamespace(data=True))
    assert reset==[True] and commands==[(0.,0.)] and node.patrol_watcher.signature is None


def test_v6_node_crosswalk_stop_bypasses_line_controller_not_fault(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter()
    r._phase("CW_STOP",9.)
    node.vision.router=r
    node._run_control_cycle()
    assert commands==[(0.,0.)] and not node.patrol_controller.fault_latched


@pytest.mark.parametrize("changes",[
    {"maintain_cruise_during_search":2},{"roundabout_entry_gate_near_ratio":0},
    {"roundabout_approach_max_seconds":0},{"crosswalk_min_stripes":1},
    {"crosswalk_roi_bottom_ratio":.2},{"crosswalk_stop_seconds":16},
    {"crosswalk_clear_seconds":7},{"finish_after_second_crosswalk":2},
    {"crosswalk_task_max_seconds":0},{"crosswalk_confirm_frames":1.5},
])
def test_v6_invalid_task_parameters_rejected(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v6_two_lap_task_order_without_timer_reentry_or_speed_drop():
    r=RoundaboutRouter()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.,
              roundabout_approach_distance_m=.10,roundabout_entry_min_seconds=.10,
              roundabout_entry_distance_m=.10,roundabout_entry_min_yaw_rad=.1,
              roundabout_follow_min_seconds=.1,roundabout_follow_min_distance_m=.1,
              roundabout_exit_min_yaw_rad=.2,roundabout_exit_distance_m=.1,
              roundabout_rearm_seconds=0.,roundabout_rearm_distance_m=0.,
              crosswalk_delay_after_roundabout=0.,crosswalk_approach_distance_m=.1,
              crosswalk_stop_seconds=.2,crosswalk_clear_seconds=.2,
              second_crosswalk_finish_distance_m=.2)
    now=0.
    phases=[]
    for _ in range(600):
        phase=r.phase
        gate=phase in ("SEEK","APPROACH")
        cmd=(0.,0.) if phase=="CW_STOP" else (.5,.5)
        r.observe(gate,.95 if gate else 0.,now,t,cmd,gate_near=.8,dash_cue=vv_test_cue(entry=phase=="APPROACH") if gate else None,
                  exit_solid=phase=="FOLLOW",crosswalk=phase=="SEEK_CROSSWALK")
        r.accept_path(PathEstimate(True,route_locked=True,route_observed=True),now,t)
        if not phases or phases[-1]!=r.phase:
            phases.append(r.phase)
        if r.phase=="FINISHED":
            break
        now+=.05
    assert phases==["SEEK","APPROACH","ENTRY","FOLLOW","EXIT","SEEK_CROSSWALK",
                    "CW_APPROACH","CW_STOP","CW_CLEAR","SEEK","APPROACH","ENTRY",
                    "FOLLOW","EXIT","SEEK_CROSSWALK","FINISH_RUN","FINISHED"]
    assert r.completed==2 and r.crosswalk_count==2


def test_v6_node_actual_crosswalk_motion_is_explicit_not_line_speed_override(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter()
    r._phase("CW_CLEAR",9.)
    node.vision.router=r
    node._run_control_cycle()
    assert commands and 0<commands[-1][0]<=node.patrol_tuning.fixed_speed_m_s
    assert commands[-1][1]>0 and not node.patrol_controller.fault_latched  # V11 follows the visible line


def test_v6_live_crosswalk_disable_cannot_skip_active_task(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    node.vision.router=RoundaboutRouter()
    node.vision.router._phase("CW_STOP",0.)
    old=node.patrol_tuning
    node.patrol_watcher=types.SimpleNamespace(poll=lambda:(replace(old,crosswalk_enabled=0),"loaded"),signature="old")
    node._poll_live_tuning_file()
    assert node.patrol_tuning==old and node.patrol_watcher.signature is None and not commands


def test_v6_progress_timeout_is_not_counted_as_crosswalk_or_finish_success():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FINISH_RUN",0.)
    r.observe(False,0.,13.,t,(.5,0.),pose=(0.,0.,0.))
    assert r.fault=="crosswalk_motion_timeout" and r.phase=="FINISH_RUN"


def test_v6_finished_task_stays_stopped_despite_new_lines_and_gates():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FINISHED",0.)
    for now in (1.,2.,50.):
        r.observe(True,.95,now,t,(0.,0.),crosswalk=True)
        assert r.phase=="FINISHED" and r.motion_override(now,t)==(0.,0.,"finished")


def test_v6_fresh_camera_does_not_silently_clear_task_fault():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    r.fault="roundabout_route_timeout"
    r.pause(1.)
    r.observe(False,0.,2.,t)
    assert r.fault=="roundabout_route_timeout"
    r.pause(2.)
    r.resume(3.,clear_fault=True)
    assert not r.fault and r.phase=="FOLLOW" and r.phase_at==3.


@pytest.mark.parametrize("phase",["SEEK","APPROACH","ENTRY","FOLLOW","EXIT","RING_WAIT_DEADLINE"])
def test_v14_first_budget_releases_control_but_never_ring_receipt(phase):
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=100.
    r._phase(phase,129.)
    assert not r.apply_time_fallback(129.999,t)
    assert r.apply_time_fallback(130.,t)
    assert r.phase=="SEEK" and not r.left_lock
    assert not r.crosswalk_armed(130.,t)  # timeout is not an exit receipt
    assert r.completed==0 and r.timed_releases==1
    assert r.crosswalk_count==0 and r.release_cycle==-1
    assert not r.apply_time_fallback(131.,t) and r.timed_releases==1
    assert r.motion_override(130.,t) is None  # ordinary patrol, not an auto-stop


def test_v8_second_deadline_is_first_zebra_plus_forty_five_not_ring_entry():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    r.last_crosswalk_at=40.
    assert r.course.confirm_ring() and r.course.confirm_zebra() and r.course.next_lap()
    r.crosswalk_count=1
    r.release_cycle=0
    r._phase("ENTRY",80.)
    assert not r.apply_time_fallback(84.999,t)
    assert r.apply_time_fallback(85.,t)
    assert r.phase=="SEEK" and r.release_cycle==0
    assert r.crosswalk_count==1 and not r.crosswalk_armed(85.,t)
    for now in (85.03,85.06):
        r.observe(False,0.,now,t,(.65,0.),crosswalk=True)
    assert r.crosswalk_count==1 and r.phase=="SEEK"
    assert r.motion_override(85.1,t) is None


@pytest.mark.parametrize("count",[0,1])
def test_v8_real_exit_cancels_only_current_cycle_deadline(count):
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    if count:
        assert r.course.confirm_ring() and r.course.confirm_zebra() and r.course.next_lap()
    r.crosswalk_count=count
    r.last_crosswalk_at=5. if count else None
    r._phase("EXIT",20.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    r.phase_distance=1.
    for now in (21.,21.03,21.06):
        r.accept_path(PathEstimate(True,route_locked=True,route_observed=True),now,t)
    assert r.ring_release_source=="visual_exit" and r.completed==count+1
    assert not r.apply_time_fallback(100.,t) and r.timed_releases==0
    assert r.phase=="SEEK_CROSSWALK"
    assert not r.crosswalk_armed(21.1,t) and r.crosswalk_armed(23.,t)


def test_v8_second_deadline_cannot_interrupt_first_stop_or_clear():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    r.last_crosswalk_at=20.
    r.crosswalk_count=1
    r.release_cycle=0
    for phase in ("CW_APPROACH","CW_STOP","CW_CLEAR"):
        r._phase(phase,21.)
        assert not r.apply_time_fallback(100.,t)
        assert r.phase==phase
    r._phase("SEEK",101.)  # only after the first stop/clear sequence completes
    assert r.apply_time_fallback(101.,t)


def test_v8_route_budget_releases_left_lock_but_waits_for_global_deadline():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    r._phase("ENTRY",6.)
    r.observe(False,0.,16.1,t,(.65,.8))
    assert r.phase=="SEEK" and not r.fault
    assert not r.left_lock and not r.crosswalk_armed(16.1,t)
    assert not r.gate_detection_allowed(20.,t)
    assert r.motion_override(20.,t) is None
    assert not r.apply_time_fallback(30.,t)  # retry budget renewed at control release and r.phase=="SEEK_CROSSWALK"


def test_v8_fallback_never_restarts_faults_and_manual_pause_extends_deadline():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    r.pause(10.)
    assert not r.apply_time_fallback(100.,t)
    r.resume(20.)
    assert r.ring_deadline(t)==40.
    assert not r.apply_time_fallback(39.99,t)
    r.fault="camera_or_localization_fault"
    assert not r.apply_time_fallback(41.,t) and r.fault
    assert r.phase=="SEEK" and r.crosswalk_count==0


def test_v8_second_deadline_pause_uses_shifted_first_zebra_time():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    assert r.course.confirm_ring() and r.course.confirm_zebra() and r.course.next_lap()
    r.crosswalk_count=1
    r.last_crosswalk_at=25.
    r.pause(30.)
    r.resume(40.)
    assert r.ring_deadline(t)==80.
    assert not r.apply_time_fallback(79.99,t)
    assert r.apply_time_fallback(80.,t)


def test_v8_disabled_fallback_does_not_fake_exit_or_stop_clock():
    r=RoundaboutRouter()
    t=replace(PatrolTuning(),roundabout_timeout_enabled=0)
    r.started_at=0.
    assert not r.apply_time_fallback(500.,t) and r.phase=="SEEK"
    r._phase("ENTRY",10.)
    r.observe(False,0.,21.,t)
    assert r.fault=="roundabout_route_timeout"


def test_v14_actual_vision_at_deadline_drops_lock_without_accepting_zebra(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    r=v.router
    r.started_at=0.
    r._phase("ENTRY",29.)
    v.previous_points=np.array([[30.,100.],[20.,230.]])
    v._detect_roundabout_gate=lambda *_: pytest.fail("entry detection after deadline")
    frame,_,_=zebra_frame()
    v.estimate(frame,PatrolTuning(),30.)
    assert r.phase=="SEEK" and r.crosswalk_frames==0
    assert r.completed==0 and r.timed_releases==1 and not r.left_lock
    v.estimate(frame,PatrolTuning(),30.03)
    assert r.phase=="SEEK" and r.crosswalk_count==0


def test_v8_timeout_without_stripes_never_counts_zebra_or_finishes():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    for now in (30.,30.03,40.,80.):
        r.observe(False,0.,now,t,(.65,0.))
    assert r.phase=="SEEK" and r.crosswalk_count==0
    assert r.timed_releases==2 and r.completed==0


def test_v8_node_deadline_handoff_keeps_cruise_and_clears_old_route_pd(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch,now=30.)
    r=RoundaboutRouter()
    r.started_at=0.
    r._phase("ENTRY",29.)
    node.vision.router=r
    node.patrol_controller.last_command=(.65,.8)
    node.patrol_controller.last_time=29.97
    node.patrol_controller.last_good_turn=.8
    def estimate(*_args,**_kwargs):
        r.apply_time_fallback(30.,node.patrol_tuning)
        return PathEstimate(True,0.,.9,source="black")
    node.vision.estimate=estimate
    node._run_control_cycle()
    assert r.phase=="SEEK" and r.ring_release_source=="time_fallback"
    assert commands[-1][0]==.65 and 0<=commands[-1][1]<.8
    assert node._navigation_reset_seen==r.navigation_reset_serial


def test_v8_hot_reload_applies_deadline_and_point_six_five_speed(tmp_path):
    config=tmp_path/"patrol.yaml"
    values=asdict(PatrolTuning())
    config.write_text(yaml.safe_dump({"patrol_test":values}),encoding="utf-8")
    watcher=PatrolFileWatcher(config)
    t,_=watcher.poll()
    assert t.fixed_speed_m_s==.65
    values["roundabout_first_timeout_seconds"]=20.
    config.write_text(yaml.safe_dump({"patrol_test":values}),encoding="utf-8")
    t,message=watcher.poll()
    r=RoundaboutRouter()
    r.started_at=0.
    assert t is not None,message
    assert r.apply_time_fallback(20.,t)


def test_v14_repeated_timeouts_and_stripes_cannot_advance_or_finish():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    for now in (30.,30.03,60.,60.03,90.,90.03):
        r.observe(False,0.,now,t,(.65,0.),crosswalk=True)
        assert r.phase=="SEEK" and r.crosswalk_count==0
        assert not any(r.course.rings) and not any(r.course.zebras)
        assert not r.finish_authorized
    assert r.timed_releases==3 and r.motion_override(90.04,t) is None



@pytest.mark.parametrize("changes",[
    {"roundabout_timeout_enabled":2},{"roundabout_first_timeout_seconds":0},
    {"roundabout_second_timeout_seconds":-1},{"roundabout_first_timeout_seconds":float("nan")},
])
def test_v8_deadline_parameters_reject_invalid_values(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v9_large_reflection_across_blue_cannot_win_by_area_or_seed_overlap():
    image=np.full((240,320,3),(255,0,0),np.uint8)
    image[:,90:151]=245
    image[:,162:306]=245  # larger white floor beyond a thin blue separator
    t=replace(PatrolTuning(),seed_half_width_ratio=.35)
    previous=np.array([[130.,0.],[130.,239.]])
    result=road_centerline(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),t,130/320,previous)
    assert result.valid and np.all(result.points[:,0]<151)
    assert result.error<0 and np.all(result.mask[:,162:]==0)


def test_v9_anchor_on_visible_blue_cannot_jump_to_either_white_patch():
    image=np.full((240,320,3),(255,0,0),np.uint8)
    image[:,50:146]=245
    image[:,170:300]=245
    result=road_centerline(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),PatrolTuning(),.5)
    assert not result.valid and result.reason=="no_near_connected_road"


def test_v9_exit_chooses_connected_left_channel_not_larger_right_channel():
    image=np.full((240,320,3),(255,0,0),np.uint8)
    image[205:,40:300]=245
    image[:205,40:120]=245
    image[:205,150:300]=245
    result=road_centerline(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),PatrolTuning(),.5,branch="left")
    assert result.valid and result.route_locked and result.branch_rows>0
    assert np.all(result.points[result.points[:,1]<190,0]<120)
    assert result.error<0


def test_v9_healthy_black_caches_real_width_before_one_edge_disappears(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    frame=np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,80:241]=245
    cv2.line(frame,(160,0),(160,239),(0,0,0),7)
    t=PatrolTuning()
    assert v.estimate(frame,t,1.).valid and v.source=="black"
    assert v.road_widths is not None and v.last_dual_at==1.
    partial=np.full((240,320,3),(255,0,0),np.uint8)
    partial[:,:241]=245  # observed right boundary only, no black
    path=v.estimate(partial,t,1.1)
    assert path.valid and v.source=="road" and v.road_estimate.single_edge
    assert v.last_dual_at==1.  # single-edge frames cannot renew width history


def test_v9_width_probe_is_low_rate_and_cannot_override_valid_black(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    frame=np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,60:261]=245
    cv2.line(frame,(160,0),(160,239),(0,0,0),7)
    calls=[]
    original=module.road_centerline
    def probe(*args,**kwargs):
        calls.append(1)
        return original(*args,**kwargs)
    monkeypatch.setattr(module,"road_centerline",probe)
    for now in np.arange(0.,1.,1/30):
        p=v.estimate(frame,PatrolTuning(),now)
        assert p.valid and v.source=="black"
    assert 3<=len(calls)<=5  # not every camera frame


def test_v9_bounded_search_continues_past_old_one_meter_then_safely_waits():
    c,t=cruise_controller(search_degraded_enabled=0)  # retain opt-in legacy boundary
    for now in np.arange(2.03,4.6,.03):
        v,w,mode=c.command(PathEstimate(False),now,t)
        assert v==.65 and mode in ("path_memory_bridge","active_search")
    assert c.loss_distance>1.
    for now in np.arange(4.6,5.2,.03):
        result=c.command(PathEstimate(False),now,t)
    assert result==(0.,0.,"localization_fault_latched")
    assert c.fault_reason in ("distance_limit","time_limit")


def test_v14_eighteen_second_zebra_shortcut_cannot_release_missing_second_ring(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    v.router.started_at=0.
    assert v.router.course.confirm_ring() and v.router.course.confirm_zebra() and v.router.course.next_lap()
    v.router.crosswalk_count=1
    v.router.last_crosswalk_at=10.
    v.router.release_cycle=0
    frame,_,_=zebra_frame(6)
    for now in (27.90,27.93,27.96):
        v.estimate(frame,PatrolTuning(),now)
    assert v.router.phase=="SEEK" and v.router.visual_releases==0
    for now in (28.,28.03,28.06):
        v.estimate(frame,PatrolTuning(),now)
    assert v.router.phase=="SEEK" and v.router.crosswalk_count==1
    assert v.router.ring_release_source=="none"
    assert v.router.completed==0 and v.router.visual_releases==0


@pytest.mark.parametrize("phase",["ENTRY","FOLLOW","EXIT","CW_APPROACH","CW_STOP","CW_CLEAR","FINISH_RUN","FINISHED"])
def test_v9_early_zebra_cannot_steal_active_ring_or_clear_task(phase):
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    r._phase(phase,29.)
    assert not r.crosswalk_visual_release_allowed(29.,t)
    for now in (29.,29.03,29.06):
        assert not r.observe_crosswalk_visual_release(True,now,t)
    assert r.phase==phase and r.visual_releases==0


def test_v9_early_zebra_flicker_and_pause_do_not_accumulate():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    for detected,now in ((True,20.),(False,20.03),(True,20.06),(True,20.4)):
        assert not r.observe_crosswalk_visual_release(detected,now,t)
    assert r.visual_release_frames==0
    r.pause(20.5)
    assert r.visual_release_frames==0
    assert not r.observe_crosswalk_visual_release(True,21.,t)


def test_v9_four_stripes_still_need_normal_exit_not_early_override(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    v.router.started_at=0.
    frame,_,_=zebra_frame(4)
    for now in (20.,20.03,20.06,20.09):
        v.estimate(frame,PatrolTuning(),now)
    assert v.router.phase=="SEEK" and v.router.crosswalk_count==0
    v.router._phase("EXIT",21.)
    v.router.ring_route_acquired=True  # fixture: actual route already acquired
    v.router._release_ring(21.,PatrolTuning(),"visual_exit")
    for now in (23.,23.03):
        v.estimate(frame,PatrolTuning(),now)
    assert v.router.crosswalk_count==1 and v.router.phase=="CW_APPROACH"


def test_v9_normal_preview_and_video_recording_remain_disabled():
    launch=(SCRIPTS.parent/"launch/patrol_test.launch.py").read_text(encoding="utf-8")
    assert '"runtime.record_raw_video:=false"' in launch
    assert '"runtime.record_debug_video:=false"' in launch
    assert '"camera.control_rate:=30.0"' in launch


@pytest.mark.parametrize("changes",[
    {"road_width_probe_seconds":0},{"crosswalk_visual_release_enabled":2},
    {"crosswalk_visual_release_frames":1},{"crosswalk_visual_release_seconds":-1},
    {"crosswalk_visual_release_min_stripes":3},
])
def test_v9_new_parameters_reject_invalid_values(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v9_branch_preview_never_creates_center_pixels_across_blue():
    image=fork_scene()
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    blue,_=road_masks(hsv,PatrolTuning())
    for enabled in (0,1):
        p=road_centerline(hsv,replace(PatrolTuning(),roundabout_left_preview_enabled=enabled),.60,branch="left")
        assert p.valid
        pixels=np.rint(p.points).astype(int)
        assert np.all(blue[pixels[:,1],pixels[:,0]]==0)
    ordinary=road_centerline(hsv,PatrolTuning(),.60)
    no_preview=road_centerline(hsv,replace(PatrolTuning(),roundabout_left_preview_enabled=0),.60)
    assert ordinary.valid==no_preview.valid
    assert np.array_equal(ordinary.points,no_preview.points)


def test_v9_main_callback_exception_keeps_node_alive_until_operator_exit(node_module,monkeypatch):
    events=[]
    class Node:
        exit_requested=False
        def _wait_for_operator(self,reason):
            events.append(("wait",reason))
        def get_logger(self):
            return types.SimpleNamespace(error=lambda text:events.append(("error",text)),warn=lambda text:events.append(("warn",text)))
        def close(self):
            events.append(("close",True))
        def destroy_node(self):
            events.append(("destroy",True))
    node=Node()
    calls=[]
    def spin(current,**_kwargs):
        calls.append(1)
        if len(calls)==1:
            raise RuntimeError("synthetic callback, no motors")
        current.exit_requested=True  # explicit operator request
    fake_ros=types.SimpleNamespace(init=lambda **_:None,ok=lambda:True,spin_once=spin,shutdown=lambda:events.append(("shutdown",True)))
    monkeypatch.setattr(node_module.base,"rclpy",fake_ros,raising=False)
    monkeypatch.setattr(node_module.base,"SignalHandlerOptions",types.SimpleNamespace(NO=0),raising=False)
    monkeypatch.setattr(node_module,"PatrolTestNode",lambda:node)
    monkeypatch.setattr(node_module.signal,"signal",lambda *_:None)
    node_module.main()
    assert len(calls)==2
    assert any(kind=="wait" for kind,_ in events)
    assert sum(kind=="close" for kind,_ in events)==1
    assert any(kind=="error" and "PATROL_CALLBACK_FAULT" in text for kind,text in events)


def test_v9_early_zebra_never_resumes_fault_or_changes_cruise_target():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.
    r.fault="localization_fault"
    for now in (20.,20.03,20.06):
        assert not r.observe_crosswalk_visual_release(True,now,t)
    assert r.fault and r.crosswalk_count==0 and t.fixed_speed_m_s==.65


def test_v9_same_path_outputs_same_commands_before_and_after_first_zebra(node_module,monkeypatch):
    outputs=[]
    for count in (0,1):
        node,commands=make_node(node_module,monkeypatch)
        r=RoundaboutRouter()
        r.crosswalk_count=count
        r.started_at=0.
        r.last_crosswalk_at=2. if count else None
        node.vision.router=r
        node.patrol_controller.last_command=(.65,0.)
        node.patrol_controller.last_time=9.97
        node.vision.estimate=lambda *_a,**_kw:PathEstimate(True,-.5,.9)
        node._run_control_cycle()
        outputs.append(commands[-1])
    assert outputs[0]==outputs[1] and outputs[0][0]==.65


def tracked_gate_chain(x=145., shift=0.):
    return [(x+(y-140)*.05,y+shift,10,12,100) for y in (65.,90.,115.,140.,165.,190.)]


def test_v10_lower_score_same_chain_can_arm_entry_without_blue():
    tracker,t=GateChainTracker(),replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    router=RoundaboutRouter()
    seen=[]
    for i in range(10):
        now=i/30
        confirmed=tracker.update(tracked_gate_chain(145+i*.5),.74,320,240,now,t)
        seen.append(confirmed)
        router.observe(confirmed,.74,now,t,(.65,0.),gate_near=.82,entry_ready=True,gate_tracked=confirmed)
    assert not any(seen[:3]) and any(seen[3:])
    assert router.phase=="SEEK"  # retired chain evidence cannot bypass VV


@pytest.mark.parametrize("case",["disabled","single","horizontal","irregular","different","stale","duplicate","low_score"])
def test_v10_gate_association_rejects_bad_or_stale_evidence(case):
    tracker,t=GateChainTracker(),PatrolTuning()
    if case=="disabled":
        t=replace(t,roundabout_gate_tracking_enabled=0)
    ready=[]
    for i in range(12):
        chain=tracked_gate_chain()
        now=i*.03
        score=.74
        if case=="single": chain=chain[:1]
        elif case=="horizontal": chain=[(40+j*35,140.,10,12,100) for j in range(6)]
        elif case=="irregular": chain=[(145.,float(y),10,12,100) for y in (60,65,70,75,80,195)]
        elif case=="different": chain=tracked_gate_chain(100. if i%2 else 220.)
        elif case=="stale": now=i*.3
        elif case=="duplicate": now=0.
        elif case=="low_score": score=.65
        ready.append(tracker.update(chain,score,320,240,now,t))
    assert not any(ready)


def test_v10_temporal_entry_evidence_is_never_exit_evidence():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    r.gate_cleared=True
    r.phase_distance=1.
    r.ring_yaw=2.
    for now in np.arange(3.,4.,.03):
        r.observe(True,.74,now,t,(.65,.1),gate_tracked=True)
    assert r.phase=="FOLLOW" and r.evidence==0.


@pytest.mark.parametrize("bad",["lock_only","rescue","jumping","gaps","too_short"])
def test_v10_entry_needs_observed_continuous_stable_geometry(bad):
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("ENTRY",0.)
    r.phase_distance=1.
    r.left_yaw=.7
    for i in range(12 if bad!="too_short" else 3):
        now=1.+i*(.3 if bad=="gaps" else .03)
        error=(-.4 if i%2 else .4) if bad=="jumping" else -.2
        p=PathEstimate(True,error,.9,near_error=error,mid_error=error,route_locked=True,
                       route_observed=bad!="lock_only",rescue=bad=="rescue")
        r.accept_path(p,now,t)
    assert r.phase=="ENTRY"


def enter_actual_black_without_blue(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    t=PatrolTuning()
    v.router._phase("ENTRY",0.)
    v.router.phase_distance=1.
    v.router.left_yaw=1.1  # V13 actual acquisition progress, not the former 20-degree release
    frame=np.full((240,320,3),245,np.uint8)
    cv2.line(frame,(110,0),(110,239),(0,0,0),7)
    for now in np.arange(1.,1.32,.04):
        p=v.estimate(frame,t,float(now))
        assert p.valid and p.route_observed
    assert v.router.phase=="FOLLOW" and v.router.branch_acquired
    assert v.ring_memory.path is not None
    return v,t


def test_v10_actual_entry_and_follow_work_without_any_blue(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    assert v.source=="black" and not v.road_estimate.valid
    assert v.router.search_left and not v.router.left_lock


def test_v10_entry_to_follow_keeps_route_and_rejects_outside_right_black(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    frame=np.full((240,320,3),245,np.uint8)
    cv2.line(frame,(230,0),(230,239),(0,0,0),7)
    for now in (1.36,1.40,1.44):
        p=v.estimate(frame,t,now)
        assert not p.valid
        assert v.black_estimate.reason=="black_outside_locked_route"
    assert v.router.phase=="FOLLOW" and v.router.branch_acquired
    assert v.ring_memory.path.near_error<0


def test_v10_owned_route_can_continuously_move_right_and_correct_right(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    for i,x in enumerate((125,140,155,170,185,200)):
        frame=np.full((240,320,3),245,np.uint8)
        cv2.line(frame,(x,0),(x,239),(0,0,0),7)
        p=v.estimate(frame,t,1.36+i*.04)
        assert p.valid and p.route_locked
    assert p.near_error>0 and p.error>0
    c=FixedSpeedController()
    for now in (2.,2.04,2.08):
        _,w,_=c.command(p,now,t,search_direction=1)
    assert w<0  # actual route correction is NOT clamped left


def test_v10_expired_ordinary_cache_cannot_erase_ring_identity(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    saved=v.ring_memory.path.near_error
    invalid=np.zeros((240,320,3),np.uint8)
    v.estimate(invalid,t,2.20)  # ordinary .8-second cache expired
    assert v.previous_points is None and v.ring_memory.path is not None
    assert v.ring_memory.path.near_error==saved
    frame=np.full((240,320,3),245,np.uint8)
    cv2.line(frame,(230,0),(230,239),(0,0,0),7)
    assert not v.estimate(frame,t,3.0).valid  # expired ring memory is not a free pass


def test_v10_follow_search_uses_left_not_stale_right_command(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter()
    r._phase("FOLLOW",9.)
    node.vision.router=r
    node.vision.estimate=lambda *_a,**_kw:PathEstimate(False,reason="lost")
    node.patrol_controller.last_valid_at=9.97
    node.patrol_controller.last_time=9.97
    node.patrol_controller.last_command=(.65,-.8)
    node.patrol_controller.last_good_turn=-.8
    node._run_control_cycle()
    assert node.patrol_controller.raw_angular>0
    assert commands[-1][0]==.65
    assert not node.patrol_controller.fault_latched


def test_v10_phase_change_preserves_only_in_ring_ownership():
    r=RoundaboutRouter()
    r._phase("ENTRY",0.)
    r.branch_acquired=True
    r.gate_cleared=True
    r.ring_yaw=.8
    r._phase("FOLLOW",1.)
    assert r.branch_acquired and r.gate_cleared and r.ring_yaw==.8
    r._phase("EXIT",2.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    assert r.branch_acquired and r.search_left
    r._phase("SEEK_CROSSWALK",3.)
    assert not r.branch_acquired and not r.search_left


def test_v10_exit_corner_builds_actual_virtual_right_boundary_only_in_exit():
    hsv=cv2.cvtColor(fork_scene(),cv2.COLOR_BGR2HSV)
    t=replace(PatrolTuning(),roundabout_exit_corner_enabled=1)
    ordinary=road_centerline(hsv,t,.5,branch="left")
    exit_path=road_centerline(hsv,t,.5,branch="left",exit_guide=True)
    assert ordinary.valid and ordinary.exit_boundary is None
    assert exit_path.valid and exit_path.exit_boundary is not None
    assert exit_path.error<0 and len(exit_path.exit_boundary)>3
    blue,_=road_masks(hsv,t)
    for points in (exit_path.points,exit_path.exit_boundary):
        pixels=np.rint(points).astype(int)
        assert np.all(blue[pixels[:,1],pixels[:,0]]==0)
    assert np.ptp(exit_path.exit_boundary[:,0])>10  # actual sloping connector
    disabled=road_centerline(hsv,replace(t,roundabout_exit_corner_enabled=0),.5,branch="left",exit_guide=True)
    assert disabled.exit_boundary is None and np.array_equal(disabled.points,ordinary.points)


@pytest.mark.parametrize("color",[(245,245,245),(255,0,0),(0,0,0)])
def test_v10_uniform_images_cannot_invent_corner_or_exit(color):
    image=np.full((240,320,3),color,np.uint8)
    p=road_centerline(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),PatrolTuning(),branch="left",exit_guide=True)
    assert not p.valid and p.exit_boundary is None


def test_v10_straight_channel_without_split_has_no_corner_connector():
    p=road_centerline(scene(),PatrolTuning(),branch="left",exit_guide=True)
    assert p.valid and p.exit_boundary is None


def test_v10_corner_is_not_exit_trigger_and_visible_fork_cannot_complete_exit():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    p=road_centerline(cv2.cvtColor(fork_scene(),cv2.COLOR_BGR2HSV),t,branch="left",exit_guide=True)
    for now in np.arange(1.,2.,.04):
        r.accept_path(p,float(now),t)
    assert r.phase=="FOLLOW" and r.completed==0
    r._phase("EXIT",2.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    r.phase_distance=1.
    for now in np.arange(2.04,3.,.04):
        r.accept_path(p,float(now),t)
    assert r.phase=="EXIT" and r.completed==0  # fork still visible


@pytest.mark.parametrize("changes",[
    {"roundabout_entry_stable_seconds":0},{"roundabout_entry_path_delta":0},
    {"roundabout_path_max_gap_seconds":0},
    {"roundabout_route_memory_seconds":0},{"roundabout_route_max_delta":1.1},
    {"roundabout_gate_tracking_enabled":2},{"roundabout_gate_track_min_score":.9},
    {"roundabout_gate_track_frames":1},{"roundabout_gate_track_frames":2.5},
    {"roundabout_gate_track_max_gap_seconds":0},{"roundabout_gate_track_max_x_ratio":0},
    {"roundabout_exit_corner_enabled":2},{"roundabout_exit_corner_min_span_ratio":-1},
    {"roundabout_exit_corner_min_shift_ratio":1.1},
])
def test_v10_new_tuning_snapshot_validation(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v10_complete_live_yaml_has_all_fields_and_preserves_non_ring_values():
    current,message=PatrolFileWatcher(SCRIPTS.parent/"config/patrol_test.yaml").poll()
    assert current is not None,message
    assert current==PatrolTuning()
    assert current.fixed_speed_m_s==.65
    assert (current.crosswalk_stop_seconds,current.crosswalk_clear_seconds)==(3.,3.)


def test_v10_actual_exit_uses_connector_and_rejects_right_branch(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    v.router._phase("EXIT",0.)
    v.router.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    frame=fork_scene()
    cv2.line(frame,(230,0),(230,239),(0,0,0),7)
    p=v.estimate(frame,replace(PatrolTuning(),roundabout_exit_corner_enabled=1),.1,draw=True)
    assert p.valid and v.source=="road" and p.error<0
    assert p.exit_boundary is not None
    assert v.black_estimate.reason=="exit_corner_route_priority"
    assert v.last_debug is not None


def test_v10_ordinary_vision_does_not_run_exit_connector(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    p=v.estimate(fork_scene(),PatrolTuning(),.1)
    assert p.exit_boundary is None
    assert v.router.phase=="SEEK"


def test_v10_reset_course_clears_both_temporal_and_route_memory(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    v.router.first_marker_at=2.
    v.router.vv_streak=4
    v.reset_course()
    assert v.ring_memory.path is None and v.router.vv_streak==0 and v.router.first_marker_at is None
    assert v.router.phase=="SEEK" and v.ring_phase=="SEEK"


def test_v10_follow_rejects_discontinuous_white_road_even_without_black(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    v.seed_x_ratio=.73  # a replaced ordinary cache must not erase task identity
    v.previous_points=None
    frame=np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,160:310]=245
    p=v.estimate(frame,t,1.36)
    assert not p.valid and v.road_estimate.reason=="road_outside_owned_ring_route"


def test_v10_ring_path_memory_does_not_renew_on_handoff_or_invalid():
    memory,t=RingRouteMemory(),PatrolTuning()
    memory.remember(PathEstimate(True,-.2,.9,near_error=-.2,mid_error=-.2,route_observed=True,route_locked=True),0.)
    memory.remember(PathEstimate(True,.5,.9,near_error=.5,mid_error=.5,route_observed=True,handoff=True),.1)
    memory.remember(PathEstimate(False,route_observed=True),.2)
    assert memory.at==0. and memory.path.near_error==-.2
    assert memory.agrees(PathEstimate(True,near_error=.5,mid_error=.5),.3,t) is False
    assert memory.agrees(PathEstimate(True,near_error=-.2,mid_error=-.2),2.,t) is None


def test_v10_exit_needs_three_fresh_plain_frames_after_fork_disappears():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("EXIT",0.)
    r.ring_route_acquired=True  # fixture: earlier ENTRY acquisition already verified
    r.branch_acquired=True
    r.phase_distance=1.
    fork=PathEstimate(True,route_locked=True,route_observed=True,branch_rows=3)
    for now in np.arange(1.,1.4,.03):
        r.accept_path(fork,float(now),t)
    assert r.phase=="EXIT" and r.path_streak==0
    plain=PathEstimate(True,route_locked=True,route_observed=True)
    for now in (1.43,1.46):
        r.accept_path(plain,now,t)
        assert r.phase=="EXIT"
    r.accept_path(plain,1.49,t)
    assert r.phase=="SEEK_CROSSWALK" and r.completed==1


def test_v10_pause_preserves_memory_age_but_still_checks_new_geometry(monkeypatch):
    v,t=enter_actual_black_without_blue(monkeypatch)
    old_at=v.ring_memory.at
    v.reset_path()
    v.router.pause(1.40)
    frame=np.full((240,320,3),245,np.uint8)
    cv2.line(frame,(110,0),(110,239),(0,0,0),7)
    p=v.estimate(frame,t,101.40)
    assert p.valid and v.router.phase=="FOLLOW"
    assert v.router.paused_at is None and v.ring_memory.at>old_at+99.
    wrong=np.full((240,320,3),245,np.uint8)
    cv2.line(wrong,(230,0),(230,239),(0,0,0),7)
    assert not v.estimate(wrong,t,101.44).valid


def test_v11_entry_rejects_the_right_road_that_cancelled_left_acquisition(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    v.router._phase("ENTRY",0.)
    v.router.left_yaw=.42  # the real log stopped forcing left at this point
    frame=np.full((240,320,3),(255,0,0),np.uint8)
    frame[:,160:310]=245
    p=v.estimate(frame,PatrolTuning(),.7)
    assert not p.valid and v.road_estimate.reason=="entry_unacquired_right_road"
    assert v.ring_memory.path is None
    speed,turn,mode=v.router.motion_override(.7,PatrolTuning())
    assert speed==.65 and turn==1.2 and mode=="roundabout_left_acquire"


def test_v11_actual_near_bands_can_acquire_without_exact_middle_row():
    t=PatrolTuning()
    p=PathEstimate(True,points=np.array([[120,223],[110,202],[95,180]]))
    assert observed_route(p,320,240,t)
    assert not observed_route(replace(p,points=p.points[:2]),320,240,t)
    assert not observed_route(replace(p,rescue=True),320,240,t)


def test_v11_unowned_path_does_not_seed_ring_memory():
    m=RingRouteMemory()
    m.remember(PathEstimate(True,.35,.9,near_error=.32,mid_error=.35,route_observed=True),1.)
    assert m.path is None


def feed_measured_turn(monitor,degrees,tuning=None,source="imu",travel=.05):
    t=tuning or PatrolTuning()
    for i,angle in enumerate(degrees):
        now=i*.1
        yaw=(math.radians(angle)+math.pi)%(2*math.pi)-math.pi
        monitor.update((source,yaw,now),now,True,travel,t)
    return now


def test_v11_measured_yaw_unwraps_180_and_requires_real_full_turn():
    m,t=MeasuredRingTurn(),PatrolTuning()
    now=feed_measured_turn(m,np.arange(170,521,10),t)
    assert m.available and math.degrees(m.angle)==pytest.approx(350.)
    assert m.loop_ready(now,t)


@pytest.mark.parametrize("case",["no_motion","oscillation","little_travel","source_switch","gap","jump","stale","duplicate"])
def test_v11_heading_return_alone_cannot_prove_ring(case):
    m,t=MeasuredRingTurn(),PatrolTuning()
    if case=="no_motion":
        feed_measured_turn(m,[0.]*36,t)
    elif case=="oscillation":
        feed_measured_turn(m,[0,20,40,20,0,-20,-40,-20,0]*4,t)
    else:
        feed_measured_turn(m,np.arange(0,351,10),t,travel=0. if case=="little_travel" else .05)
        if case=="source_switch": m.update(("odom",math.radians(-10),3.6),3.6,True,.05,t)
        elif case=="gap": m.update(("imu",math.radians(-10),4.),4.,True,.05,t)
        elif case=="jump": m.update(("imu",math.radians(100),3.6),3.6,True,.05,t)
        elif case=="stale": assert not m.loop_ready(4.,t); return
        elif case=="duplicate":
            before=m.angle
            for _ in range(5): m.update(("imu",math.radians(90),3.5),3.5,True,0.,t)
            assert m.angle==before
            return
    assert not m.loop_ready(3.6 if case in ("source_switch","jump") else 4. if case=="gap" else 3.5,t)


def test_v11_loop_evidence_releases_ring_once_without_fake_visual_success():
    # Explicit V11 compatibility: V12 default requires a real exit route.
    r,t=RoundaboutRouter(),replace(PatrolTuning(),roundabout_require_exit_route=0)
    r._phase("FOLLOW",0.)
    r.branch_acquired=True
    for i,deg in enumerate(np.arange(0,351,10)):
        now=i*.1
        yaw=(math.radians(deg)+math.pi)%(2*math.pi)-math.pi
        r.observe(False,0.,now,t,(.65,1.),pose=(i*.05,0.,yaw),heading=("imu",yaw,now))
    assert r.phase=="SEEK" and r.ring_release_source=="yaw_loop_release"
    assert not r.crosswalk_armed(now,t)
    assert r.yaw_releases==1 and r.completed==0 and r.timed_releases==0
    for now in (4.,5.,6.):
        r.observe(True,.99,now,t,(.65,0.))
    assert r.phase=="SEEK" and r.yaw_releases==1


@pytest.mark.parametrize("case",["no_turn","no_route"])
def test_v14_failed_measured_entry_retries_without_authorizing_zebra(case):
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("ENTRY",0.)
    for i in range(33):
        now=i*.1
        yaw=0. if case=="no_turn" else i*.1
        r.observe(False,0.,now,t,(.65,1.2),pose=(i*.05,0.,yaw),heading=("odom",yaw,now))
        if r.phase!="ENTRY": break
    assert r.phase=="SEEK" and r.completed==0 and r.aborted_rings==1
    assert r.ring_release_source==("entry_no_turn_abort" if case=="no_turn" else "entry_no_route_abort")
    assert not r.crosswalk_armed(now+1.6,t)


def test_v11_commanded_circle_without_measured_heading_is_not_loop_proof():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    for now in np.arange(0.,7.,.1):
        r.observe(False,0.,float(now),t,(.65,1.))
    assert r.ring_yaw>6. and r.measured_turn.angle==0.
    assert r.yaw_releases==0 and r.phase=="FOLLOW"


def test_v11_actual_heading_gates_early_visual_exit():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    r.ring_yaw=2.
    r.phase_distance=1.
    r.gate_cleared=True
    m=r.measured_turn
    m.source,m.last_yaw,m.stamp="imu",math.pi/2,3.
    m.angle=m.absolute=math.pi/2
    for now in (3.1,3.2,3.3):
        r.observe(False,0.,now,t,(.65,0.),exit_solid=True,heading=("imu",math.pi/2,now))
    assert r.phase=="FOLLOW"


def test_v11_invalid_quaternion_falls_back_to_fresh_odom(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    node=module.PatrolTestNode.__new__(module.PatrolTestNode)
    monkeypatch.setattr(module.time,"monotonic",lambda:10.)
    node._last_odom_time=9.98
    message=types.SimpleNamespace(orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=0.),
                                  angular_velocity=types.SimpleNamespace(z=0.),orientation_covariance=[0.]*9)
    node._on_imu(message)
    assert node._heading_snapshot(10.,PatrolTuning(),(0.,0.,.2))==("odom",.2,9.98)
    message.orientation.w=1.
    node._on_imu(message)
    assert node._heading_snapshot(10.,PatrolTuning(),(0.,0.,.2))[0]=="imu"
    message.orientation_covariance[0]=-1
    node._on_imu(message)
    assert node._heading_snapshot(10.,replace(PatrolTuning(),roundabout_heading_source=1),(0.,0.,.2)) is None


def test_v11_heading_source_selection_staleness_and_switch_off(node_module,monkeypatch):
    node,_=make_node(node_module,monkeypatch)
    node._patrol_imu_heading=("imu",.3,9.99)
    node._last_odom_time=9.98
    assert node._heading_snapshot(10.,PatrolTuning(),(0.,0.,.2))[0]=="imu"
    assert node._heading_snapshot(10.,replace(PatrolTuning(),roundabout_heading_source=2),(0.,0.,.2))[0]=="odom"
    assert node._heading_snapshot(11.,PatrolTuning(),(0.,0.,.2)) is None
    assert node._heading_snapshot(10.,replace(PatrolTuning(),roundabout_heading_enabled=0),(0.,0.,.2)) is None


def test_v11_follow_multiplier_applies_only_to_reliable_follow(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    clock=[10.]
    monkeypatch.setattr(node_module.time,"monotonic",lambda:clock[0])
    r=RoundaboutRouter()
    r._phase("FOLLOW",9.)
    r.follow_stable_since=9.
    r.branch_acquired=True
    node.vision.router=r
    node.vision.estimate=lambda *_a,**_kw:PathEstimate(True,-.2,.9,route_observed=True,route_locked=True)
    node.patrol_controller.last_time=9.97
    node.patrol_controller.last_command=(.65,0.)
    for i in range(25):
        clock[0]=10.+i*.04
        r.measured_turn.source="imu"
        r.measured_turn.stamp=clock[0]
        r.measured_turn.angle=math.radians(120.)
        node._frame_sequence=i+1
        node._latest_frame_time=clock[0]
        node._run_control_cycle()
    assert commands[-1][0]==pytest.approx(.845)
    assert commands[-1][1]>0
    r._phase("SEEK",clock[0])
    clock[0]+=.04
    node._frame_sequence+=1
    node._latest_frame_time=clock[0]
    node._run_control_cycle()
    assert commands[-1][0]<.845  # only confirmed FOLLOW, not the second lap generally


def test_v11_old_loss_boundary_degrades_instead_of_latching_and_can_recover():
    c,t=cruise_controller()
    for now in np.arange(2.03,6.,.03):
        v,w,mode=c.command(PathEstimate(False),float(now),t)
        assert v>0 and not c.fault_latched
    assert mode=="degraded_search" and v==pytest.approx(.12)
    for now in (6.01,6.04,6.07):
        v,w,mode=c.command(PathEstimate(True,-.2,.9),now,t)
    assert mode=="fixed_speed" and c.loss_started_at is None and not c.fault_latched


def test_v11_final_no_localization_protection_remains_after_degraded_search():
    c,t=cruise_controller(search_hard_max_distance_m=100.)
    for i,now in enumerate(np.arange(2.03,10.1,.03)):
        path=PathEstimate(i%3!=0,-.2,.9)  # flicker never reaches 3 consecutive observations
        result=c.command(path,float(now),t)
    assert result==(0.,0.,"localization_fault_latched") and c.fault_reason=="hard_time_limit"


def test_v11_crosswalk_cooldown_keeps_tracking_and_cannot_reconsume_zebra():
    r,t=RoundaboutRouter(),PatrolTuning()
    assert r.course.confirm_ring() and r.course.confirm_zebra()
    r.crosswalk_count=1
    r.last_crosswalk_at=0.
    r._phase("CW_STOP",0.)
    r.observe(False,0.,3.1,t)
    assert r.phase=="CW_CLEAR" and r.motion_override(3.1,t) is None
    for now in (3.2,4.,5.,6.2):
        r.observe(False,0.,now,t,(.65,.4),crosswalk=True)
        assert r.crosswalk_count==1
    assert r.phase=="SEEK"
    r._phase("EXIT",6.3)
    r.ring_route_acquired=True  # fixture for second acquired route
    r._release_ring(6.3,t,"visual_exit")
    assert r.crosswalk_block_reason(8.,t)=="crosswalk_cooldown"
    assert r.crosswalk_armed(8.2,t)


def test_v11_node_clear_corrects_real_line_instead_of_holding_heading(node_module,monkeypatch):
    node,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter()
    r._phase("CW_CLEAR",9.)
    r.heading_target=0.
    node.vision.router=r
    node.vision.estimate=lambda *_a,**_kw:PathEstimate(True,.65,.9)
    node.patrol_controller.last_command=(.65,0.)
    node.patrol_controller.last_time=9.97
    node._run_control_cycle()
    assert commands[-1][0]==.65 and commands[-1][1]<0


@pytest.mark.parametrize("changes",[
    {"search_degraded_enabled":2},{"search_degraded_speed_m_s":0},
    {"search_hard_max_seconds":3.},{"search_hard_max_distance_m":1.8},
    {"roundabout_heading_enabled":2},{"roundabout_heading_source":3},
    {"roundabout_heading_timeout_seconds":0},{"roundabout_heading_max_rate_rad_s":0},
    {"roundabout_entry_turn_rad_s":0},{"roundabout_entry_acquire_yaw_deg":361},
    {"roundabout_entry_min_observed_span_ratio":0},{"roundabout_entry_check_seconds":0},
    {"roundabout_entry_required_turn_deg":-1},{"roundabout_entry_acquire_max_seconds":1.},
    {"roundabout_loop_min_turn_deg":100},{"roundabout_loop_return_tolerance_deg":90},
    {"roundabout_loop_min_distance_m":-1},{"roundabout_loop_min_distance_m":0},{"roundabout_loop_min_coherence":0},
    {"roundabout_visual_exit_min_turn_deg":361},{"roundabout_follow_speed_multiplier":0},
    {"roundabout_scale_tracking_turn":2},{"crosswalk_resume_tracking":2},
    {"crosswalk_ignore_after_stop_seconds":-1},
])
def test_v11_new_realtime_values_reject_invalid_complete_save(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v14_unconfirmed_yaw_release_cannot_start_old_exit_bridge():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.measured_turn.source="imu"
    r.measured_turn.last_yaw=0.
    r.measured_turn.angle=2*math.pi
    r._release_ring(1.,t,"yaw_loop_release")
    turn=r.exit_search_turn(PathEstimate(False),1.1,t,("imu",-.1,1.1))
    assert turn is None and not r.crosswalk_armed(1.1,t)
    assert r.exit_search_turn(PathEstimate(True),1.1,t,("imu",-.1,1.1)) is None
    assert r.exit_search_turn(PathEstimate(False),1.1,t,("odom",-.1,1.1)) is None
    assert r.exit_search_turn(PathEstimate(False),1.4,t,("imu",-.1,1.)) is None
    assert r.exit_search_turn(PathEstimate(False),1.81,t,("imu",-.1,1.81)) is None
    r.pause(1.2)
    assert r.exit_search_turn(PathEstimate(False),1.3,t,("imu",-.1,1.3)) is None


def test_v11_heading_bridge_cannot_renew_search_protection_or_force_valid_path():
    c,t=cruise_controller(search_hard_max_distance_m=100.)
    for now in np.arange(2.03,10.1,.03):
        result=c.command(PathEstimate(False),float(now),t,search_heading_turn=0.)
    assert result==(0.,0.,"localization_fault_latched") and c.fault_reason=="hard_time_limit"
    c,_=cruise_controller()
    result=c.command(PathEstimate(True,.5,.9),2.03,t,search_heading_turn=1.)
    assert c.raw_angular<0 and result[2]=="fixed_speed"


@pytest.mark.parametrize("changes",[
    {"roundabout_yaw_exit_recovery_seconds":-1},{"roundabout_yaw_exit_recovery_seconds":2.1},
    {"roundabout_yaw_exit_heading_gain":0},
])
def test_v11_heading_bridge_snapshot_validation(changes):
    with pytest.raises(ValueError):
        replace(PatrolTuning(),**changes).validate()


def test_v11_entry_uses_fresh_measured_heading_over_command_progress():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("ENTRY",0.)
    r.left_yaw=2.  # stale/command progress is not permission to stop turning
    r.measured_turn.source="imu"
    r.measured_turn.stamp=.5
    r.measured_turn.angle=.1
    assert r.motion_override(.5,t)[1]==1.2


def test_v11_multiplier_arithmetic_overflow_is_rejected_not_silently_clipped():
    with pytest.raises(ValueError):
        replace(PatrolTuning(),fixed_speed_m_s=1e308,roundabout_follow_speed_multiplier=10.).validate()


def moving_entry_chain(dy=0.,kind="left"):
    xs=[120.,130.,140.] if kind=="left" else [195.,200.,205.] if kind=="right" else [130.,130.,130.]
    return [(x,y+dy,10.,12.,100.) for x,y in zip(xs,(100.,130.,160.))]


def main_route_points():
    return np.array([[170.,85.],[170.,130.],[170.,175.],[170.,225.]])


def test_v14_entry_proof_needs_moving_same_left_branch_not_blue():
    p,t=EntryApproachProof(),PatrolTuning()
    ready=[]
    for i in range(16):
        ready.append(p.update(moving_entry_chain(i*1.6),main_route_points(),320,240,i/30,t))
    assert not any(ready[:11]) and ready[-1]
    assert p.reason=="entry_left_branch_ready" and p.advance>=t.roundabout_entry_near_advance_ratio


def test_v14_actual_vision_moving_left_chain_can_enter_with_observed_mainline(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    phases=[]
    for i in range(27):
        frame=np.full((240,320,3),245,np.uint8)
        cv2.line(frame,(180,0),(180,239),(0,0,0),7)
        for x,y,w,h,_ in moving_entry_chain(i*.9):
            cv2.rectangle(frame,(int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),(0,0,0),-1)
        v.motion_command=(.65,0.)
        v.estimate(frame,t,i/30)
        phases.append(v.router.phase)
        if v.router.phase=="ENTRY":break
    assert set(phases)=={"SEEK"}  # old three-block proof cannot bypass VV min four
    assert v.router.course.rings==[False,False]  # acquisition is not completion


@pytest.mark.parametrize("case",["static","right","parallel","no_reference","one_reference","stale","jump","reverse_time","stopped","unobserved_far"])
def test_v14_entry_proof_rejects_unqualified_geometry(case):
    p,t=EntryApproachProof(),PatrolTuning()
    seen=[]
    for i in range(18):
        now=i/30
        chain=moving_entry_chain(0. if case=="static" else i*1.3,case if case in ("right","parallel") else "left")
        ref=main_route_points()
        if case=="no_reference":ref=None
        if case=="one_reference":ref=ref[:1]
        if case=="unobserved_far":ref=ref[-2:]
        if case=="stale":now=i*.30
        if case=="reverse_time":now=-i/30
        if case=="jump":chain=[(x+(60 if i%2 else 0),y,w,h,a) for x,y,w,h,a in chain]
        seen.append(p.update(chain,ref,320,240,now,t,moving=case!="stopped"))
    assert not any(seen),case


def test_v14_duplicate_entry_frames_cannot_count_time_or_frames():
    p,t=EntryApproachProof(),PatrolTuning()
    for _ in range(30):
        assert not p.update(moving_entry_chain(20.),main_route_points(),320,240,1.,t)
    assert p.frames==1


def test_v14_near_gate_without_geometric_permission_never_starts_entry():
    r=RoundaboutRouter();t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    for i in range(45):
        r.observe(True,.99,i/30,t,(.65,0.),gate_near=.9)
    assert r.phase=="SEEK" and r.near_frames==0 and not r.left_lock


@pytest.mark.parametrize("source",["entry_no_turn_abort","entry_no_route_abort","time_fallback","yaw_exit_unconfirmed","yaw_loop_release","zebra_evidence_fallback","route_budget_retry"])
def test_v14_control_release_never_creates_ring_zebra_or_finish_receipt(source):
    r,t=RoundaboutRouter(),PatrolTuning()
    r.started_at=0.;r._phase("ENTRY",1.)
    r._release_ring(2.,t,source)
    assert r.phase=="SEEK" and not r.course.rings[0] and r.after_ring_at is None
    for now in (2.01,2.04,3.,20.):
        r.observe(False,0.,now,t,(.65,0.),crosswalk=True)
        assert r.crosswalk_count==0 and not r.crosswalk_armed(now,t)
    assert not r.course.finish_allowed and r.motion_override(20.,t) is None


def test_v14_receipts_require_ring_for_each_zebra_and_both_laps_for_finish():
    c=CourseProgress()
    assert c.steps==["SKIPPED_TEST","PENDING"]
    assert not c.confirm_zebra() and not c.next_lap() and not c.finish_allowed
    assert c.confirm_ring() and not c.confirm_ring()
    assert c.confirm_zebra() and not c.confirm_zebra() and not c.finish_allowed
    assert c.next_lap() and c.steps==["SKIPPED_TEST","SKIPPED_TEST"]
    assert not c.confirm_zebra() and not c.finish_allowed
    assert c.confirm_ring() and c.confirm_zebra() and c.finish_allowed
    assert not c.next_lap() and not c.confirm_zebra()


@pytest.mark.parametrize("acquired",[False,True])
def test_v14_exit_must_have_acquired_route_and_actual_exit_phase(acquired):
    r,t=RoundaboutRouter(),PatrolTuning()
    r.ring_route_acquired=acquired
    assert not r._release_ring(1.,t,"visual_exit")  # SEEK is never a valid exit
    r._phase("EXIT",1.)
    if acquired:
        assert r._release_ring(1.1,t,"visual_exit")
        assert r.completed==1 and r.course.rings==[True,False]
    else:
        assert not r._release_ring(1.1,t,"visual_exit") and r.completed==0


def test_v14_numeric_counts_and_fake_seek_crosswalk_cannot_authorize_finish():
    r,t=RoundaboutRouter(),PatrolTuning()
    r.crosswalk_count=r.completed=2
    r.after_ring_at=0.;r._phase("SEEK_CROSSWALK",0.)
    for now in (5.,5.03,5.06):r.observe(False,0.,now,t,(.65,0.),crosswalk=True)
    assert r.phase=="SEEK_CROSSWALK" and not r.course.finish_allowed
    assert r.crosswalk_block_reason(5.06,t)=="lap1_ring_not_confirmed"


def test_v14_unobserved_exit_or_forged_finish_phase_cannot_move_course_forward():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("EXIT",0.);r.ring_route_acquired=True;r.phase_distance=1.
    for now in (1.,1.03,1.06):
        r.accept_path(PathEstimate(True,route_locked=True,route_observed=False),now,t)
    assert r.phase=="EXIT" and not any(r.course.rings)
    r._phase("FINISH_RUN",2.)
    assert r.motion_override(2.,t)==(0.,0.,"unauthorized_finish")
    assert r.fault=="finish_without_ordered_receipts"


def test_v14_stale_or_duplicate_zebra_frames_cannot_consume_receipt():
    r,t=RoundaboutRouter(),PatrolTuning()
    assert r.course.confirm_ring()
    r.after_ring_at=0.;r._phase("SEEK_CROSSWALK",0.)
    for now in (2.,2.,2.5,3.):
        r.observe(False,0.,now,t,crosswalk=True)
        assert r.crosswalk_count==0
    r.observe(False,0.,3.03,t,crosswalk=True)
    assert r.crosswalk_count==1 and r.course.zebras==[True,False]


@pytest.mark.parametrize("changes",[
    {"roundabout_entry_observe_seconds":0},{"roundabout_entry_near_advance_ratio":0},
    {"roundabout_entry_left_gap_ratio":.5},{"roundabout_entry_branch_divergence_ratio":-1},
    {"roundabout_retry_seconds":0},{"roundabout_retry_distance_m":-1},
    {"crosswalk_visual_release_enabled":1},{"crosswalk_resume_tracking":0},
])
def test_v14_new_parameters_and_retired_shortcuts_are_validated(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()


def test_v14_retry_budget_and_blocks_use_latest_hot_values_without_restarting_task():
    r=RoundaboutRouter();t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    r.started_at=0.;r.distance=2.
    r._release_ring(5.,t,"entry_no_route_abort")
    r.distance=2.3
    assert not r.gate_detection_allowed(6.,t)
    updated=replace(t,roundabout_retry_seconds=.5,roundabout_retry_distance_m=.2,roundabout_first_timeout_seconds=20.)
    assert r.gate_detection_allowed(6.,updated) and r.ring_deadline(updated)==25.
    r.pause(7.);r.resume(17.)
    assert r.ring_deadline(updated)==35. and not r.course.rings[0]


def approach_path(x=170.,source="black",**kwargs):
    pts=main_route_points().copy();pts[:,0]=x
    e=2*(x/320-.5)
    return PathEstimate(True,e,.9,points=pts,near_error=e,mid_error=e,route_observed=True,source=source,**kwargs)


def test_v15_independent_reference_survives_bounded_missing_frames_not_chain_refresh():
    m,t=ApproachRouteMemory(),PatrolTuning()
    assert m.remember(approach_path(),moving_entry_chain(),320,0.,t)
    assert m.reference(.6,t) is not None
    assert not m.remember(approach_path(130.),[(130.,100.,10,12,100),(130.,140.,10,12,100),(130.,170.,10,12,100)],320,.7,t)
    assert m.at==0. and m.reference(.81,t) is None


@pytest.mark.parametrize("kind",["rescue","handoff","invalid","one_point","no_overlap"])
def test_v15_weak_or_chain_source_cannot_refresh_reference(kind):
    m,t=ApproachRouteMemory(),PatrolTuning()
    p=approach_path()
    if kind=="rescue":p.rescue=True
    if kind=="handoff":p.handoff=True
    if kind=="invalid":p.valid=False
    if kind=="one_point":p.points=p.points[:1]
    if kind=="no_overlap":p.points[:,1]+=300
    assert not m.remember(p,moving_entry_chain(),320,0.,t)
    assert m.reference(.1,t) is None


def test_v15_approach_same_route_wins_against_opposite_branch_then_expires():
    m,t=ApproachRouteMemory(),PatrolTuning()
    m.remember(approach_path(),[],320,0.,t)
    black=approach_path(50.,source="black");road=approach_path(175.,source="road")
    picked=m.select(black,road,.1,t)
    assert picked.source=="road" and picked.error>0 and black.valid
    missing=m.select(PathEstimate(False),PathEstimate(False),.2,t)
    assert missing is not None and not missing.valid and missing.reason=="approach_route_gap"
    assert m.select(black,PathEstimate(False),.31,t) is None  # no indefinite veto
    assert m.select(black,road,.81,t) is None


def test_v15_approach_proof_retains_short_chain_gap_but_never_confirms_missing():
    p,t=EntryApproachProof(),PatrolTuning()
    for i in range(8):p.update(moving_entry_chain(i*1.3),main_route_points(),320,240,i/30,t)
    frames=p.frames;at=p.at
    assert not p.update([],None,320,240,.32,t) and p.frames==frames and p.at==at
    result=p.update(moving_entry_chain(22.),main_route_points(),320,240,.42,t)
    assert result and p.frames==frames+1
    assert not p.update([],None,320,240,.8,t) and p.frames==0


@pytest.mark.parametrize("phase",["ENTRY","EXIT"])
def test_v15_short_missing_path_never_awards_phase_or_renews_observation(phase):
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase(phase,0.);r.left_yaw=1.;r.phase_distance=1.;r.ring_route_acquired=True
    p=approach_path(160.,route_locked=True)
    r.accept_path(p,1.,t);r.accept_path(p,1.03,t)
    before=(r.path_streak,r.last_good_path_at)
    r.accept_path(PathEstimate(False),1.15,t)
    assert (r.path_streak,r.last_good_path_at)==before and r.phase==phase
    assert not any(r.course.rings)
    r.accept_path(PathEstimate(False),1.4,t)
    assert r.path_streak==0 and r.phase==phase


def observed_departure_router():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.);r.ring_route_acquired=True;r.branch_acquired=True;r.gate_cleared=True
    r.phase_distance=1.
    r.measured_turn.source="imu";r.measured_turn.reliable=True
    r.measured_turn.angle=math.radians(270.)
    return r,t


def test_v15_observed_departure_nominates_exit_then_real_path_counts_once():
    r,t=observed_departure_router();p=approach_path(160.,route_locked=True)
    for now in np.arange(3.,3.51,.03):
        r.measured_turn.stamp=float(now)
        r.accept_path(p,float(now),t)
        if r.phase=="EXIT":break
    assert r.phase=="EXIT" and r.completed==0 and not r.crosswalk_armed(now,t)
    r.phase_distance=.5
    for dt in (.03,.06,.09):r.accept_path(p,float(now)+dt,t)
    assert r.course.rings==[True,False] and r.completed==1 and r.phase=="SEEK_CROSSWALK"
    r.accept_path(p,float(now)+.12,t)
    assert r.completed==1


@pytest.mark.parametrize("bad",["no_measured_turn","small_turn","stale","not_acquired","uncleared_gate","fork","near_gap","rescue","offcenter","curve","no_observed"])
def test_v15_departure_cannot_be_time_or_ordinary_bend_shortcut(bad):
    r,t=observed_departure_router();p=approach_path(160.,route_locked=True)
    if bad=="no_measured_turn":r.measured_turn.reliable=False
    if bad=="small_turn":r.measured_turn.angle=.8
    if bad=="not_acquired":r.ring_route_acquired=False
    if bad=="uncleared_gate":r.gate_cleared=False
    if bad=="fork":p.branch_rows=3
    if bad=="near_gap":p.near_gap_ratio=.1
    if bad=="rescue":p.rescue=True
    if bad=="offcenter":p.near_error=p.mid_error=.6
    if bad=="curve":p.mid_error=.4
    if bad=="no_observed":p.route_observed=False
    for now in np.arange(3.,4.,.03):
        r.measured_turn.stamp=float(now)- (1. if bad=="stale" else 0.)
        r.accept_path(p,float(now),t)
    assert r.phase=="FOLLOW" and not any(r.course.rings)


@pytest.mark.parametrize("changes",[
    {"roundabout_approach_memory_seconds":0},{"roundabout_approach_gap_seconds":1.},
    {"roundabout_approach_route_delta":0},{"roundabout_path_dropout_seconds":3.},
    {"roundabout_observed_exit_seconds":0},{"roundabout_observed_exit_agreement":2.},
])
def test_v15_bounded_new_values_are_validated(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()


def test_v15_two_observed_exits_authorize_zebras_in_order_and_distance_finish():
    r,t=observed_departure_router()
    p=approach_path(160.,route_locked=True)
    first=True;now=3.
    for lap in (0,1):
        assert r.course.lap==lap
        # Set an acquired FOLLOW fixture; departure itself must produce EXIT
        # and the actual exit path must generate the ring receipt.
        if not first:
            r._phase("FOLLOW",now);r.ring_route_acquired=True;r.branch_acquired=True;r.gate_cleared=True
            r.phase_distance=1.;r.phase_at=now-3.;r.measured_turn.angle=math.radians(270.)
        for i in range(20):
            now+=.03;r.measured_turn.stamp=now
            r.accept_path(p,now,t)
            if r.phase=="EXIT":break
        assert r.phase=="EXIT" and r.crosswalk_count==lap
        r.phase_distance=.5
        for _ in range(3):now+=.03;r.accept_path(p,now,t)
        assert r.phase=="SEEK_CROSSWALK" and r.course.rings[lap]
        now+=1.6
        for _ in range(2):now+=.03;r.observe(False,0.,now,t,(.65,0.),crosswalk=True)
        assert r.crosswalk_count==lap+1
        if first:
            r.phase_distance=.26;now+=.03;r.observe(False,0.,now,t,(.65,0.))
            assert r.phase=="CW_STOP"
            now+=3.01;r.observe(False,0.,now,t)
            assert r.phase=="CW_CLEAR" and r.motion_override(now,t) is None
            now+=3.01;r.observe(False,0.,now,t,(.65,0.))
            assert r.phase=="SEEK" and r.course.lap==1
            first=False
        else:
            assert r.phase=="FINISH_RUN" and r.finish_authorized
            r.phase_distance=1.01;now+=.03;r.observe(False,0.,now,t,(.65,0.))
            assert r.phase=="FINISHED" and r.motion_override(now,t)==(0.,0.,"finished")
