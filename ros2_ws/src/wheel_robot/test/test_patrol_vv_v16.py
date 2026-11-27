"""V16 deployed VV entry integration; no ROS/motor actuation."""
from dataclasses import replace
import ast
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from test_patrol_contour import load_actual_vision_module
from patrol_contour_runtime import PatrolTuning, RoundaboutRouter, PathEstimate, FixedSpeedController, SearchSideMemory
from patrol_dash_entry import DashObservation, detect_vv_dashes


def masks(kind="diverging"):
    dark=np.zeros((240,320),np.uint8)
    for i,y in enumerate((210,185,160,135,110,85)):
        gap=18+12*i if kind=="diverging" else 78-12*i
        for x in (175,175-gap):
            cv2.rectangle(dark,(x-4,y-5),(x+4,y+5),255,-1)
    return dark,np.where(dark==0,255,0).astype(np.uint8)


def cue(kind="diverging",t=None):
    return detect_vv_dashes(*masks(kind),t or PatrolTuning())


def armed():
    return RoundaboutRouter(),replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)


def pass_five(r,t):
    for i in range(15):
        r.observe(True,1.,i*.1,t,(.65,0.),dash_cue=cue("converging",t))
    assert r.phase=="APPROACH"


def test_entry_and_exit_directions_are_separate_single_frame_cues():
    a,b=cue(),cue("converging")
    assert a.valid and a.blocks>=4 and a.branch.valid and not a.exit_branch.valid
    assert b.valid and b.exit_branch.valid and not b.branch.valid
    assert PatrolTuning().vv_dash_confirm_frames==1


@pytest.mark.parametrize("kind",["diverging","converging"])
@pytest.mark.parametrize("slant",[-.2,.2])
def test_fork_direction_is_relative_to_main_not_screen_heading(kind,slant):
    dark,_=masks(kind)
    transform=np.float32([[1,slant,-160*slant],[0,1,0]])
    dark=cv2.warpAffine(dark,transform,(320,240))
    result=detect_vv_dashes(dark,np.where(dark==0,255,0).astype(np.uint8),PatrolTuning())
    assert result.valid
    assert result.branch.valid==(kind=="diverging")
    assert result.exit_branch.valid==(kind=="converging")


def test_cooldown_has_no_motion_override_and_keeps_baseline_controller_moving():
    r,t=armed();c=FixedSpeedController()
    for i in range(100):c.command(PathEstimate(True,0.,.9),i/30,t)
    for i in range(1,8):
        now=4.+i*.1
        r.observe(True,1.,now,t,(.65,0.),dash_cue=cue("converging",t))
        if r.phase=="APPROACH":
            assert r.motion_override(now,t) is None
            assert c.command(PathEstimate(True,0.,.9),now,t)[0]==pytest.approx(.65)


@pytest.mark.parametrize("kind",["solid","zebra","few","blue","right"])
def test_reject_non_entry_cues(kind):
    dark,white=masks()
    if kind in ("solid","zebra","few"):
        dark[:]=0
        if kind=="solid":cv2.line(dark,(175,220),(175,65),255,8)
        if kind=="zebra":
            for y in (85,115,145,175,205):cv2.rectangle(dark,(65,y),(245,y+9),255,-1)
        if kind=="few":cv2.rectangle(dark,(170,200),(180,210),255,-1)
        white=np.where(dark==0,255,0).astype(np.uint8)
    if kind=="blue":white[:]=0
    if kind=="right":dark=cv2.flip(dark,1);white=cv2.flip(white,1)
    got=detect_vv_dashes(dark,white,PatrolTuning())
    assert not got.valid or (kind=="right" and not got.branch.valid)


def test_entry_accepts_a_left_curving_main_chain_without_a_branch():
    """2026-11-23: some entry poses only show a left-curving main dash chain."""
    r,t=armed()
    main=PathEstimate(True,near_error=-.12,mid_error=-.25,route_observed=True)
    c=DashObservation(valid=True,blocks=4,near_ratio=.88,main=main)
    r.observe(False,0.,0.,t,(.65,0.))
    r.observe(True,1.,.10,t,(.65,0.),dash_cue=c)
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.01
    r.observe(True,1.,.20,t,(.65,0.),dash_cue=c)
    r.observe(True,1.,.30,t,(.65,0.),dash_cue=c)
    assert r.phase=="ENTRY" and r.entry_detection_reason=="vv_main_curve_left_probe"


def test_main_curve_fallback_needs_the_dashes_to_be_close():
    r,t=armed()
    main=PathEstimate(True,near_error=-.12,mid_error=-.25,route_observed=True)
    far=DashObservation(valid=True,blocks=4,near_ratio=.40,main=main)
    r.observe(False,0.,0.,t,(.65,0.))
    r.observe(True,1.,.10,t,(.65,0.),dash_cue=far)
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.01
    for i in range(3):
        r.observe(True,1.,.2+i*.1,t,(.65,0.),dash_cue=far)
    assert r.phase=="APPROACH"


def test_main_curve_fallback_rejects_an_already_far_left_line():
    """If the main chain is already at -0.55 it is the car's own deviation, not
    a ring entry line (2026-11-23 19:15 false probe)."""
    r,t=armed()
    main=PathEstimate(True,near_error=-.55,mid_error=-.64,route_observed=True)
    c=DashObservation(valid=True,blocks=4,near_ratio=.89,main=main)
    r.observe(False,0.,0.,t,(.65,0.))
    r.observe(True,1.,.10,t,(.65,0.),dash_cue=c)
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.01
    for i in range(4):
        r.observe(True,1.,.2+i*.1,t,(.65,0.),dash_cue=c)
    assert r.phase=="APPROACH"


def test_first_marker_never_immediately_turns_even_if_it_looks_like_entry():
    r,t=armed()
    r.observe(False,0.,0.,t,(.65,0.))
    r.observe(True,1.,.1,t,(.65,0.),dash_cue=cue(t=t))
    assert r.phase=="APPROACH" and r.first_marker_at==.1
    assert r.motion_override(.1,t) is None














@pytest.mark.parametrize("kind,want_exit",[("converging",True),("diverging",False)])
def test_actual_follow_gate_uses_return_branch_not_entry(monkeypatch,kind,want_exit):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision();_,t=armed()
    t=replace(t,roundabout_heading_enabled=0)
    r=v.router;r._phase("FOLLOW",0.);r.started_at=0.;r.last_time=3.
    r.phase_distance=1.;r.ring_yaw=4.5;r.gate_cleared=True
    r.branch_acquired=r.ring_route_acquired=True
    v.ring_phase="FOLLOW";v.motion_command=(.3,0.)
    dark,_=masks(kind);frame=cv2.cvtColor(255-dark,cv2.COLOR_GRAY2BGR)
    for i in range(4):v.estimate(frame,t,3.1+i*.1)
    assert (r.phase=="EXIT")==want_exit
    assert not any(r.course.rings)  # EXIT candidate still needs actual departure


@pytest.mark.parametrize("changes",[
    {"vv_dash_min_blocks":1},{"vv_dash_confirm_frames":0},{"vv_dash_confirm_frames":1.5},
    {"vv_dash_roi_top_ratio":.95},{"vv_dash_max_x_spread_px80":0},
    {"vv_dash_min_gap_rows56":11},{"vv_dash_entry_near_ratio":1.},
    {"roundabout_first_marker_cooldown_seconds":0},{"roundabout_first_marker_pass_distance_m":0},
    {"roundabout_first_marker_cooldown_seconds":8.0},
])
def test_invalid_vv_parameters_reject_entire_snapshot(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()


def test_reviewed_ordinary_algorithms_and_numeric_speed_defaults():
    scripts=Path(__file__).resolve().parents[1]/"scripts"
    text=(scripts/"patrol_contour_runtime.py").read_text(encoding="utf-8-sig")
    tree=ast.parse(text)
    for name,digest in (("PathSelector","8e5d914aff1fe47fd4039d05214dae936fbf56fd79d36ade94128af131015224"),
                        ("FixedSpeedController","55b2b8c1dff6f20ec57c842309786a1754fa61e77726baa4cf1ce4a7d4813f37")):
        item=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
        assert hashlib.sha256(ast.get_source_segment(text,item).strip().encode()).hexdigest()==digest
    t=PatrolTuning();c=FixedSpeedController()
    for i in range(100):v,_,_=c.command(PathEstimate(True,0.,.9),i/30,t)
    assert v==pytest.approx(.65) and t.roundabout_follow_speed_multiplier==1.3


def test_current_incremental_manifest_includes_vv_and_matches_files():
    root=Path(__file__).resolve().parents[4]
    manifest=root/"checksums/PATROL_TEST_ADDITIONS_SHA256SUMS.txt"
    assert b"\r" not in manifest.read_bytes()  # Linux sha256sum/SFTP-safe lines
    entries={line.split(maxsplit=1)[1]:line.split(maxsplit=1)[0]
             for line in manifest.read_text(encoding="utf-8-sig").splitlines() if line.strip() and not line.startswith("#")}
    assert "ros2_ws/src/wheel_robot/scripts/patrol_dash_entry.py" in entries
    assert "ros2_ws/src/wheel_robot/test/test_patrol_vv_v16.py" in entries
    for relative,digest in entries.items():
        assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==digest,relative


def branch_cue(branch=True,exit_branch=False):
    return DashObservation(valid=True,blocks=4,branch=PathEstimate(branch),
                           exit_branch=PathEstimate(exit_branch))


def failed_approach_inside_retry_lock():
    """Real sequence: 0.60m APPROACH limit -> SEEK with retry lock still active."""
    r,t=armed()
    r.observe(False,0.,0.,t,(.3,0.))
    r.observe(True,1.,.1,t,(.3,0.),dash_cue=DashObservation(valid=True,blocks=4))
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_approach_max_m
    r.observe(True,1.,.2,t,(.3,0.),dash_cue=DashObservation(valid=True,blocks=4))
    assert r.phase=="SEEK" and r.retry_started_at is not None
    assert not r.gate_detection_allowed(.3,t)
    return r,t


def test_confirmed_left_branch_rearms_inside_retry_lock_and_enters():
    r,t=failed_approach_inside_retry_lock()
    # A plain dash frame must still respect the failed-attempt lock.
    r.observe(True,1.,.3,t,(.3,0.),dash_cue=DashObservation(valid=True,blocks=4))
    assert r.phase=="SEEK"
    # The fork that appears inside the lock is new evidence: latch immediately.
    r.observe(True,1.,.4,t,(.3,0.),dash_cue=branch_cue())
    assert r.phase=="APPROACH"
    assert r.entry_detection_reason=="vv_rearm_on_confirmed_left_branch"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.001
    r.observe(True,1.,.5,t,(.3,0.),dash_cue=branch_cue())
    assert r.phase=="ENTRY"


def test_branch_rearm_switch_off_keeps_old_lock_and_detection_widens():
    r,t=failed_approach_inside_retry_lock()
    off=replace(t,roundabout_entry_branch_rearm_enabled=0)
    assert r.detection_allowed(.3,off)
    r.observe(True,1.,.3,off,(.3,0.),dash_cue=branch_cue())
    assert r.phase=="SEEK"
    # Not a hidden latch path: the same cue still cannot bypass the lock.
    assert not r.gate_detection_allowed(.3,off,branch_confirmed=True)


def test_branch_rearm_never_bypasses_initial_arm_gate():
    r=RoundaboutRouter()
    r.observe(False,0.,0.,PatrolTuning(),(.3,0.))
    r.distance=2.
    assert not r.gate_detection_allowed(1.,PatrolTuning(),branch_confirmed=True)
    r.observe(True,1.,1.,PatrolTuning(),(.3,0.),dash_cue=branch_cue())
    assert r.phase=="SEEK"


def test_converging_exit_branch_does_not_rearm():
    r,t=failed_approach_inside_retry_lock()
    r.observe(True,1.,.3,t,(.3,0.),dash_cue=branch_cue(branch=False,exit_branch=True))
    assert r.phase=="SEEK"


def straight_dashes():
    dark=np.zeros((240,320),np.uint8)
    for y in (210,185,160,135,110,85):
        cv2.rectangle(dark,(171,y-5),(179,y+5),255,-1)
    return dark,np.where(dark==0,255,0).astype(np.uint8)


def test_actual_vision_measures_then_rearms_on_branch_inside_lock(monkeypatch):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision();_,t=armed()
    r=v.router;r.started_at=0.;r.distance=4.;r.phase="SEEK";r.phase_at=0.
    r.retry_started_at=2.0;r.retry_origin_distance=3.5
    r.entry_block_until=4.0;r.entry_block_distance=4.0
    v.motion_command=(.3,0.)
    assert not r.gate_detection_allowed(2.1,t) and r.detection_allowed(2.1,t)
    straight,_=straight_dashes()
    v.estimate(cv2.cvtColor(255-straight,cv2.COLOR_GRAY2BGR),t,2.1)
    v.estimate(cv2.cvtColor(255-straight,cv2.COLOR_GRAY2BGR),t,2.13)
    # Measurement runs during the lock, but a plain chain must not latch.
    assert v.dash_observation.valid and r.phase=="SEEK"
    dark,_=masks()
    v.estimate(cv2.cvtColor(255-dark,cv2.COLOR_GRAY2BGR),t,2.16)
    assert r.phase=="APPROACH"
    assert r.entry_detection_reason=="vv_rearm_on_confirmed_left_branch"


def entry_probe_router():
    r=RoundaboutRouter();r.started_at=0.;r._phase("ENTRY",0.)
    t=replace(PatrolTuning(),ring_entry_probe_seconds=.5,
              ring_entry_probe_max_seconds=.9,ring_entry_probe_max_turn_deg=60.)
    return r,t


def test_entry_probe_extends_only_with_evidence_and_stops_at_turn_limit():
    r,t=entry_probe_router()
    assert r.motion_override(.6,t,path=None,entry_branch_visible=True)[2]=="roundabout_left_acquire"

    r,t=entry_probe_router()
    assert r.motion_override(.6,t,path=None,entry_branch_visible=False)==(0.,0.,"roundabout_entry_release")
    assert r.phase=="SEEK" and r.ring_release_source=="entry_probe_no_route"

    r,t=entry_probe_router()
    r.left_yaw=math.radians(60.5)
    assert r.motion_override(.6,t,path=None,entry_branch_visible=True)==(0.,0.,"roundabout_entry_release")
    assert r.ring_release_source=="entry_probe_turn_limit"


def test_entry_probe_extension_respects_max_seconds_and_switch():
    r,t=entry_probe_router()
    assert r.motion_override(.6,t,path=None,entry_branch_visible=True)[2]=="roundabout_left_acquire"
    assert r.entry_probe_extended
    assert r.motion_override(.95,t,path=None,entry_branch_visible=True)==(0.,0.,"roundabout_entry_release")
    assert r.ring_release_source=="entry_confirm_timeout"

    r,t=entry_probe_router()
    off=replace(t,ring_entry_probe_extend_enabled=0)
    assert r.motion_override(.8,off,path=None,entry_branch_visible=True)==(0.,0.,"roundabout_entry_release")


def test_entry_probe_holds_after_evidence_drops_once_dash_is_proven():
    """2026-11-25: the diverging branch only survives ~0.2s of a 1.2rad/s ramp.

    Once the VV dash was proven at ENTRY start the probe must keep turning up
    to the commit angle instead of dropping the whole ring task at the first
    evidence-free frame (that produced every entry_probe_no_route).
    """
    r,t=entry_probe_router()
    r.entry_dash_proven=True
    r.left_yaw=.20
    assert r.motion_override(.6,t,path=None,entry_branch_visible=False)[2]=="roundabout_left_acquire"
    assert r.phase=="ENTRY"
    # Without the dash proof the old evidence rule still releases immediately.
    r2,t2=entry_probe_router()
    r2.left_yaw=.20
    assert r2.motion_override(.6,t2,path=None,entry_branch_visible=False)==(0.,0.,"roundabout_entry_release")
    assert r2.ring_release_source=="entry_probe_no_route"


def test_committed_entry_hands_to_tracking_instead_of_dropping_the_ring():
    """A proven dash + a real entry turn is enough to keep the ring task alive.

    The remembered pre-turn guide cannot describe the post-turn pose, so the
    committed probe hands over to ordinary tracking; the bounded ENTRY/FOLLOW
    contract (turn/distance/stable route) still decides confirmation.
    """
    r,t=entry_probe_router()
    r.entry_dash_proven=True
    r.left_yaw=.55
    assert r.motion_override(.95,t,path=None,entry_branch_visible=False) is None
    assert r.phase=="ENTRY" and r.entry_visual_tracking
    assert r.route_status=="entry_commit_follow"


def test_entry_turn_commit_uses_measured_heading_when_fresh():
    r,t=entry_probe_router()
    assert not r.entry_turn_committed(.1,t)
    r.left_yaw=.5
    assert r.entry_turn_committed(.1,t)
    r._phase("SEEK",.2)
    assert not r.entry_turn_committed(.3,t)


def test_time_fallback_starts_a_new_failed_entry_epoch():
    r,t=RoundaboutRouter(),replace(PatrolTuning(),roundabout_max_failed_entries=2)
    r._phase("ENTRY",0.)
    assert not r._release_ring(1.,t,"entry_probe_no_route")
    assert r.failed_entries==1
    r._phase("APPROACH",2.)
    # A lap-scale deadline release is a new attempt epoch: earlier probe
    # failures must not accumulate into a freeze on a later ring encounter.
    assert not r._release_ring(3.,t,"time_fallback")
    assert r.failed_entries==0 and not r.fault


def entry_with_previous_steering(prev_w):
    r,t=armed()
    main=PathEstimate(True,near_error=-.12,mid_error=-.25,route_observed=True)
    branch=PathEstimate(True,near_error=-.35,mid_error=-.45,route_observed=True,route_locked=True)
    c=DashObservation(valid=True,blocks=4,near_ratio=.88,main=main,branch=branch)
    r.observe(False,0.,0.,t,(.65,prev_w))
    r.observe(True,1.,.10,t,(.65,prev_w),dash_cue=c)
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.01
    r.observe(True,1.,.20,t,(.65,prev_w),dash_cue=c)
    assert r.phase=="ENTRY"
    return r,t


def test_entry_follows_an_existing_left_curve_without_a_second_blind_ramp():
    """2026-11-25 20:16: the fork latched while already turning hard left.

    A second 1.2rad/s ramp then over-rotated into the island. When the previous
    command is already a strong left, ordinary tracking owns steering instead.
    """
    r,t=entry_with_previous_steering(1.2)
    assert r.entry_follow_only and r.entry_dash_proven
    assert r.entry_detection_reason=="vv_follow_existing_curve"
    assert r.motion_override(.30,t,path=None,entry_branch_visible=False) is None
    assert r.phase=="ENTRY" and r.route_status=="entry_follow_existing_left_curve"
    # Bounded: an unconfirmed follow-existing entry still expires.
    assert r.motion_override(.20+t.roundabout_entry_acquire_max_seconds,t,path=None)==(0.,0.,"roundabout_entry_release")
    assert r.phase=="SEEK" and r.ring_release_source=="entry_no_route_abort"


def test_entry_follows_an_existing_curve_from_cumulative_recent_turn():
    """20:38: the car already circles the roundabout dash circle when the fork
    latches; per-frame w is moderate but the last second accumulated ~0.5rad."""
    r,t=armed()
    main=PathEstimate(True,near_error=-.12,mid_error=-.25,route_observed=True)
    branch=PathEstimate(True,near_error=-.35,mid_error=-.45,route_observed=True,route_locked=True)
    c=DashObservation(valid=True,blocks=4,near_ratio=.88,main=main,branch=branch)
    r.observe(False,0.,0.,t,(.65,0.))
    for i in range(1,7):
        r.observe(False,0.,i*.1,t,(.65,.8))
    r.observe(True,1.,.70,t,(.65,.8),dash_cue=c)
    assert r.phase=="APPROACH" and r.recent_turn_abs>=.35
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.01
    r.observe(True,1.,.80,t,(.65,.8),dash_cue=c)
    assert r.phase=="ENTRY" and r.entry_follow_only
    assert r.entry_detection_reason=="vv_follow_existing_curve"


def test_entry_still_probes_when_approach_was_not_already_turning_left():
    r,t=entry_with_previous_steering(0.1)
    assert not r.entry_follow_only
    assert r.entry_detection_reason=="vv_distance_complete_left_probe"
    assert r.motion_override(.30,t,path=None,entry_branch_visible=False)[2]=="roundabout_left_acquire"


def test_failed_left_probe_keeps_an_immediate_left_search_window():
    r=RoundaboutRouter();t=replace(PatrolTuning(),roundabout_entry_search_seconds=.8)
    r._phase("ENTRY",0.)
    assert not r._release_ring(1.,t,"entry_probe_no_route")
    assert r.phase=="SEEK" and r.entry_search_side==1
    assert r.entry_search_active(1.1,t) and not r.entry_search_active(1.9,t)
    # Non-entry failures never arm it, and a real exit clears it.
    r2=RoundaboutRouter();r2._phase("APPROACH",0.)
    assert not r2._release_ring(1.,t,"time_fallback")
    assert r2.entry_search_until==0 and not r2.entry_search_active(1.1,t)
    r.entry_search_until=0
    r._phase("EXIT",2.);r.ring_route_acquired=True;r.course.lap=0
    assert r._release_ring(2.,t,"visual_exit")
    assert r.entry_search_until==0 and not r.entry_search_active(2.1,t)


def test_repeated_failed_probes_stop_instead_of_circling():
    r=RoundaboutRouter();t=replace(PatrolTuning(),roundabout_max_failed_entries=2)
    r._phase("ENTRY",0.)
    assert not r._release_ring(1.,t,"entry_probe_no_route")
    assert r.failed_entries==1 and not r.fault and r.entry_search_active(1.1,t)
    r._phase("ENTRY",2.)
    assert not r._release_ring(3.,t,"entry_probe_no_route")
    assert r.failed_entries==2 and r.fault=="ring_entry_unconfirmed"
    assert r.route_status=="ring_entry_unconfirmed" and not r.entry_search_active(3.1,t)
    # An approach-distance release is not a probe failure and never counts.
    r2=RoundaboutRouter();r2._phase("APPROACH",0.)
    assert not r2._release_ring(1.,t,"entry_approach_distance_limit")
    assert r2.failed_entries==0 and not r2.fault
    # Operator resume clears the fault and the retry count.
    r.pause(4.);r.resume(5.,clear_fault=True)
    assert r.fault=="" and r.failed_entries==0


def test_post_exit_guard_only_arms_after_real_visual_exit():
    r=RoundaboutRouter();t=replace(PatrolTuning(),roundabout_post_exit_guard_seconds=3.)
    r.last_heading=("imu",.1,5.)
    r._phase("EXIT",0.);r.ring_route_acquired=True;r.course.lap=0
    assert r._release_ring(1.,t,"visual_exit")
    assert r.phase=="SEEK_CROSSWALK" and r.post_exit_until==4.
    assert r.post_exit_guard_active(2.,t) and not r.post_exit_guard_active(4.1,t)
    assert not r.post_exit_guard_active(2.,replace(t,roundabout_post_exit_guard_enabled=0))
    r2=RoundaboutRouter();r2._phase("ENTRY",0.)
    assert not r2._release_ring(1.,t,"entry_probe_no_route")
    assert r2.post_exit_until==0 and not r2.post_exit_guard_active(1.1,t)


def test_post_exit_guard_limits_are_validated():
    for changes in ({"roundabout_post_exit_guard_enabled":2},
                    {"roundabout_post_exit_guard_seconds":0},
                    {"roundabout_post_exit_guard_seconds":11},
                    {"roundabout_post_exit_left_limit_rad_s":-0.1},
                    {"roundabout_post_exit_left_limit_rad_s":2.5}):
        with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()


def test_entry_probe_new_limits_are_validated():
    for changes in ({"ring_entry_probe_max_seconds":.29},{"ring_entry_probe_max_seconds":2.5},
                    {"ring_entry_probe_max_turn_deg":0},{"ring_entry_probe_max_turn_deg":200},
                    {"ring_entry_probe_extend_enabled":2}):
        with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()


def test_search_side_memory_points_to_last_clearly_seen_line():
    m=SearchSideMemory()
    assert m.update(PathEstimate(True,near_error=-.8,mid_error=-.7,confidence=.4),1.0) is None
    assert m.update(PathEstimate(False),1.05)==1      # line was left -> search left
    assert m.update(PathEstimate(False),1.5) is None  # stale hint expires
    m.update(PathEstimate(True,near_error=.6,mid_error=.5),2.0)
    assert m.update(PathEstimate(False),2.1)==-1      # line right -> search right
    # Crossed, weak or handoff observations cannot establish a side.
    m2=SearchSideMemory()
    m2.update(PathEstimate(True,near_error=.2,mid_error=-.5),1.0)
    assert m2.update(PathEstimate(False),1.1) is None
    m2.update(PathEstimate(True,near_error=.8,mid_error=.7,handoff=True),2.0)
    assert m2.update(PathEstimate(False),2.1) is None
    m2.update(PathEstimate(True,near_error=-.8,mid_error=-.7,rescue=True),3.0)
    assert m2.update(PathEstimate(False),3.1) is None
    # A shallow near point with a strong mid bend is still a real side hint.
    m3=SearchSideMemory()
    m3.update(PathEstimate(True,near_error=-.31,mid_error=-.77),1.0)
    assert m3.update(PathEstimate(False),1.1)==1




def test_latched_distance_sequence_replaces_retired_five_four_cooldown():
    r,t=armed()
    r._observe_vv_entry(DashObservation(valid=True,blocks=4),.1,t,.03,.01,(.3,0.))
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.001
    r._observe_vv_entry(None,.5,t,.03,.01,(.3,0.))
    assert r.phase=="APPROACH"
    assert r.entry_detection_reason=="vv_wait_diverging_left_branch"
    r._observe_vv_entry(DashObservation(valid=True,branch=PathEstimate(True)),.53,t,.03,.01,(.3,0.))
    assert r.phase=="ENTRY"
