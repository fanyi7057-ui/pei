"""V13 deployed VV entry, adaptive speed and measured gyro safety contracts."""
from dataclasses import replace
import hashlib
import math
from pathlib import Path
import numpy as np
import cv2
import pytest

from test_patrol_contour import load_actual_vision_module, v11_fork_components
from patrol_contour_runtime import PatrolTuning, FixedSpeedController, PathEstimate, RoundaboutRouter
from patrol_dash_entry import detect_vv_dashes, DashObservation


def masks(kind="diverging"):
    events=np.zeros((240,320),np.uint8)
    for x,y,w,h,_ in v11_fork_components(kind):
        cv2.rectangle(events,(int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),255,-1)
    white=np.where(events==0,255,0).astype(np.uint8)
    return events,white


def test_vv_detects_single_image_and_does_not_choose_exit_backwards():
    t=PatrolTuning()
    entry=detect_vv_dashes(*masks(),t)
    outgoing=detect_vv_dashes(*masks("converging"),t)
    assert entry.valid and entry.blocks>=4 and entry.branch.valid
    assert outgoing.valid and not outgoing.branch.valid
    assert t.vv_dash_confirm_frames==1


@pytest.mark.parametrize("case",["solid","zebra","noise","blue_sides","far","few"])
def test_vv_rejects_non_dash_cues(case):
    t=PatrolTuning()
    black,white=masks()
    if case in ("solid","zebra","noise"):
        black[:]=0
        if case=="solid": cv2.line(black,(175,230),(175,60),255,8)
        if case=="zebra":
            for y in (80,110,140,170,200): cv2.rectangle(black,(75,y),(245,y+9),255,-1)
        if case=="noise":
            for x,y in ((40,150),(270,190),(190,75)): cv2.rectangle(black,(x,y),(x+3,y+3),255,-1)
        white=np.where(black==0,255,0).astype(np.uint8)
    if case=="blue_sides": white[:]=0
    if case=="far": black[125:]=0
    if case=="few": black[:180]=0
    cue=detect_vv_dashes(black,white,t)
    assert not cue.valid or (case=="far" and not cue.branch.valid)


def test_vv_first_cue_one_frame_then_pass_exit_and_enter_four():
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    r=RoundaboutRouter()
    exit_cue=detect_vv_dashes(*masks("converging"),t)
    entry_cue=detect_vv_dashes(*masks(),t)
    r.observe(True,1.,0.,t,(.4,0.),dash_cue=exit_cue)
    assert r.phase=="APPROACH" and r.first_marker_at==0.
    for i in range(1,20):
        r.observe(True,1.,i*.1,t,(.4,0.),dash_cue=exit_cue)
    assert r.phase=="APPROACH"  # same 5-exit NEVER authorizes entry
    r.observe(True,1.,2.,t,(.4,0.),dash_cue=entry_cue)
    assert r.phase=="ENTRY"  # exactly one valid current entry frame


@pytest.mark.parametrize("case",["stopped","short_time","no_curve","duplicate"])
def test_vv_pass_guard_rejects_early_left_probe(case):
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    r=RoundaboutRouter()
    cue=detect_vv_dashes(*masks(),t)
    r.observe(True,1.,0.,t,(.4,0.),dash_cue=cue)
    if case=="no_curve": cue.branch=PathEstimate(False)
    for i in range(1,15):
        now=0. if case=="duplicate" else i*.02 if case=="short_time" else i*.1
        r.observe(True,1.,now,t,(0.,0.) if case=="stopped" else (.4,0.),dash_cue=cue)
    assert r.phase=="APPROACH"


def test_actual_vision_uses_vv_only_and_seeds_curved_entry(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    v=module.PatrolVision()
    t=replace(PatrolTuning(),roundabout_arm_seconds=0.,roundabout_arm_distance_m=0.)
    v._detect_roundabout_gate=lambda *_:pytest.fail("old entry detector executed")
    black,_=masks("converging")
    frame=cv2.cvtColor(255-black,cv2.COLOR_GRAY2BGR)
    v.motion_command=(.4,0.)
    for i in range(16): v.estimate(frame,t,i*.1)
    assert v.router.phase=="APPROACH"
    black,_=masks()
    frame=cv2.cvtColor(255-black,cv2.COLOR_GRAY2BGR)
    v.estimate(frame,t,1.6)
    assert v.router.phase=="ENTRY"
    assert v.ring_memory.path is not None and v.ring_memory.path.mid_error<0


def test_brief_startup_and_reacquire_do_not_block_normal_cruise():
    t,c=PatrolTuning(),FixedSpeedController()
    straight=PathEstimate(True,0.,.9,source="black")
    for i in range(15):
        v,_,_=c.command(straight,i/30,t)
        assert v<=t.startup_speed_m_s
    for i in range(15,100): v,_,_=c.command(straight,i/30,t)
    assert v==pytest.approx(.65)
    v,_,_=c.command(PathEstimate(False),100/30,t)
    assert 0<v<=t.lost_speed_m_s
    v,_,_=c.command(straight,101/30,t)
    assert v<=t.startup_speed_m_s


def test_s_preview_brakes_before_near_error_and_holds_across_reversal():
    t,c=PatrolTuning(),FixedSpeedController()
    p=PathEstimate(True,0.,.9,near_error=0.,mid_error=.5,far_error=.7)
    c.planned_speed(p,0.,t)
    assert c.planned_speed(p,.6,t)<=.45
    p.mid_error=-.5
    assert c.planned_speed(p,.63,t)<=.45
    p.mid_error=p.far_error=0.
    assert c.planned_speed(p,.9,t)<=.45
    c.planned_speed(p,1.,t)
    assert c.planned_speed(p,1.15,t)<=.65


def test_ring_limit_and_second_lap_same_controller_plan():
    t=PatrolTuning()
    c=FixedSpeedController()
    path=PathEstimate(True,0.,.9)
    for i in range(100): v,_,_=c.command(path,i/30,t,ring_tracking=True)
    assert v<=.3
    assert t.fixed_speed_m_s==.65 and t.roundabout_entry_turn_rad_s==1.2
    assert t.roundabout_search_turn_rad_s==.8


def gyro_run(r,t,rate=1.,start=0.,duration=6.5):
    r.last_motion_linear=.3
    for now in np.arange(start,start+duration,.05): r.observe_gyro(rate,float(now),t)


@pytest.mark.parametrize("phase",["SEEK","APPROACH","ENTRY","EXIT","CW_STOP"])
def test_gyro_outside_follow_never_fills_a_lap(phase):
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase(phase,0.)
    gyro_run(r,t)
    assert r.gyro_yaw==0.


@pytest.mark.parametrize("case",["stationary","no_odom","short_distance","stale","clockwise","gap","invalid"])
def test_gyro_cannot_fake_exit(case):
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    if case=="stationary":
        r.last_motion_linear=0.
        for now in np.arange(0.,7.,.05): r.observe_gyro(1.,float(now),t)
    elif case=="gap":
        r.last_motion_linear=.3
        for now in (0.,1.,2.,3.,4.,5.,6.): r.observe_gyro(2.,now,t)
    else:
        gyro_run(r,t,rate=-1. if case=="clockwise" else math.nan if case=="invalid" else 1.)
    r.phase_distance=.1 if case=="short_distance" else 1.
    r.phase_measured_distance=r.phase_distance
    now=10. if case=="stale" else 6.5
    pose=None if case=="no_odom" else (0.,0.,0.)
    r.observe(False,0.,now,t,(.3,.1),pose=pose)
    assert r.phase=="FOLLOW" and not r.gyro_exit_requested and r.completed==0


def test_measured_full_rotation_requests_exit_but_not_completion():
    r,t=RoundaboutRouter(),PatrolTuning()
    r._phase("FOLLOW",0.)
    gyro_run(r,t)
    r.phase_distance=1.
    r.phase_measured_distance=1.
    r.observe(False,0.,6.5,t,(.3,.1),pose=(0.,0.,0.))
    assert r.phase=="EXIT" and r.gyro_exit_requested and r.completed==0
    assert r.motion_override(6.6,t)==(.2,0.,"gyro_exit_seek")
    assert not r.crosswalk_armed(6.6,t)


@pytest.mark.parametrize("change",[{"vv_dash_confirm_frames":0},{"vv_dash_min_blocks":1},
    {"vv_dash_roi_top_ratio":1},{"ring_gyro_exit_degrees":0},{"ring_gyro_timeout_seconds":0},
    {"ring_gyro_exit_enabled":2},{"lost_speed_m_s":.8},{"linear_decel_m_s2":0}])
def test_new_values_reject_unsafe_or_incoherent_snapshot(change):
    with pytest.raises(ValueError): replace(PatrolTuning(),**change).validate()


def test_patrol_transfer_manifest_matches_all_deployed_additions():
    root=Path(__file__).resolve().parents[4]
    entries={}
    for line in (root/"checksums/PATROL_TEST_ADDITIONS_SHA256SUMS.txt").read_text(encoding="utf-8").splitlines():
        digest,relative=line.split(maxsplit=1)
        entries[relative.strip()]=digest
        assert hashlib.sha256((root/relative.strip()).read_bytes()).hexdigest()==digest,relative
    assert "ros2_ws/src/wheel_robot/scripts/patrol_dash_entry.py" in entries
    assert "ros2_ws/src/wheel_robot/test/test_patrol_vv_entry.py" in entries
