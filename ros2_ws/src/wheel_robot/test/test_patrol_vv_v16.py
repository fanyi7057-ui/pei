"""V16 deployed VV entry integration; no ROS/motor actuation."""
from dataclasses import replace
import ast
import hashlib
from pathlib import Path

import cv2
import numpy as np
import pytest

from test_patrol_contour import load_actual_vision_module
from patrol_contour_runtime import PatrolTuning, RoundaboutRouter, PathEstimate, FixedSpeedController
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
                        ("FixedSpeedController","6558b6fb24caf548c4b49abbd9034341f6c3cc5c0618ff3621c36b1b6d665638")):
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




def test_latched_distance_sequence_replaces_retired_five_four_cooldown():
    r,t=armed()
    r._observe_vv_entry(DashObservation(valid=True,blocks=4),.1,t,.03,.01,(.3,0.))
    assert r.phase=="APPROACH"
    r.distance=r.first_marker_distance+t.ring_entry_advance_m+.001
    r._observe_vv_entry(None,.5,t,.03,.01,(.3,0.))
    assert r.phase=="ENTRY"
