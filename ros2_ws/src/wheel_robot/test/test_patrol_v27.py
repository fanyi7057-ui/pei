"""Two independent measured bends change posture, never ring receipts."""
import io
import math
from dataclasses import replace
from types import SimpleNamespace
import pytest
from test_patrol_contour import make_node,node_module
from patrol_contour_runtime import PatrolTuning,PathEstimate
from patrol_turn_posture import TurnPostureGate
from patrol_pc_viewer import read_jpeg


def path(error=0.):return PathEstimate(True,error,.9,near_error=error,source='black')

def drive(g,yaws,t=None,now=0.,position=0.,center=True,source='imu'):
    t=t or PatrolTuning()
    for degrees in yaws:
        now+=.05;position+=.02
        g.update((source,math.radians(degrees),now),(position,0.,0.),now,.4,
                 0. if center else .7,path(0. if center else .3),t)
    return now,position

def bend(g,start=0.,now=0.,position=0.,sign=1):
    now,position=drive(g,[start]*6,now=now,position=position)
    now,position=drive(g,[start+sign*i*3 for i in range(1,31)],now=now,position=position,center=False)
    return drive(g,[start+sign*90]*7,now=now,position=position)

def test_two_independent_bends_not_merely_total_angle():
    g=TurnPostureGate();now,p=bend(g)
    assert g.count==1 and not g.complete
    bend(g,start=90,now=now,position=p)
    assert g.count==2 and g.complete
    bend(g,start=180,now=now+3,position=p+1)
    assert g.count==2

def test_right_turn_and_yaw_wrap_are_supported():
    g=TurnPostureGate();bend(g,start=170)
    assert g.count==1
    g=TurnPostureGate();bend(g,sign=-1)
    assert g.count==1

def test_wobble_does_not_sum_absolute_angles():
    g=TurnPostureGate();drive(g,[0]*5+[v for _ in range(30) for v in (3,6,9,6,3,0,-3,-6,-9,-6,-3,0)],center=False)
    assert g.count==0

def test_heading_reached_but_not_centered_does_not_count():
    g=TurnPostureGate();drive(g,[0]*4+list(range(3,91,3))+[90]*10,center=False)
    assert g.count==0

def test_wide_spin_cannot_be_counted_as_two_bends():
    g=TurnPostureGate();drive(g,[0]*5+list(range(3,181,3))+[180]*8,center=False)
    assert g.count==0

def test_stationary_odom_and_duplicate_timestamps_do_not_mint_turns():
    g=TurnPostureGate();t=PatrolTuning()
    for i in range(50):
        now=i*.05;yaw=min(90,i*3)
        g.update(('imu',math.radians(yaw),now),(0.,0.,0.),now,.4,0.,path(),t)
    assert g.count==0
    before=g.center_hits
    for _ in range(20):g.update(('imu',math.pi/2,2.45),(0.,0.,0.),2.45,.4,0.,path(),t)
    assert g.center_hits==before and g.count==0

def test_pause_preserves_completed_bend_and_reset_clears_it():
    g=TurnPostureGate();bend(g);g.pause()
    assert g.count==1 and g.last_stamp is None
    g.reset();assert g.count==0 and not g.complete

@pytest.mark.parametrize('case',['manual','stale','source_switch','jump'])
def test_invalid_evidence_cannot_complete_partial_turn(case):
    g=TurnPostureGate();t=PatrolTuning()
    drive(g,[0]*5+list(range(3,76,3)),center=False)
    if case=='manual':g.update(('imu',math.pi/2,2.),(1.,0.,0.),2.,.4,0.,path(),t,False)
    if case=='stale':g.update(('imu',math.pi/2,1.),(1.,0.,0.),2.,.4,0.,path(),t)
    if case=='source_switch':g.update(('odom',math.pi/2,2.),(1.,0.,0.),2.,.4,0.,path(),t)
    if case=='jump':g.update(('imu',math.pi,1.76),(1.,0.,0.),1.76,.4,0.,path(),t)
    assert not g.complete and g.count==0

def test_effective_initial_and_after_two_postures(node_module,monkeypatch):
    from race_posture import RacePosture
    monkeypatch.setattr(node_module.base,'RacePosture',RacePosture,raising=False)
    node,_=make_node(node_module,monkeypatch);node.turn_posture=TurnPostureGate();targets=[]
    node._posture_transition=SimpleNamespace(set_target=lambda p,stamp:targets.append(p))
    node._set_patrol_posture();assert (targets[-1].height_m,targets[-1].pitch_deg)==(.15,25.)
    node.turn_posture.count=2;node._set_patrol_posture()
    assert (targets[-1].height_m,targets[-1].pitch_deg)==(.25,28.)

def test_node_switch_does_not_change_speed_or_task_receipts(node_module,monkeypatch):
    from race_posture import RacePosture
    monkeypatch.setattr(node_module.base,'RacePosture',RacePosture,raising=False)
    node,_=make_node(node_module,monkeypatch);node.turn_posture=TurnPostureGate()
    node.vision.router=SimpleNamespace(phase='SEEK',course=SimpleNamespace(lap=0,rings=[False,False],zebras=[False,False]))
    targets=[];node._posture_transition=SimpleNamespace(set_target=lambda p,stamp:targets.append(p))
    node._set_patrol_posture();now=0.;position=0.
    for start in (0,90):
        for degrees,straight in [(start,True)]*6+[(start+i*3,False) for i in range(1,31)]+[(start+90,True)]*7:
            now+=.05;position+=.02
            node.vision.motion_pose=(position,0.,0.)
            node._last_command=(.4,0. if straight else .7)
            node._heading_snapshot=lambda *args,d=degrees,n=now:('imu',math.radians(d),n)
            node._update_turn_posture(path(0. if straight else .3),now,node.patrol_tuning)
    assert node.turn_posture.complete and targets[-1].height_m==.25 and targets[-1].pitch_deg==28.
    assert node._posture_geometry_reset_pending and node._last_command==(.4,0.)
    assert node.vision.router.phase=='SEEK' and node.vision.router.course.rings==[False,False]
    assert node.vision.router.course.zebras==[False,False]

def test_disabled_staging_uses_goal_posture_immediately(node_module,monkeypatch):
    from race_posture import RacePosture
    monkeypatch.setattr(node_module.base,'RacePosture',RacePosture,raising=False)
    node,_=make_node(node_module,monkeypatch);node.turn_posture=TurnPostureGate()
    node.patrol_tuning=replace(node.patrol_tuning,posture_stage_enabled=0)
    targets=[];node._posture_transition=SimpleNamespace(set_target=lambda p,stamp:targets.append(p))
    node._set_patrol_posture();assert targets[-1].height_m==.25 and targets[-1].pitch_deg==28.

@pytest.mark.parametrize('changes',[{'posture_turn_target_deg':50},{'posture_turn_center_frames':1},
    {'posture_start_height_m':.1},{'posture_start_pitch_deg':90},{'posture_stage_enabled':2}])
def test_bad_parameters_rejected(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()

def test_mjpeg_reader_rejects_oversized_or_truncated_frames():
    assert read_jpeg(io.BytesIO(b'--frame\r\nContent-Length: 3\r\n\r\nabc'))==b'abc'
    with pytest.raises(ValueError):read_jpeg(io.BytesIO(b'--frame\r\nContent-Length: 9999999\r\n\r\n'))
    with pytest.raises(OSError):read_jpeg(io.BytesIO(b'--frame\r\nContent-Length: 3\r\n\r\na'))
