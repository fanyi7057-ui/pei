"""V23 baseline with short-gap hold, bounded sustained-loss search and FPV."""
from dataclasses import replace,fields
import ast
import hashlib
import time
import http.client
import numpy as np
import pytest
from test_patrol_contour import cruise_controller,SCRIPTS
from patrol_contour_runtime import PatrolTuning,PathEstimate,FixedSpeedController
from patrol_preview import FirstPersonPreview

def current_cruise(error=-.1):
    return cruise_controller(error,maintain_cruise_during_search=1,search_slow_after_seconds=.2)

def test_short_single_frame_loss_does_not_brake_or_accelerate():
    c,t=current_cruise();old=c.last_command
    assert c.command(PathEstimate(False),2.03,t)==(*old,'brief_loss_hold')
    for now in (2.06,2.09,2.12):assert c.command(PathEstimate(True,-.1,.9,source='black'),now,t)[0]==pytest.approx(old[0])

def test_repeated_short_dropouts_do_not_pulse_speed():
    c,t=current_cruise();t=replace(t,reacquire_confirm_frames=3)
    for i in range(100):
        valid=i%5!=0
        v,_,_=c.command(PathEstimate(valid,-.1,.9,source='black'),2.03+i*.03,t)
        assert v==pytest.approx(.65)

def test_sustained_loss_gently_caps_speed_and_turn_without_leakage():
    c,t=current_cruise(-.9)
    assert c.last_command[1]>1.5
    assert c.command(PathEstimate(False),2.03,t)[1]>1.5
    for now in (2.21,2.24,2.3,2.5):
        v,w,_=c.command(PathEstimate(False),now,t)
        assert v<=.2 and abs(w)<=.6

def test_hard_budget_still_stops_without_faking_reacquisition():
    c,t=current_cruise();result=c.command(PathEstimate(False),11.,t)
    assert result==(0.,0.,'localization_fault_latched')

def test_one_flash_does_not_change_search_direction_memory():
    c,t=current_cruise(-.8);c.command(PathEstimate(False),2.21,t);goal=c.search_turn_goal
    c.command(PathEstimate(True,.8,.9,near_error=.8,source='black'),2.24,t)
    assert c.search_turn_goal==goal
    assert c.command(PathEstimate(False),2.27,t)[1]>=0

def test_entry_distance_cannot_change_ordinary_controller_outputs():
    commands=[]
    for distance in (.12,.3,.73):
        t=replace(PatrolTuning(),roundabout_first_marker_pass_distance_m=distance);c=FixedSpeedController();out=[]
        for i in range(90):out.append(c.command(PathEstimate(True,.1,.9,source='black'),i/30,t))
        commands.append(out)
    assert commands[0]==commands[1]==commands[2]

def test_no_experimental_masks_or_posture_stage_defaults():
    t=PatrolTuning();t.validate();names={f.name for f in fields(t)}
    assert (t.posture_height_m,t.posture_pitch_deg)==(.15,25.)
    assert (t.roundabout_first_marker_pass_distance_m,t.ring_gyro_exit_seek_distance_m)==(.12,.73)
    assert not names&{'mask_stability_enabled','black_wide_guard_enabled','posture_stage_enabled','loss_grace_seconds'}
    assert not (SCRIPTS/'patrol_stability.py').exists() and not (SCRIPTS/'patrol_turn_posture.py').exists()

def test_original_path_selector_source_hash_is_restored():
    s=(SCRIPTS/'patrol_contour_runtime.py').read_text(encoding='utf8')
    node=next(n for n in ast.parse(s).body if isinstance(n,ast.ClassDef) and n.name=='PathSelector')
    assert hashlib.sha256(ast.get_source_segment(s,node).strip().encode()).hexdigest()=='8e5d914aff1fe47fd4039d05214dae936fbf56fd79d36ade94128af131015224'

def test_kept_preview_is_loopback_read_only_and_stops():
    stream=FirstPersonPreview(lambda:(np.zeros((48,64,3),np.uint8),time.monotonic()),port=0)
    try:
        assert stream.server.server_address[0]=='127.0.0.1'
        c=http.client.HTTPConnection('127.0.0.1',stream.port,timeout=2);c.request('GET','/stream')
        r=c.getresponse();assert r.status==200 and b'--frame' in r.read(40);c.close()
    finally:stream.close()
    assert not stream.encoder.is_alive()

@pytest.mark.parametrize('changes',[{'search_slow_after_seconds':-.1},{'search_slow_after_seconds':float('nan')},
    {'search_slow_after_seconds':.6},{'search_max_turn_rad_s':0},{'search_max_turn_rad_s':float('inf')}])
def test_new_search_parameters_are_validated(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()
