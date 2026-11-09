"""Offline tests: no ROS, camera or actuator starts."""
import ast
from pathlib import Path
import types
import cv2
import numpy as np
from test_patrol_contour import load_actual_vision_module
from patrol_contour_runtime import PatrolTuning, road_masks
from patrol_dash_entry import detect_vv_dashes


def test_capture_uses_same_masks_and_preserves_original(monkeypatch):
    module=load_actual_vision_module(monkeypatch)
    source=(Path(__file__).resolve().parents[1]/'scripts/camera_capture_diag.py').read_text(encoding='utf-8')
    function=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='build_capture')
    ns=dict(cv2=cv2,np=np,base=module.base,road_masks=road_masks,detect_vv_dashes=detect_vv_dashes)
    exec(compile(ast.Module(body=[function],type_ignores=[]),'capture','exec'),ns)
    frame=np.full((480,640,3),245,np.uint8)
    for y in range(80,420,60):cv2.rectangle(frame,(300,y),(320,y+25),(0,0,0),-1)
    original=frame.copy()
    images,metadata,table=ns['build_capture'](frame,PatrolTuning())
    assert np.array_equal(frame,original)
    assert np.array_equal(images['raw'],original)
    assert 'events_actual' in images and 'events_before_guard' in images
    assert 'valid=' in metadata and 'reason' in table
    assert images['comparison'].shape==(480,960,3)
    assert not np.any(images['events_actual'][images['blue_guard']>0])


def test_capture_does_not_construct_race_controller():
    path=Path(__file__).resolve().parents[1]/'scripts/camera_capture_diag.py'
    source=path.read_text(encoding='utf-8')
    assert 'node=WheellegStandOnly()' in source
    assert 'PatrolTestNode(' not in source and 'WheelLegRaceNode(' not in source
    launch=(path.parents[1]/'launch/camera_capture.launch.py').read_text(encoding='utf-8')
    assert 'stand_height_m:=0.15' in launch and 'stand_pitch_deg:=25.0' in launch
    assert 'CAPTURE_TOPIC_MODE' in launch
