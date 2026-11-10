from dataclasses import replace
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import json
import time
import cv2
import numpy as np
from patrol_contour_runtime import PatrolTuning, contour_order_reference
from patrol_recorder import PatrolRecorder


def test_ordered_arcs_pair_straight_road_without_border_fabrication():
    mask=np.zeros((160,320),np.uint8);mask[:,80:241]=255
    result=contour_order_reference(mask,np.zeros_like(mask),240,80,PatrolTuning())
    assert result is not None and result.valid and result.reason=="road_contour_order"
    assert abs(result.error)<.01 and np.allclose(result.points[:,0],160)
    assert np.all(result.widths[:,0]==160)


def test_ordered_arcs_reject_blue_hole_and_missing_side():
    t=PatrolTuning();mask=np.zeros((160,320),np.uint8);mask[:,80:241]=255
    mask[40:80,140:180]=0
    assert contour_order_reference(mask,np.zeros_like(mask),240,80,t) is None
    mask[:]=0;mask[:,:240]=255
    assert contour_order_reference(mask,np.zeros_like(mask),240,80,t) is None


def test_recorder_roundtrip_has_original_frames_and_drops_are_accounted(tmp_path):
    package=Path(__file__).resolve().parents[1]
    recorder=PatrolRecorder(tmp_path/'run',package)
    frame=np.full((120,160,3),150,np.uint8)
    for i in range(8):
        recorder.submit(frame,frame,dict(_tuning=PatrolTuning(),capture_monotonic=float(i),control_monotonic=float(i),sequence=i))
    recorder.close()
    summary=json.loads((tmp_path/'run/summary.json').read_text())
    assert summary['error'] is None and summary['saved']>0
    assert summary['submitted']==summary['saved']+summary['dropped']
    video=cv2.VideoCapture(str(tmp_path/'run/raw.avi'));ok,image=video.read();video.release()
    assert ok and image.shape==frame.shape
    assert (tmp_path/'run/frames.csv').exists()
    assert (tmp_path/'run/initial_config.yaml').exists()
    from offline_contour_replay import replay
    assert replay(tmp_path/'run',tmp_path/'replay')==summary['saved']
    assert (tmp_path/'replay/comparison.csv').exists()


def test_new_algorithm_is_used_on_simple_white_road():
    from patrol_contour_runtime import road_centerline
    f=np.full((240,320,3),(255,0,0),np.uint8)
    f[:,80:241]=245
    hsv=cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
    p=road_centerline(hsv,replace(PatrolTuning(),contour_order_enabled=1))
    assert p.valid and p.reason=="road_contour_order" and abs(p.error)<.01


def test_contour_winding_does_not_swap_left_and_right(monkeypatch):
    import patrol_contour_runtime as runtime
    mask=np.zeros((160,320),np.uint8);mask[:,70:230]=255
    expected=contour_order_reference(mask,np.zeros_like(mask),240,80,PatrolTuning())
    original=runtime.cv2.findContours
    def reverse(*args,**kwargs):
        contours,hierarchy=original(*args,**kwargs)
        return [c[::-1].copy() for c in contours],hierarchy
    monkeypatch.setattr(runtime.cv2,'findContours',reverse)
    actual=contour_order_reference(mask,np.zeros_like(mask),240,80,PatrolTuning())
    assert expected is not None and actual is not None
    assert np.allclose(actual.points,expected.points)
