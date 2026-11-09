from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from test_patrol_contour import load_actual_vision_module
from patrol_contour_runtime import PathEstimate, PatrolTuning


@pytest.mark.parametrize('mask_height', [175, 240, None])
def test_snapshot_composes_roi_without_stretching(monkeypatch,tmp_path,mask_height):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision()
    v.snapshot_directory=tmp_path
    frame=np.full((240,320,3),80,np.uint8);dark=np.zeros((240,320),np.uint8)
    mask=None if mask_height is None else np.full((mask_height,320),255,np.uint8)
    images=[]
    monkeypatch.setattr(m.cv2,'imwrite',lambda _p,image:images.append(image.copy()) or True)
    v._snapshot_if_missing(frame,dark,PathEstimate(False,mask=mask),PathEstimate(False),10.,road_top=60)
    v._snapshot_worker.join(timeout=3)
    assert not v._snapshot_worker.is_alive()
    assert images[0].shape==(240,960,3)
    if mask_height==175:
        assert np.all(images[0][50,640:]==0)
        assert np.all(images[0][100,640:]==255)
        assert np.all(images[0][239,640:]==0)


def test_snapshot_preparation_failure_cannot_interrupt_estimate(monkeypatch):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision()
    def fail(*args,**kwargs):raise ValueError('diagnostic failure')
    monkeypatch.setattr(v,'_prepare_snapshot',fail)
    frame=np.full((240,320,3),245,np.uint8)
    cv2.line(frame,(160,0),(160,239),(0,0,0),7)
    result=v.estimate(frame,PatrolTuning(),10.)
    assert result.valid and v.source=='black'


def test_snapshot_disk_failure_is_contained_and_rate_limited(monkeypatch,tmp_path,capsys):
    m=load_actual_vision_module(monkeypatch);v=m.PatrolVision();v.snapshot_directory=tmp_path
    monkeypatch.setattr(m.cv2,'imwrite',lambda *_:False)
    frame=np.zeros((240,320,3),np.uint8);mask=np.zeros((240,320),np.uint8)
    for now in (10.,10.1):
        v._snapshot_if_missing(frame,mask,PathEstimate(False),PathEstimate(False),now,force=True)
        v._snapshot_worker.join(timeout=3)
    assert capsys.readouterr().out.count('snapshot_failed=')==1
