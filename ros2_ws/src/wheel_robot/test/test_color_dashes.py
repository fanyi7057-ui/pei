from dataclasses import replace
import cv2
import numpy as np
from test_patrol_vv_v16 import masks
from patrol_dash_entry import detect_color_dashes
from patrol_contour_runtime import PatrolTuning


def test_color_keeps_entry_and_exit_geometry():
    for kind in ('diverging','converging'):
        dark,_=masks(kind)
        frame=np.full((*dark.shape,3),235,np.uint8)
        frame[dark>0]=30
        result=detect_color_dashes(frame,PatrolTuning())
        assert result.valid
        assert result.branch.valid == (kind=='diverging')
        assert result.exit_branch.valid == (kind=='converging')


def test_reflective_blocks_with_dark_core_remain_visible():
    dark,_=masks()
    frame=np.full((*dark.shape,3),235,np.uint8)
    frame[dark>0]=130
    for y in (210,185,160,135,110,85):
        cv2.rectangle(frame,(173,y-2),(177,y+2),(45,45,45),-1)
    result=detect_color_dashes(frame,PatrolTuning())
    assert result.valid and result.blocks>=4


def test_blue_blocks_solid_line_and_isolated_object_do_not_authorize():
    dark,_=masks()
    frame=np.full((*dark.shape,3),235,np.uint8)
    frame[dark>0]=(150,30,20)
    assert not detect_color_dashes(frame,PatrolTuning()).valid


def test_color_detector_is_read_only_and_rejects_flat_reflection():
    dark,_=masks()
    frame=np.full((*dark.shape,3),235,np.uint8)
    frame[dark>0]=150
    before=frame.copy()
    assert not detect_color_dashes(frame,PatrolTuning()).valid
    assert np.array_equal(frame,before)
    frame[:]=235
    cv2.line(frame,(175,20),(175,235),(20,20,20),9)
    assert not detect_color_dashes(frame,PatrolTuning()).valid
    frame[:]=235
    cv2.rectangle(frame,(160,170),(180,200),(20,20,20),-1)
    assert not detect_color_dashes(frame,PatrolTuning()).valid
