"""V21 bounded search prediction, never synthesized route evidence."""
from dataclasses import replace
import numpy as np
import pytest
from test_patrol_contour import load_actual_vision_module
from test_patrol_v20 import seek
from patrol_contour_runtime import PatrolTuning,PathEstimate
from patrol_dash_entry import RingSearchPredictor,track_vv_curve


def path(shift=0):
    return PathEstimate(True,confidence=.8,route_locked=True,route_observed=True,
        points=np.array([[100.+shift,100.],[120.+shift,150.],[140.+shift,210.]]))


def test_two_actual_routes_predict_bounded_reference_without_mutating_them():
    p=RingSearchPredictor();t=PatrolTuning();a,b=path(),path(10)
    p.remember(a,0.);p.remember(b,.1)
    reference=p.reference(.2,320,t)
    assert np.allclose(reference[:,0],b.points[:,0]+10)
    assert np.allclose(b.points[:,0],[110,130,150])
    assert p.reference(.46,320,t) is None
    assert p.latest[0]==.1  # asking for prediction never renews an observation


@pytest.mark.parametrize("kind",["missing","rescue","handoff","unlocked","unobserved"])
def test_non_observations_cannot_renew_predictor(kind):
    p=RingSearchPredictor();p.remember(path(),0.)
    b=path(10)
    if kind=="missing":b.valid=False
    if kind=="rescue":b.rescue=True
    if kind=="handoff":b.handoff=True
    if kind=="unlocked":b.route_locked=False
    if kind=="unobserved":b.route_observed=False
    p.remember(b,.1)
    assert p.reference(.2,320,PatrolTuning()) is None


def test_prediction_clamps_motion_and_requires_current_pixels():
    p=RingSearchPredictor();t=PatrolTuning()
    p.remember(path(),0.);p.remember(path(100),.03)
    ref=p.reference(.2,320,t)
    assert np.max(ref[:,0]-p.latest[1][:,0])<=32.001
    dark=np.zeros((240,320),np.uint8)
    assert not track_vv_curve(dark,255-dark,t,ref,reacquire=True).valid
    p.reset();assert p.reference(.2,320,t) is None


def test_exit_new_distance_and_time_limits_are_consistent():
    r,t,now=seek()
    assert t.ring_gyro_exit_seek_distance_m==.83
    assert .83/t.ring_recovery_speed_m_s<t.ring_gyro_exit_seek_seconds<t.roundabout_exit_acquire_max_seconds
    r.measured_turn.stamp=now+3.6
    assert r.motion_override(now+3.6,t,(.42,0.,0.),PathEstimate(False))[2]=="gyro_exit_seek"
    r.measured_turn.stamp=now+3.7
    assert r.motion_override(now+3.7,t,(.831,0.,0.),PathEstimate(False))[2]=="ring_recovery_stop"
    assert not any(r.course.rings)


@pytest.mark.parametrize("changes",[{"ring_prediction_seconds":0},{"ring_prediction_seconds":1},
    {"ring_prediction_max_shift_ratio":.3},{"ring_gyro_exit_seek_seconds":9.1}])
def test_invalid_prediction_and_exit_limits_rejected(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()
