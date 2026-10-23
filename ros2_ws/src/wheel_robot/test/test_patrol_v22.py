"""V22 user-requested 0.83m target with paired deadlines, unchanged speed."""
from dataclasses import replace
from test_patrol_contour import make_node,node_module
from test_patrol_v20 import seek
from patrol_contour_runtime import PatrolTuning,PathEstimate


def test_exit_083_can_pass_old_043_and_063_without_early_stop():
    r,t,now=seek()
    t=replace(t,ring_gyro_exit_seek_distance_m=.83,ring_gyro_exit_seek_seconds=8.,roundabout_exit_acquire_max_seconds=9.)  # historical V22 configuration
    assert (t.ring_gyro_exit_seek_distance_m,t.ring_gyro_exit_seek_seconds,t.roundabout_exit_acquire_max_seconds)==(.83,8.,9.)
    assert .83/t.ring_recovery_speed_m_s<8<9<t.roundabout_route_max_seconds
    for elapsed,distance in ((4.,.43),(6.,.63),(7.,.82)):
        r.measured_turn.stamp=now+elapsed
        assert r.motion_override(now+elapsed,t,(distance,0.,0.),PathEstimate(False))==(.12,0.,"gyro_exit_seek")
    r.measured_turn.stamp=now+7.1
    assert r.motion_override(now+7.1,t,(.831,0.,0.),PathEstimate(False))==(0.,0.,"ring_recovery_stop")
    assert not any(r.course.rings)


def test_063_no_longer_triggers_startup_validation_error():
    replace(PatrolTuning(),ring_gyro_exit_seek_distance_m=.63).validate()
    PatrolTuning().validate()


def test_fresh_imu_stall_stops_after_eight_seconds():
    r,t,now=seek();r.measured_turn.stamp=now+8.01
    assert r.motion_override(now+8.01,t,(0.,0.,0.),PathEstimate(False))==(0.,0.,"ring_recovery_stop")
    assert r.fault=="gyro_exit_seek_no_reliable_path"
