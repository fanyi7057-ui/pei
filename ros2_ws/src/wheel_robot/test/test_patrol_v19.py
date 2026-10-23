"""V19 log1043b9e6: current guide timing, safe wait and confirmed unwind."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from test_patrol_contour import make_node, node_module
from patrol_contour_runtime import PatrolTuning,RoundaboutRouter,PathEstimate,FixedSpeedController,CornerExitGuard


def owned(observed=False):
    return PathEstimate(True,-.1,.8,near_error=-.1,mid_error=-.1,route_locked=True,route_observed=observed)


@pytest.mark.parametrize("phase",["SEEK","ENTRY","FOLLOW","EXIT"])
def test_starting_line_budget_cannot_abort_active_ring(phase):
    r,t=RoundaboutRouter(),PatrolTuning();r.started_at=0.;r._phase(phase,29.)
    assert not r.apply_time_fallback(30.,t)
    assert r.phase==phase and r.completed==0 and not r.fault


def test_current_owned_pixels_extend_only_to_hard_six_second_limit():
    r,t=RoundaboutRouter(),PatrolTuning();r.started_at=0.;r._phase("ENTRY",29.)
    r.accept_path(owned(),31.99,t)
    r.observe(False,0.,32.01,t,(.12,.2))
    assert r.phase=="ENTRY" and not r.fault
    r.accept_path(owned(),34.99,t)
    r.observe(False,0.,35.01,t,(.12,.2))
    assert r.phase=="SEEK" and r.ring_release_source=="entry_no_route_abort"
    assert not any(r.course.rings)


def test_missing_pixels_cannot_renew_extended_entry_budget():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r.accept_path(owned(),2.9,t)
    r.observe(False,0.,3.05,t,(.12,.2))
    assert r.phase=="ENTRY"
    r.observe(False,0.,3.3,t,(.12,.2))
    assert r.phase=="SEEK" and not any(r.course.rings)


def test_low_speed_can_reach_real_distance_before_acquisition_hard_limit():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    for i in range(50):
        now=i*.1
        r.observe(False,0.,now,t,(.12,.2),(now*.12,0.,now*.2),heading=("imu",now*.2,now))
        r.accept_path(owned(observed=True),now,t)
        if r.phase=="FOLLOW":break
    assert r.phase=="FOLLOW" and r.ring_route_acquired
    assert now>3 and r.measured_turn.travel>=.45 and not any(r.course.rings)


def test_pause_erases_current_guide_extension_credit():
    r,t=RoundaboutRouter(),PatrolTuning();r._phase("ENTRY",0.)
    r.accept_path(owned(),1.,t);r.pause(1.1)
    assert r.entry_guide_at is None


def test_localization_wait_keeps_gate_and_course_and_never_auto_moves(node_module,monkeypatch):
    n,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter();n.vision.router=r;n.vision.reset_path=lambda:None
    gate=object();n._stand_gate=gate
    n._wait_for_operator("ring_recovery_no_reliable_path",localization=True)
    assert n._stand_gate is gate and n._localization_wait and r.paused_at==10.
    n._recovery_preview=SimpleNamespace(estimate=lambda *a,**k:owned(True))
    n._debug_window_ready=False
    for now in (10.1,10.2,10.3):
        n._run_localization_preview(n._latest_frame,n.patrol_tuning,now,False)
    assert n._recovery_ready_frames==3 and n._recovery_ready_announced
    assert all(v==0 and w==0 for v,w in commands)
    assert n._localization_wait and r.paused_at==10. and not any(r.course.rings)
    n._on_race_start_confirm(SimpleNamespace(data=True))
    assert not n._localization_wait and r.paused_at is None and n._stand_gate is gate


def test_false_confirmation_does_not_resume_localization_wait(node_module,monkeypatch):
    n,_=make_node(node_module,monkeypatch);n.vision.reset_path=lambda:None
    n._wait_for_operator("localization",localization=True)
    n._on_race_start_confirm(SimpleNamespace(data=False))
    assert n._localization_wait


def test_actual_wait_cycle_cannot_resume_or_advance_original_task(node_module,monkeypatch):
    n,commands=make_node(node_module,monkeypatch)
    r=RoundaboutRouter();n.vision.router=r;n.vision.reset_path=lambda:None
    n._wait_for_operator("localization",localization=True)
    n._recovery_preview=SimpleNamespace(estimate=lambda *a,**k:owned(True))
    n.vision.estimate=lambda *a,**k:pytest.fail("paused task perception must not execute")
    monkeypatch.setattr(node_module.base,"SHOW_DEBUG",False)
    n._run_control_cycle()
    assert n._localization_wait and r.paused_at==10. and r.phase=="SEEK"
    assert all(cmd==(0.,0.) for cmd in commands)


def test_exit_guard_requires_three_current_centered_route_frames():
    t=PatrolTuning();c=FixedSpeedController();g=CornerExitGuard()
    c.filtered_error=.8;c.last_command=(.65,-1.4)
    p=PathEstimate(True,.02,.9,near_error=.02,mid_error=.03,route_observed=True)
    for now in (0.,.03):g.observe(p,c,now,t)
    assert c.filtered_error==.8
    g.observe(p,c,.06,t)
    assert c.filtered_error is None and c.last_command==(.65,-1.4)


@pytest.mark.parametrize("kind",["curved","rescue","handoff","missing","ring"])
def test_exit_guard_cannot_reduce_real_corner_or_ring(kind):
    t=PatrolTuning();c=FixedSpeedController();g=CornerExitGuard();c.filtered_error=.8
    p=PathEstimate(True,.02,.9,near_error=.02,mid_error=.03,route_observed=True)
    if kind=="curved":p.mid_error=.4
    if kind=="rescue":p.rescue=True
    if kind=="handoff":p.handoff=True
    if kind=="missing":p.valid=False
    for i in range(5):g.observe(p,c,i*.03,t,allowed=kind!="ring")
    assert c.filtered_error==.8


@pytest.mark.parametrize("changes",[{"ring_entry_progress_max_seconds":2},{"ring_entry_progress_max_seconds":100},
    {"corner_exit_confirm_frames":1},{"corner_exit_confirm_frames":2.5},{"corner_exit_error":0},
    {"ring_entry_progress_max_seconds":"6"},{"corner_exit_confirm_frames":float("nan")}])
def test_new_tuning_rejects_unsafe_values(changes):
    with pytest.raises(ValueError):replace(PatrolTuning(),**changes).validate()
