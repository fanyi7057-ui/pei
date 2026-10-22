"""Unit tests for the pre-race stand-confirmation state gate."""

from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from stand_gate import (
    StandGate,
    StandGateState,
    is_enter_confirmation,
    safe_stand_height,
)


def test_disabled_gate_never_blocks_motion():
    gate = StandGate(enabled=False, settle_seconds=2.0)

    assert gate.advance(10.0) is False
    assert gate.state is StandGateState.DISABLED


def test_gate_waits_then_needs_confirm_before_unlocking():
    gate = StandGate(enabled=True, settle_seconds=2.0)

    assert gate.advance(10.0) is True
    assert gate.state is StandGateState.STAND_SETTLE
    assert gate.advance(11.99) is True
    assert gate.advance(12.0) is True
    assert gate.state is StandGateState.STAND_CONFIRM
    assert gate.confirm() is True
    assert gate.advance(12.01) is False
    assert gate.state is StandGateState.RACE_ACTIVE


def test_confirmation_is_ignored_before_wait_finishes():
    gate = StandGate(enabled=True, settle_seconds=2.0)

    gate.advance(0.0)

    assert gate.confirm() is False
    assert gate.state is StandGateState.STAND_SETTLE


def test_ros_start_confirmation_is_accepted_only_after_the_stand_hold():
    gate = StandGate(enabled=True, settle_seconds=2.0)

    assert gate.accept_ros_start_confirmation() is False
    gate.advance(10.0)
    assert gate.accept_ros_start_confirmation() is False
    gate.advance(12.0)
    assert gate.accept_ros_start_confirmation() is True
    assert gate.state is StandGateState.RACE_ACTIVE


def test_zero_second_gate_still_requires_manual_confirmation():
    gate = StandGate(enabled=True, settle_seconds=0.0)

    assert gate.advance(5.0) is True
    assert gate.state is StandGateState.STAND_SETTLE
    assert gate.advance(5.0) is True
    assert gate.state is StandGateState.STAND_CONFIRM
    assert gate.advance(99.0) is True


def test_only_an_empty_line_confirms_the_start_gate():
    assert is_enter_confirmation("") is True
    assert is_enter_confirmation("abc") is False
    assert is_enter_confirmation(" ") is False


def test_invalid_stand_height_falls_back_to_default():
    assert safe_stand_height(0.25) == 0.25
    assert safe_stand_height(float("nan")) == 0.25
    assert safe_stand_height(0.0) == 0.25
    assert safe_stand_height(0.50) == 0.25
