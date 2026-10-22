"""Tests for data used by the stand-only ROS2 node."""

from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from stand_only_core import STAND_JOINT_NAMES, StandCommand, safe_stand_height


def test_default_command_is_stationary_and_has_expected_joints():
    command = StandCommand.default()

    assert command.linear_x == 0.0
    assert command.angular_z == 0.0
    assert command.jump is False
    assert command.joint_names == STAND_JOINT_NAMES
    assert command.joint_positions == (0.25, 0.0, 0.0, 0.0)


def test_invalid_height_falls_back_to_safe_default():
    assert safe_stand_height(float("nan")) == 0.25
    assert safe_stand_height(0.14) == 0.25
    assert safe_stand_height(0.36) == 0.25
    assert safe_stand_height(0.30) == 0.30


def test_stand_command_accepts_complete_race_posture():
    command = StandCommand.from_posture(0.20, 0.0, 25.0, 0.0)

    assert command.linear_x == 0.0
    assert command.angular_z == 0.0
    assert command.jump is False
    assert command.joint_positions == (0.20, 0.0, 25.0, 0.0)


def test_stand_node_is_packaged_and_launch_starts_the_bridge():
    package = Path(__file__).resolve().parents[1]
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    launch = (package / "launch" / "stand_only.launch.py").read_text(
        encoding="utf-8"
    )

    assert "scripts/stand_only.py" in cmake
    assert "scripts/stand_only_core.py" in cmake
    assert 'executable="stand_only.py"' in launch
    assert 'executable="wl_base_node"' in launch
    assert '"stand_pitch_deg"' in launch
    assert '"stand_roll_deg"' in launch
    assert '"stand_slide_m"' in launch


def test_stand_node_cleanup_keeps_destroying_after_publish_error():
    package = Path(__file__).resolve().parents[1]
    node = (package / "scripts" / "stand_only.py").read_text(encoding="utf-8")

    assert "except Exception as error:" in node
    assert "清理时安全站立消息发送失败" in node
