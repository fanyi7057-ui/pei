"""Regression checks for the robot-transfer checksum manifest."""

import hashlib
from pathlib import Path


def test_manifest_covers_the_menu_and_safe_stand_files():
    root = Path(__file__).resolve().parents[4]
    manifest = root / "checksums" / "SHA256SUMS.txt"
    entries = {
        line.split(maxsplit=1)[1].strip()
        for line in manifest.read_text(encoding="utf-8").splitlines()
        if line.strip()
    }

    assert "tools/robot_menu.sh" in entries
    assert "ros2_ws/src/wheel_robot/scripts/stand_only.py" in entries
    assert "ros2_ws/src/wheel_robot/scripts/race_runtime.py" in entries
    assert "ros2_ws/src/wheel_robot/test/test_live_race_tuning.py" in entries
    assert "ros2_ws/src/wheel_robot/test/test_live_race_contract.py" in entries
    assert "ros2_ws/src/wheel_robot/test/test_live_tuning_file_reload.py" in entries
    assert "ros2_ws/src/wheel_robot/launch/stand_only.launch.py" in entries


def test_manifest_hashes_match_the_recorded_source_files():
    root = Path(__file__).resolve().parents[4]
    manifest = root / "checksums" / "SHA256SUMS.txt"

    for line in manifest.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        expected_hash, relative_path = line.split(maxsplit=1)
        actual_hash = hashlib.sha256(
            (root / relative_path.strip()).read_bytes()
        ).hexdigest()
        assert actual_hash == expected_hash
