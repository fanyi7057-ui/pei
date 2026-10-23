"""Static contract for the intentionally compact competition workspace."""

from pathlib import Path


def test_competition_tree_has_only_required_ros_packages_and_assets():
    src_root = Path(__file__).resolve().parents[2]

    assert {path.name for path in src_root.iterdir() if path.is_dir()} == {
        "motor_temperature_pkg",
        "wheel_robot",
        "wheeled_legged_pkg",
    }

    package = src_root / "wheel_robot"
    required_paths = (
        "scripts/icn.py",
        "scripts/race_runtime.py",
        "scripts/race_posture.py",
        "scripts/stand_gate.py",
        "scripts/stand_only.py",
        "scripts/stand_only_core.py",
        "scripts/step_detector_node.py",
        "scripts/step_approach_test.py",
        "scripts/step_preview.py",
        "scripts/depth_preview.py",
        "launch/icn.launch.py",
        "launch/stand_only.launch.py",
        "launch/step_approach_test.launch.py",
        "config/astra_camera.yaml",
        "config/icn.yaml",
        "config/step_approach_test.yaml",
        "config/step_jump.yaml",
        "vendor/OrbbecSDK_ROS2/orbbec_camera/SDK/include/libobsensor/ObSensor.hpp",
        "vendor/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64",
    )
    for relative_path in required_paths:
        assert (package / relative_path).exists(), relative_path


def test_competition_cmake_has_no_tutorial_install_or_test_references():
    cmake = (
        Path(__file__).resolve().parents[1] / "CMakeLists.txt"
    ).read_text(encoding="utf-8")

    for removed_name in (
        "camera_recorder.py",
        "camera_viewer.py",
        "dual_line_follower.py",
        "keyboard_teleop.py",
        "lesson17_vision_follower.py",
        "lesson18_person_patrol.py",
        "lessop16_opencv.py",
        "line_follower.py",
        "shiwai.py",
        "scripts/models",
        "test_lesson17_vision_follower.py",
        "test_lesson18_person_patrol.py",
    ):
        assert removed_name not in cmake
