"""Static contract tests for the robot-side Bash menu."""

from pathlib import Path


def test_menu_source_uses_linux_line_endings():
    root = Path(__file__).resolve().parents[4]
    raw_script = (root / "tools" / "robot_menu.sh").read_bytes()

    assert b"\r\n" not in raw_script


def test_robot_launch_scripts_use_linux_line_endings():
    root = Path(__file__).resolve().parents[4]
    scripts = root / "ros2_ws" / "src" / "wheel_robot" / "scripts"

    for script_name in ("icn.py", "step_detector_node.py", "stand_only.py", "race_operator_console.py"):
        assert b"\r\n" not in (scripts / script_name).read_bytes()


def test_menu_allows_ros_setup_scripts_that_read_unset_variables():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert "source_setup_file()" in script
    assert "set +u" in script
    assert 'source "$setup_file"' in script
    assert "set -u" in script
    assert 'source "$ROS_SETUP"' not in script
    assert 'source "$WORKSPACE/install/setup.bash"' not in script
    assert script.count(
        'source_setup_file "$WORKSPACE/install/setup.bash" || return'
    ) == 2


def test_menu_makes_python_nodes_executable_before_a_symlink_build():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    permission_fix = 'chmod +x "$WORKSPACE/src/wheel_robot/scripts/"*.py'
    assert permission_fix in script
    assert script.index(permission_fix) < script.index("colcon build --symlink-install")


def test_menu_prepares_the_orbbec_runtime_library_for_camera_launches():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert "prepare_orbbec_runtime()" in script
    assert 'libOrbbecSDK.so.1.10.37' in script
    assert 'libOrbbecSDK.so.1.10' in script
    assert 'export LD_LIBRARY_PATH="$RUNTIME_LIBRARY_DIR' in script
    assert script.count("prepare_orbbec_runtime || return") == 4
    stand_only_section = script.split("stand_only_check()", 1)[1].split(
        "formal_race()", 1
    )[0]
    formal_race_section = script.split("formal_race()", 1)[1].split(
        "show_devices()", 1
    )[0]
    assert "prepare_orbbec_runtime || return" in stand_only_section
    assert "prepare_orbbec_runtime || return" in formal_race_section


def test_menu_has_fixed_target_and_only_task_relevant_entries():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert 'TARGET="/home/orangepi/robot"' in script
    assert "1. 编译比赛程序" in script
    assert "2. 检查与预览" in script
    assert "3. 比赛姿态原地站立检查" in script
    assert "4. 支架底盘联合检查" in script
    assert "5. 视觉与台阶识别预览" in script
    assert "6. 正式比赛（含每 2.5 秒状态打印）" in script
    assert "7. 查看最近一次比赛日志" in script
    assert "8. 查看使用方法" in script
    assert 'rm -rf -- "$TARGET"' not in script
    assert "delete_deployment" not in script


def test_menu_uses_existing_safe_launch_arguments():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert "enable_motion:=false start_base:=false start_step_support:=true" in script
    assert (
        "enable_motion:=false start_base:=true serial_port:=/dev/ttyUSB0 "
        "start_step_support:=true"
    ) in script


def test_menu_refuses_launching_an_unbuilt_workspace():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert "source_built_workspace" in script
    assert '[[ ! -f "$WORKSPACE/install/setup.bash" ]]' in script
    assert "请先在主菜单选择 1 编译比赛程序" in script
    assert script.count("source_built_workspace || return") == 4
    assert "stand_only.launch.py serial_port:=/dev/ttyUSB0" in script
    assert (
        "enable_motion:=true start_base:=true serial_port:=/dev/ttyUSB0 "
        "start_step_support:=true"
    ) in script


def test_menu_requires_a_successful_build_in_the_current_menu_session():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert "BUILT_THIS_SESSION=0" in script
    assert "BUILT_THIS_SESSION=1" in script
    assert "当前菜单会话尚未成功编译" in script


def test_race_uses_ros_start_confirmation_not_terminal_stdin():
    root = Path(__file__).resolve().parents[4]
    source = (root / "ros2_ws" / "src" / "wheel_robot" / "scripts" / "icn.py").read_text(
        encoding="utf-8"
    )

    assert '"/race_start_confirm"' in source
    assert "accept_ros_start_confirmation" in source
    assert "def _stand_confirm_worker" not in source


def test_menu_explains_current_terminal_numeric_controls():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")
    console = (
        root
        / "ros2_ws"
        / "src"
        / "wheel_robot"
        / "scripts"
        / "race_operator_console.py"
    ).read_text(encoding="utf-8")

    assert "当前终端输入数字" in script
    assert "第二个 SSH 终端" not in script
    assert "def handle_command" in console
    assert 'command == "1"' in console
    assert 'command == "2"' not in console
    assert 'command == "3"' not in console
    assert 'command == "0"' in console
    assert "request_full_race" in console
    assert "read_operator_commands" in console
    assert "threading.Thread" in console
    assert "commands.get_nowait" in console


def test_menu_prepares_robot_local_display_without_blocking_builds():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")

    assert "prepare_robot_display()" in script
    assert "export DISPLAY=:0" in script
    assert 'export XAUTHORITY="/home/orangepi/.Xauthority"' in script
    assert "xset q >/dev/null 2>&1" in script
    assert "小车屏幕连接正常" in script
    assert "小车屏幕不可用" in script


def test_menu_option_three_uses_the_selected_complete_race_posture():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")
    section = script.split("stand_only_check()", 1)[1].split("formal_race()", 1)[0]

    assert "比赛姿态原地站立检查" in section
    assert "stand_height_m:=0.15" in section
    assert "stand_roll_deg:=0.0" in section
    assert "stand_pitch_deg:=28.0" in section
    assert "stand_slide_m:=0.0" in section


def test_race_vision_node_is_pinned_to_the_robot_local_display():
    package = Path(__file__).resolve().parents[1]
    launch = (package / "launch" / "icn.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n                "local_display"' in launch
    assert 'DeclareLaunchArgument(\n                "local_xauthority"' in launch
    assert '"DISPLAY": LaunchConfiguration("local_display")' in launch
    assert '"XAUTHORITY": LaunchConfiguration("local_xauthority")' in launch
    assert '"QT_X11_NO_MITSHM": "1"' in launch


def test_stand_only_launch_includes_a_robot_local_camera_preview():
    package = Path(__file__).resolve().parents[1]
    launch = (package / "launch" / "stand_only.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'executable="astra_camera_node"' in launch
    assert 'executable="depth_distance_node"' in launch
    assert 'executable="depth_preview.py"' in launch
    assert '"DISPLAY": LaunchConfiguration("local_display")' in launch
    assert '"XAUTHORITY": LaunchConfiguration("local_xauthority")' in launch


def test_menu_stand_only_prepares_camera_runtime_and_explains_local_preview():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")
    section = script.split("stand_only_check()", 1)[1].split("formal_race()", 1)[0]

    assert "prepare_orbbec_runtime || return" in section
    assert "小车屏幕" in section


def test_formal_race_uses_quiet_operator_console_and_step_topic():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "robot_menu.sh").read_text(encoding="utf-8")
    assert "race_operator_console.py" in script
    assert "/step/execute_jump" in script
    assert '>$launch_log' not in script  # quoted redirect is required
    assert '>"$launch_log" 2>&1 &' in script


def test_adaptive_node_prints_read_only_status_every_two_point_five_seconds():
    package = Path(__file__).resolve().parents[1]
    source = (package / "scripts" / "icn_adaptive.py").read_text(encoding="utf-8")

    assert 'self.create_timer(2.5, self._log_robot_status)' in source
    assert '"/robot_joint_states"' in source
    assert "def _log_robot_status" in source
    assert "状态[2.5s]" in source
    assert "黑线=" in source
    assert "台阶=" in source
