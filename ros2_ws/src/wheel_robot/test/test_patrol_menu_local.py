"""Run package-local environment helpers with fake ROS/build commands, no motors."""
from pathlib import Path
import os
import shutil
import subprocess

import pytest

TOOLS=Path(__file__).resolve().parents[4]/"tools"
BASH=Path("C:/Program Files/Git/bin/bash.exe") if os.name=="nt" else shutil.which("bash")


def run_bash(script,*args):
    if not BASH or not Path(BASH).is_file():
        pytest.skip("Bash unavailable")
    return subprocess.run([str(BASH),"--noprofile","--norc","-c",script,"test",*map(str,args)],
                          capture_output=True,text=True,encoding="utf-8",timeout=10)


@pytest.fixture
def helper(tmp_path):
    root=tmp_path/"package with spaces"/"Two"
    (root/"tools").mkdir(parents=True)
    target=root/"tools/workspace_env.sh"
    target.write_bytes((TOOLS/"workspace_env.sh").read_bytes().replace(b"\r\n",b"\n"))
    sdk=root/"ros2_ws/src/wheel_robot/vendor/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64"
    sdk.mkdir(parents=True)
    (sdk/"libOrbbecSDK.so.1.10.37").write_bytes(b"fake SDK for link tests, never loaded")
    (root/"ros2_ws/src/wheel_robot/package.xml").write_text("<package/>")
    return target


def test_menu_shell_syntax():
    for name in ("robot_menu.sh","patrol_test_menu.sh","workspace_env.sh"):
        result=run_bash('bash -n "$1"',TOOLS/name)
        assert result.returncode==0,result.stderr


@pytest.mark.parametrize("state",["missing","old","correct"])
def test_v12_old_driver_binary_cannot_pass_load_guard(helper,state):
    binary=helper.parents[1]/"ros2_ws/install_patrol/wheeled_legged_pkg/lib/wheeled_legged_pkg/wl_base_node"
    if state!="missing":
        binary.parent.mkdir(parents=True)
        binary.write_bytes(b"IMU_QUATERNION_MAPPING_V12" if state=="correct" else b"old norm-valid but semantically wrong quaternion driver")
    result=run_bash('source "$1"; two_verify_heading_binary',helper)
    assert (result.returncode==0)==(state=="correct")
    if state!="correct":assert "重新编译" in result.stdout


def test_helper_resolves_its_own_root_with_spaces_not_cwd(helper):
    result=run_bash('source "$1"; printf "%s" "$TWO_ROOT"',helper)
    assert result.returncode==0,result.stderr
    assert result.stdout.endswith("/package with spaces/Two")


def test_missing_local_install_refuses_external_fallback(helper):
    result=run_bash('source "$1"; two_source_ros(){ :; }; two_load_workspace',helper)
    assert result.returncode!=0
    assert "尚未编译" in result.stdout
    assert "source /home/" not in (TOOLS/"patrol_test_menu.sh").read_text(encoding="utf-8")


def test_own_prefix_is_required_even_if_another_project_is_in_environment(helper):
    install=helper.parents[1]/"ros2_ws/install_patrol"
    install.mkdir()
    (install/"local_setup.bash").write_text(":\n")
    script='source "$1"; two_source_ros(){ :; }; ros2(){ echo /tmp/unrelated-project; }; two_load_workspace'
    result=run_bash(script,helper)
    assert result.returncode!=0 and "拒绝加载" in result.stdout


def test_build_is_local_and_failure_cannot_start_nodes(helper):
    script='''source "$1"
two_source_ros(){ :; }
two_check_idle(){ :; }
uname(){ echo aarch64; }
colcon(){ printf '%s\\n' "$@" > "$TWO_ROOT/build-arguments.txt"; return 7; }
two_build_workspace
'''
    result=run_bash(script,helper)
    assert result.returncode!=0 and "未启动任何" in result.stdout
    args=(helper.parents[1]/"build-arguments.txt").read_text()
    assert "build_patrol" in args and "install_patrol" in args and "log_patrol" in args
    assert "--symlink-install" not in args
    assert not (helper.parents[1]/"ros2_ws/install").exists()


def test_relocated_build_cache_is_refused_without_delete(helper):
    cache=helper.parents[1]/"ros2_ws/build_patrol/wheel_robot/CMakeCache.txt"
    cache.parent.mkdir(parents=True)
    original="CMAKE_HOME_DIRECTORY:INTERNAL=/tmp/some-other-workspace/src/wheel_robot\n"
    cache.write_text(original)
    script='source "$1"; two_source_ros(){ :; }; two_check_idle(){ :; }; uname(){ echo aarch64; }; colcon(){ return 99; }; two_build_workspace'
    result=run_bash(script,helper)
    assert result.returncode!=0 and "旧编译缓存" in result.stdout
    assert cache.read_text()==original


def test_live_robot_nodes_block_build_without_killing_other_processes(helper):
    script='source "$1"; two_source_ros(){ :; }; timeout(){ echo /wl_base_node; }; two_build_workspace'
    result=run_bash(script,helper)
    assert result.returncode!=0 and "已有底盘" in result.stdout


def test_successful_local_load_keeps_package_library_path(helper):
    install=helper.parents[1]/"ros2_ws/install_patrol"
    for name in ("wheel_robot","wheeled_legged_pkg"):
        (install/name).mkdir(parents=True)
    (install/"local_setup.bash").write_text(":\n")
    binary=install/"wheeled_legged_pkg/lib/wheeled_legged_pkg/wl_base_node"
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"fake executable marker IMU_QUATERNION_MAPPING_V12")
    script='''source "$1"
two_source_ros(){ :; }
uname(){ echo aarch64; }
ros2(){ printf '%s/%s\\n' "$TWO_INSTALL" "$3"; }
two_load_workspace
'''
    result=run_bash(script,helper)
    assert result.returncode==0,result.stdout+result.stderr
    assert "已加载本包" in result.stdout
