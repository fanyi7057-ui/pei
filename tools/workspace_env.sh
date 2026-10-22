#!/usr/bin/env bash
# 两个菜单共用：所有工程路径由本文件位置解析，不借用其它工程。
TWO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || return 1
TWO_WS="$TWO_ROOT/ros2_ws"
TWO_INSTALL="$TWO_WS/install_patrol"

two_source_file() {
  local two_status
  set +u
  source "$1"
  two_status=$?
  set -u
  return "$two_status"
}

two_source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] || { printf '缺少系统ROS Humble：/opt/ros/humble/setup.bash\n'; return 1; }
  # Do not inherit old project overlays from MobaXterm/.bashrc.
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
  export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  two_source_file /opt/ros/humble/setup.bash
}

two_prepare_sdk() {
  local two_arch two_sdk two_dir
  case "$(uname -m)" in
    aarch64|arm64) two_arch=arm64 ;;
    x86_64|amd64) two_arch=x64 ;;
    *) printf '不支持的SDK架构。\n'; return 1 ;;
  esac
  two_sdk="$TWO_WS/src/wheel_robot/vendor/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/$two_arch"
  [[ -s "$two_sdk/libOrbbecSDK.so.1.10.37" ]] || { printf '本包缺少本机架构的Orbbec SDK：%s\n' "$two_sdk"; return 1; }
  # Exact package-local linker/SONAME links, never /usr/lib or another project.
  for two_dir in "$two_sdk" "$TWO_INSTALL/wheel_robot/lib/wheel_robot"; do
    [[ -d "$two_dir" ]] || continue
    [[ -s "$two_dir/libOrbbecSDK.so.1.10.37" ]] || continue
    [[ ! -d "$two_dir/libOrbbecSDK.so" && ! -d "$two_dir/libOrbbecSDK.so.1.10" ]] || return 1
    ln -sfn libOrbbecSDK.so.1.10.37 "$two_dir/libOrbbecSDK.so" || return 1
    ln -sfn libOrbbecSDK.so.1.10.37 "$two_dir/libOrbbecSDK.so.1.10" || return 1
  done
  export LD_LIBRARY_PATH="$TWO_INSTALL/wheel_robot/lib/wheel_robot:$two_sdk${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
}

two_load_workspace() {
  local two_package two_prefix
  two_source_ros || return 1
  [[ -f "$TWO_INSTALL/local_setup.bash" ]] || {
    printf '本包尚未编译。请先运行本包tools/robot_menu.sh，选择1；不会借用其它目录。\n'
    return 1
  }
  two_source_file "$TWO_INSTALL/local_setup.bash" || return 1
  for two_package in wheel_robot wheeled_legged_pkg; do
    two_prefix="$(ros2 pkg prefix "$two_package")" || return 1
    [[ "$(realpath -- "$two_prefix")" == "$TWO_INSTALL/$two_package" ]] || {
      printf '拒绝加载其它工程的%s：%s\n' "$two_package" "$two_prefix"; return 1;
    }
  done
  two_prepare_sdk || return 1
  printf '已加载本包：%s\n' "$TWO_INSTALL"
}

two_build_workspace() {
  local two_cache two_source
  two_source_ros || return 1
  two_check_idle || return 1
  command -v colcon >/dev/null || { printf '系统缺少colcon，请安装ROS编译工具。\n'; return 1; }
  [[ -f "$TWO_WS/src/wheel_robot/package.xml" ]] || return 1
  two_prepare_sdk || return 1
  # Dedicated trees avoid old symlink/non-symlink caches; never delete files.
  for two_cache in "$TWO_WS"/build_patrol/*/CMakeCache.txt; do
    [[ -f "$two_cache" ]] || continue
    two_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$two_cache")"
    case "$two_source" in
      "$TWO_WS/src/"*) ;;
      *) printf '检测到搬家后的旧编译缓存：%s；请保留源码，重新解压干净包编译。未删除任何文件。\n' "$two_cache"; return 1 ;;
    esac
  done
  (
    cd -- "$TWO_WS" || exit 1
    colcon --log-base log_patrol build --build-base build_patrol --install-base install_patrol \
      --packages-select wheeled_legged_pkg motor_temperature_pkg wheel_robot \
      --cmake-args -DBUILD_TESTING=OFF
  ) || { printf '编译失败：未启动任何底盘/比赛节点。\n'; return 1; }
  two_load_workspace || return 1
  printf '本包编译及加载完成。实时YAML/Python源码仍直接从本包读取。\n'
}

two_check_idle() {
  local two_nodes
  two_nodes="$(timeout 5 ros2 node list 2>/dev/null)" || { printf '无法检查ROS节点，未执行编译/启动。\n'; return 1; }
  if [[ "$two_nodes" == *"/wl_base_node"* || "$two_nodes" == *"/wheelleg_vision_race"* || "$two_nodes" == *"/wheelleg_stand_only"* ]]; then
    printf '已有底盘/比赛/站立程序运行，请在原终端Ctrl+C结束。不会强杀其它程序。\n'
    return 1
  fi
}
