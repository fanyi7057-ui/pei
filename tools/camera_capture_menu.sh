#!/usr/bin/env bash
set -u
TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || exit 1
source "$TOOLS/workspace_env.sh" || exit 1
two_load_workspace && two_check_idle || exit 1
capture_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$capture_pid" ]] && kill -0 "$capture_pid" 2>/dev/null; then
    kill -INT -- "-$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM
setsid ros2 launch "$TWO_WS/src/wheel_robot/launch/camera_capture.launch.py" &
capture_pid=$!
printf '站立拍照：零轮速，腿高0.15m，低头25度。每5秒自动拍照。1再拍一组，0退出。\n'
while kill -0 "$capture_pid" 2>/dev/null; do
  capture_command=""
  if read -r -t 1 capture_command; then
    case "$capture_command" in
      1) timeout 5 ros2 topic pub --once /camera_capture/request std_msgs/msg/Bool '{data: true}' ;;
      0) break ;;
    esac
  fi
done
