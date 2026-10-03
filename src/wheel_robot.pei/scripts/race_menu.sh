#!/usr/bin/env bash
set -u

SCRIPT_PATH="$(readlink -f "$0")"
PACKAGE_SOURCE="$(cd "$(dirname "$SCRIPT_PATH")/.." && pwd)"
WORKSPACE="$(cd "$(dirname "$SCRIPT_PATH")/../../.." && pwd)"

prepare_ros() {
  source /opt/ros/humble/setup.bash
  if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
  fi
}

while true; do
  echo
  echo "========== 灵动越障菜单 =========="
  echo "1) 开始比赛"
  echo "2) 只编译当前比赛版本（wheel_robot.pei）"
  echo "3) 仅显示摄像头调试画面（不驱动车轮）"
  echo "4) 查看最近巡线录像和日志"
  echo "0) 退出"
  read -r -p "请输入数字: " choice
  case "$choice" in
    1) prepare_ros; ros2 launch wheel_robot race.launch.py ;;
    # --paths makes colcon scan only this package.  The workspace can retain
    # old wheel_robot copies without their duplicate package name blocking us.
    2) prepare_ros; cd "$WORKSPACE" || exit 1; colcon build --paths "$PACKAGE_SOURCE" --symlink-install ;;
    3) prepare_ros; ros2 launch wheel_robot race.launch.py enable_motion:=false ;;
    4) ls -lt "$WORKSPACE/recordings" 2>/dev/null | head -n 12 ;;
    0) exit 0 ;;
    *) echo "无效输入，请输入 0、1、2、3 或 4。" ;;
  esac
done
