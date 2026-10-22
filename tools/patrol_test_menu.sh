#!/usr/bin/env bash
# V13测试子菜单；加载本包编译环境，不借用其它工程。
set -u
PATROL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || exit 1
PATROL_WS="$PATROL_ROOT/ros2_ws"
PATROL_LAUNCH="$PATROL_WS/src/wheel_robot/launch/patrol_test.launch.py"
PATROL_PID=""

cleanup_patrol() {
  if [[ -n "$PATROL_PID" ]] && kill -0 "$PATROL_PID" 2>/dev/null; then
    # Only signal the new setsid process group created below, never killall.
    kill -INT -- "-$PATROL_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$PATROL_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$PATROL_PID" 2>/dev/null; then
      kill -TERM -- "-$PATROL_PID" 2>/dev/null || true
    fi
  fi
  if [[ -n "$PATROL_PID" ]]; then
    wait "$PATROL_PID" 2>/dev/null || true
    PATROL_PID=""
  fi
}
trap cleanup_patrol EXIT
trap 'exit 130' INT TERM

if [[ ! -t 0 ]]; then
  printf '本菜单需要交互终端，拒绝无终端启动。\n'
  exit 1
fi
source "$PATROL_ROOT/tools/workspace_env.sh" || exit 1
two_load_workspace || exit 1
if ! python3 -c 'import cv2; import numpy; import yaml; from wheel_robot.msg import StepDetection'; then
  printf '缺少OpenCV/numpy或原包已生成的ROS消息，未启动测试。\n'
  exit 1
fi
export DISPLAY=:0
export XAUTHORITY=/home/orangepi/.Xauthority
export QT_X11_NO_MITSHM=1
if ! xset q >/dev/null 2>&1; then
  printf '车载屏幕连接失败，未启动测试；请检查HDMI桌面及Xauthority。\n'
  exit 1
fi
printf '小车屏幕连接正常。实时参数：%s\n' "$PATROL_WS/src/wheel_robot/config/patrol_test.yaml"
if ! command -v setsid >/dev/null || ! command -v timeout >/dev/null; then
  printf '缺少Ubuntu标准工具setsid/timeout，未启动。\n'
  exit 1
fi
patrol_nodes="$(timeout 5 ros2 node list 2>/dev/null)" || { printf 'ROS节点查询失败，未启动。\n'; exit 1; }
if [[ "$patrol_nodes" == *"/wheelleg_vision_race"* || "$patrol_nodes" == *"/wl_base_node"* || "$patrol_nodes" == *"/wheelleg_stand_only"* ]]; then
  printf '已有比赛或底盘节点在运行，请先在原终端Ctrl+C退出；禁止两个程序抢/cmd_vel。\n'
  exit 1
fi
printf '\n===== V13巡线＋VV虚线环岛＋横道/终点（不执行台阶）=====\n'
printf '正确路线：沿底部主路先通过5号出口岔口，再从4号进入、5号出环。VV虚线确认1帧；参数在config/patrol_test.yaml。\n'
case "${1:-}" in
  --dry) patrol_option=1 ;;
  --real) patrol_option=2 ;;
  *) printf '1. 视觉干跑，不启动底盘\n2. 实车测试（按实时开关）\n0. 退出\n'
     read -r -p '请选择：' patrol_option ;;
esac
case "$patrol_option" in
  1) patrol_motion=false; patrol_base=false ;;
  2)
    printf '默认巡航上限0.40、起步/重新咬线0.20、弯道0.25~0.28、环岛最高0.30、丢线最高0.10m/s。30/45秒只报警不算完成环岛；陀螺仪保底需真实转角+实测前进行程，出环仍需视觉确认。第一次横道停车3秒，第二次横道后走1m结束；不跳台阶、不录像。请调零/使能、清场并保留遥控急停。\n'
    read -r -p '确认在安全普通赛道测试：1继续，其他取消：' patrol_confirm
    [[ "$patrol_confirm" == 1 ]] || exit 0
    patrol_motion=true; patrol_base=true ;;
  *) exit 0 ;;
esac
set +m
setsid ros2 launch "$PATROL_LAUNCH" enable_motion:="$patrol_motion" start_base:="$patrol_base" &
PATROL_PID=$!
printf '\n启动后先站立等待。看到“已站立”再输入：\n1 开始/继续当前任务；2 停车并保留阶段；3 清零任务，从起点重跑（仍等待1）；0 退出。Ctrl+C也会退出。\n'
while kill -0 "$PATROL_PID" 2>/dev/null; do
  patrol_command=""
  if ! read -r -t 1 patrol_command; then
    continue
  fi
  case "$patrol_command" in
    1) timeout 5 ros2 topic pub --once /race_start_confirm std_msgs/msg/Bool '{data: true}' || printf '开始确认发送失败，请查看测试日志。\n' ;;
    2) timeout 5 ros2 topic pub --once /patrol_test_stop std_msgs/msg/Bool '{data: true}' || printf '停车话题发送失败，请立即使用遥控急停或Ctrl+C。\n' ;;
    3) timeout 5 ros2 topic pub --once /patrol_test_reset_course std_msgs/msg/Bool '{data: true}' || printf '新场重置发送失败，请先急停再检查。\n' ;;
    0) break ;;
    '') ;;
    *) printf '输入1继续、2暂停、3从起点重置、0退出。\n' ;;
  esac
done
