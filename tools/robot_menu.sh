#!/usr/bin/env bash
# V21包内入口：普通巡线回V11；保留V12偏航修复与旧二进制检查。
set -u
TWO_TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || exit 1
source "$TWO_TOOLS/workspace_env.sh" || exit 1
[[ -t 0 ]] || { printf '请在交互SSH终端中运行菜单。\n'; exit 1; }
export DISPLAY=:0
export XAUTHORITY=/home/orangepi/.Xauthority
export QT_X11_NO_MITSHM=1

while true; do
  printf '\n===== V21 本包机器人菜单（普通巡线V11）=====\n工程：%s\n' "$TWO_ROOT"
  printf '1. 编译并检查本包环境（未编译偏航修复时必须先选）\n2. 视觉干跑：巡线/环岛/横道，不启动底盘\n3. 原地站立检查（沿用原站立程序）\n4. V21巡线＋环岛＋横道/终点测试（不跳台阶）\n5. 原比赛流程（含原台阶；共享底盘偏航发布修复）\n6. 查看本包环境与设备\n0. 退出\n'
  read -r -p '请选择：' two_option || break
  case "$two_option" in
    1) two_build_workspace ;;
    2) bash "$TWO_TOOLS/patrol_test_menu.sh" --dry ;;
    3|5)
      printf '本项会启动底盘：先完成调零/使能，清场，保持遥控急停可用。\n'
      read -r -p '1确认，其他取消：' two_confirm
      [[ "$two_confirm" == 1 ]] || continue
      if two_load_workspace && two_check_idle; then
        if [[ "$two_option" == 3 ]]; then
          ros2 launch "$TWO_WS/src/wheel_robot/launch/stand_only.launch.py"
        else
          printf '原比赛流程不是V21测试；仍使用原比赛参数，共享修复后的IMU发布。启动后在另一个已加载本包环境的终端发送/race_start_confirm；Ctrl+C结束。\n'
          ros2 launch "$TWO_WS/src/wheel_robot/launch/icn.launch.py" \
            enable_motion:=true show_debug:=true start_base:=true start_step_support:=true
        fi
      fi ;;
    4) bash "$TWO_TOOLS/patrol_test_menu.sh" --real ;;
    6)
      printf '当前根目录：%s\n自有编译安装目录：%s\n实时参数：%s\n' "$TWO_ROOT" "$TWO_INSTALL" "$TWO_WS/src/wheel_robot/config/patrol_test.yaml"
      xset q >/dev/null 2>&1 && printf '小车屏幕连接正常。\n' || printf '小车屏幕暂不可用。\n'
      printf '串口设备：\n'; compgen -G '/dev/ttyUSB*' || true ;;
    0) break ;;
    *) printf '请输入菜单中的编号。\n' ;;
  esac
done
