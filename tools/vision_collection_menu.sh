#!/usr/bin/env bash
set -u
TOOLS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || exit 1
printf '\n1. 原地每5秒拍照（零轮速、腿高0.15、低头25度）\n2. 行驶采集（巡线、视频、逐帧日志、每秒抽样图片）\n0. 返回\n'
read -r -p '请选择：' choice || exit 0
case "$choice" in
  1) bash "$TOOLS/camera_capture_menu.sh" ;;
  2)
    root="$(cd -- "$TOOLS/.." && pwd -P)" || exit 1
    export PATROL_RECORD_DIR="$root/log/driving_capture/$(date +%Y%m%d_%H%M%S)_$$"
    printf '本次采集目录：%s\n' "$PATROL_RECORD_DIR"
    bash "$TOOLS/patrol_test_menu.sh" --real ;;
  *) exit 0 ;;
esac
