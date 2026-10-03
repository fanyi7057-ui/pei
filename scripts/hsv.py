#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时黄色标定，无 ROS、无运动控制、无黄黑胶带过滤。
运行: python3 yellow_live_calibrate.py
左键拖框重新取样（覆盖旧阈值）；空格暂停/继续；S保存；R重置；Q退出。
依赖: python3-opencv python3-numpy；需在有图形桌面的终端运行。
"""
import argparse
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np

CAMERA = 'Camera - drag ROI here'
MASK = 'Mask - white is selected'
RESULT = 'Yellow result'
CONTROL = 'HSV controls'
DEFAULT = [12, 42, 50, 70]
BARS = ['LOW_H', 'HIGH_H', 'LOW_S', 'LOW_V']


def estimate(hsv_roi):
    """2/98百分位抑制少量离群像素，加容差；S/V上限与shiwai一致。"""
    pixels = hsv_roi.reshape(-1, 3)
    low = np.percentile(pixels, 2, axis=0)
    high = np.percentile(pixels, 98, axis=0)
    return [max(0, int(np.floor(low[0])) - 5),
            min(179, int(np.ceil(high[0])) + 5),
            max(0, int(np.floor(low[1])) - 20),
            max(0, int(np.floor(low[2])) - 20)]


def parameters(values):
    return dict(zip(['yellow_h_min', 'yellow_h_max',
                     'yellow_s_min', 'yellow_v_min'], map(int, values)))


class Calibrator:
    def __init__(self, args):
        self.args = args
        self.displayed = None
        self.frame = None
        self.sample = None
        self.dragging = False
        self.paused = False
        self.start = (0, 0)
        self.end = (0, 0)
        self.rect = None
        self.selection_id = 0
        self.last_values = None
        self.fps = 0.0
        self.last_frame_at = time.monotonic()
        # AUTOSIZE keeps mouse coordinates identical to image coordinates.
        cv2.namedWindow(CAMERA, cv2.WINDOW_AUTOSIZE)
        for name in (MASK, RESULT, CONTROL):
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        for name, value, maximum in zip(BARS, DEFAULT, [179, 179, 255, 255]):
            cv2.createTrackbar(name, CONTROL, value, maximum, lambda _: None)
        cv2.setMouseCallback(CAMERA, self.mouse)

    def values(self):
        values = [cv2.getTrackbarPos(n, CONTROL) for n in BARS]
        if values[0] > values[1]:
            values[1] = values[0]
            cv2.setTrackbarPos('HIGH_H', CONTROL, values[1])
        return values

    def set_values(self, values):
        for name, value in zip(BARS, values):
            cv2.setTrackbarPos(name, CONTROL, int(value))

    def mouse(self, event, x, y, flags, param):
        if self.displayed is None:
            return
        h, w = self.displayed.shape[:2]
        point = (max(0, min(w - 1, x)), max(0, min(h - 1, y)))
        if event == cv2.EVENT_LBUTTONDOWN:
            self.sample = self.displayed.copy()  # exact clean frame visible on click
            self.start = self.end = point
            self.dragging = True
            self.rect = None
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end = point
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.end = point
            self.dragging = False
            x0, x1 = sorted([self.start[0], point[0]])
            y0, y1 = sorted([self.start[1], point[1]])
            if x1 - x0 < 3 or y1 - y0 < 3:
                print('选框太小，请重新拖动至少 3x3 像素。', flush=True)
                return
            roi = self.sample[y0:y1, x0:x1]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            values = estimate(hsv)
            self.set_values(values)  # replace, never accumulate old samples
            self.rect = (x0, y0, x1, y1)
            self.selection_id += 1
            print('\n新选框 #{}：已覆盖上一组阈值'.format(self.selection_id), flush=True)
            self.print_values(values)
            if values[1] - values[0] > 60:
                print('提示：色相范围较宽，建议只框黄色海绵内部，避免背景。', flush=True)

    @staticmethod
    def print_values(values):
        for name, value in parameters(values).items():
            print('"{}": {},'.format(name, value), flush=True)
        print('LOWER_YELLOW = [{}, {}, {}]'.format(values[0], values[2], values[3]))
        print('UPPER_YELLOW = [{}, 255, 255]'.format(values[1]), flush=True)

    def render(self):
        base = self.sample if self.dragging else self.frame
        self.displayed = base.copy()
        values = self.values()
        hsv = cv2.cvtColor(base, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([values[0], values[2], values[3]], np.uint8),
                           np.array([values[1], 255, 255], np.uint8))
        result = cv2.bitwise_and(base, base, mask=mask)
        view = base.copy()
        if self.dragging:
            cv2.rectangle(view, self.start, self.end, (0, 255, 0), 2)
        elif self.rect:
            x0, y0, x1, y1 = self.rect
            cv2.rectangle(view, (x0, y0), (x1, y1), (0, 255, 0), 2)
        state = 'SELECTING (frozen)' if self.dragging else ('PAUSED' if self.paused else 'LIVE')
        lines = [state + '  FPS {:.1f}  ROI #{}'.format(self.fps, self.selection_id),
                 'H {}..{}   S {}..255   V {}..255'.format(*values),
                 'Drag: replace ROI | SPACE: pause | S: save | R: reset | Q: quit']
        # Text lives outside the camera image so it cannot contaminate ROI sampling.
        footer = np.zeros((78, base.shape[1], 3), np.uint8)
        for i, line in enumerate(lines):
            cv2.putText(footer, line, (6, 20 + i * 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, (0, 255, 255), 1, cv2.LINE_AA)
        # CAMERA contains only the unscaled image; no coordinate conversion needed.
        cv2.imshow(CAMERA, view)
        cv2.imshow(MASK, mask)
        cv2.imshow(RESULT, result)
        cv2.imshow(CONTROL, footer)
        self.current_mask = mask
        self.current_result = result
        if values != self.last_values:
            self.last_values = values[:]

    def save(self):
        folder = Path(self.args.output).expanduser()
        try:
            folder.mkdir(parents=True, exist_ok=True)
            values = self.values()
            payload = {'lower_yellow': [values[0], values[2], values[3]],
                       'upper_yellow': [values[1], 255, 255],
                       'shiwai_parameters': parameters(values),
                       'roi': self.rect, 'selection_id': self.selection_id,
                       'note': 'HSV only; no yellow-black tape filtering'}
            (folder / 'yellow_thresholds.json').write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
            for name, frame in [('camera.png', self.displayed),
                                ('mask.png', self.current_mask),
                                ('result.png', self.current_result)]:
                if not cv2.imwrite(str(folder / name), frame):
                    raise OSError('图片写入失败: ' + name)
            self.print_values(values)
            print('已保存（覆盖上次保存）：' + str(folder.resolve()), flush=True)
        except OSError as error:
            print('保存失败：' + str(error), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--format', choices=['YUYV', 'MJPG'], default='YUYV')
    parser.add_argument('--rotate', type=int, choices=[0, 90, 180, 270], default=0)
    parser.add_argument('--mirror', action='store_true', help='水平镜像')
    parser.add_argument('--output', default=str(Path.home() / 'wheel_robot' / 'yellow_calibration'))
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        parser.error('分辨率和帧率必须大于0')
    if os.name == 'posix' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('没有图形显示环境。请在机器人桌面终端运行，或使用已配置图形转发的连接。')
        return 1
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            print('无法打开摄像头 {}；检查设备路径、权限，并退出其他摄像头程序。'.format(args.device))
            return 1
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.format))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print('摄像头实际配置：{}x{} @ {} FPS'.format(
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            cap.get(cv2.CAP_PROP_FPS)), flush=True)
        app = Calibrator(args)
        print('在 Camera 窗口左键拖框。拖动时暂时定格，松开后恢复实时显示。\n'
              '每次新选框覆盖旧阈值，不合并；框内尽量只包含黄色海绵。\n'
              '可用四个滑条微调，S/V上限固定255，与shiwai.py一致。\n'
              '空格暂停/继续，S保存当前结果，R恢复默认值，Q/Esc退出。', flush=True)
        failures = 0
        while True:
            # Keep draining camera while paused/dragging to avoid stale buffered frames.
            ok, frame = cap.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 30:
                    print('连续读帧失败，退出。检查USB连接或尝试 --format MJPG。')
                    return 1
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord('q'), ord('Q')):
                    break
                continue
            failures = 0
            rotate = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                      270: cv2.ROTATE_90_COUNTERCLOCKWISE}
            if args.rotate:
                frame = cv2.rotate(frame, rotate[args.rotate])
            if args.mirror:
                frame = cv2.flip(frame, 1)
            now = time.monotonic()
            fps = 1.0 / max(0.001, now - app.last_frame_at)
            app.fps = fps if app.fps == 0 else 0.9 * app.fps + 0.1 * fps
            app.last_frame_at = now
            if app.frame is None or (not app.paused and not app.dragging):
                app.frame = frame
            app.render()
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q'), ord('Q')):
                break
            if key == ord(' ') and not app.dragging:
                app.paused = not app.paused
            elif key in (ord('s'), ord('S')) and not app.dragging:
                app.save()
            elif key in (ord('r'), ord('R')):
                app.dragging = False
                app.rect = None
                app.selection_id = 0
                app.set_values(DEFAULT)
                print('已重置默认阈值。', flush=True)
    except KeyboardInterrupt:
        pass
    except cv2.error as error:
        print('OpenCV错误：{}\n请确认安装带GUI的OpenCV，并在图形桌面运行。'.format(error))
        return 1
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
