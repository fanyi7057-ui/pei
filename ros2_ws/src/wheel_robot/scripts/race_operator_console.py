#!/usr/bin/env python3
"""Quiet operator console for the two-lap race.

All ROS launch output is redirected to a file by robot_menu.sh. This foreground
console only shows the few operator actions that are intentionally manual.
"""

import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def clear() -> None:
    print("\033[2J\033[H", end="", flush=True)


def read_operator_commands(commands: queue.Queue) -> None:
    """Use blocking terminal input in its own thread.

    SSH terminals can fail to report stdin readiness through select() while a
    background log follower is writing status. input() remains reliable there.
    The ROS thread consumes the queue, so ROS publishers stay single-threaded.
    """
    while True:
        try:
            commands.put(input().strip())
        except EOFError:
            return


class RaceOperatorConsole(Node):
    def __init__(self) -> None:
        super().__init__("race_operator_console")
        self.last_message = ""
        self.done = False
        self.step_ready = False
        self.full_race_auto = True
        self.start_requested = False
        self._next_start_publish = 0.0
        self.start_pub = self.create_publisher(Bool, "/race_start_confirm", 10)
        self.step_pub = self.create_publisher(Bool, "/step/execute_jump", 10)
        self.create_subscription(String, "/race_operator/status", self._on_status, 10)

    @staticmethod
    def _publish_true(publisher) -> None:
        message = Bool()
        message.data = True
        publisher.publish(message)

    def request_start(self) -> None:
        self.start_requested = True
        self._next_start_publish = 0.0
        print("已发送开始确认，正在等待小车接收。", flush=True)

    def retry_start_if_needed(self) -> None:
        if not self.start_requested:
            return
        now = time.monotonic()
        if now < self._next_start_publish:
            return
        self._publish_true(self.start_pub)
        self._next_start_publish = now + 0.25

    def request_step_execute(self) -> None:
        if not self.step_ready:
            return
            return
        self._publish_true(self.step_pub)
        self.step_ready = False
        print("已发送台阶执行确认。", flush=True)

    def request_full_race(self) -> None:
        self.full_race_auto = True
        print("全流程模式已确认：开始后自动处理台阶，急弯、环岛等保持比赛状态机自主运行。", flush=True)
        self.request_start()

    def handle_command(self, command: str) -> None:
        if command == "1":
            self.request_full_race()
        elif command == "0":
            self.done = True
        elif command:
            print("无效输入：1 启动完整比赛，0 结束。", flush=True)

    def _on_status(self, msg: String) -> None:
        text = str(msg.data)
        if not text or text == self.last_message:
            return
        self.last_message = text
        stage, _, payload = text.partition("|")

        if stage == "RACE_START_ACCEPTED":
            self.start_requested = False
            clear()
            print("比赛开始确认已收到。\n")
            print("完整比赛运行中：巡线、跳台、急弯和环岛均自动执行。")
            return

        if stage == "RACE_RUNNING":
            clear()
            print("比赛运行中。")
            print("当前无需操作；台阶确认后会自动执行跳跃。")
            return

        if stage == "LAP_START":
            clear()
            print("下一圈已开始。")
            print("本圈继续自动识别并执行全部任务。")
            return

        if stage == "STEP_WAIT":
            self.step_ready = True
            clear()
            print("=" * 68)
            print("台阶条件已满足，小车已停车")
            print("=" * 68)
            print(payload.replace(",", "   "))
            print("\n完整比赛模式：自动执行起步加速 → 接近 → 起跳 → 落地 → 找线。")
            print("=" * 68)
            print("台阶已确认，自动执行跳跃。", flush=True)
            self.request_step_execute()
            return

        if stage == "STEP_EXECUTING":
            self.step_ready = False
            clear()
            print("已收到台阶执行话题。")
            print("正在自动完成整套跳跃动作，中途无需再操作。")
            return

        if stage == "STEP_DONE":
            clear()
            print("台阶动作完成，已重新看到黑线并恢复正常巡线。")
            print("环岛识别仍处于 4 秒冷却；冷却结束后自动恢复环岛判断。")
            return

        if stage == "RACE_DONE":
            clear()
            print("两圈比赛完成，车辆已停车。")
            self.done = True


def main() -> int:
    rclpy.init()
    node = RaceOperatorConsole()
    clear()
    print("=" * 68)
    print("两轮轮足机器人静默比赛操作台")
    print("=" * 68)
    detail_log = os.environ.get("RACE_DETAIL_LOG", "")
    if detail_log:
        print(f"ROS 详细日志已写入：{detail_log}")
    print("本终端只显示必要操作和每 2.5 秒状态。")
    print("\n1. 启动完整比赛（自动巡线、跳台、急弯、环岛和停车）")
    print("0. 结束本次比赛")
    print("\n输入 1 后全程自动运行；Ctrl+C 也可结束。")
    print("=" * 68)
    commands: queue.Queue = queue.Queue()
    threading.Thread(
        target=read_operator_commands, args=(commands,), daemon=True
    ).start()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.10)
            node.retry_start_if_needed()
            while True:
                try:
                    node.handle_command(commands.get_nowait())
                except queue.Empty:
                    break
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
