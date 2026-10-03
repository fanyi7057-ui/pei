#!/usr/bin/env python3
"""Send a one-shot, deliberate confirmation to the guarded jump routine."""

import rclpy
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = rclpy.create_node("jump_confirm")
    publisher = node.create_publisher(Bool, "/race/jump_confirm", 10)
    print("小车应已在台阶前 30cm 停车。输入 1 并回车才会允许后退、助跑和跳跃。")
    try:
        while rclpy.ok():
            try:
                answer = input("> ").strip()
            except EOFError:
                break
            if answer != "1":
                print("未发送跳跃指令；请输入 1 确认，或 Ctrl-C 退出。")
                continue
            message = Bool()
            message.data = True
            # Publish a short burst so a subscriber that has just connected
            # still receives the one-shot approval.
            for _ in range(3):
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
            print("已发送跳跃许可。")
            break
    except KeyboardInterrupt:
        print("已取消跳跃许可。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
