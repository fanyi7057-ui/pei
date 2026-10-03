"""Safe terminal menu for independent task-3 tests."""
import os, shlex, signal, subprocess
from pathlib import Path


def run(command, motion=False):
    if motion and input("此项会让机器人运动，确认安全后输入 RUN：").strip()!="RUN": return
    print("运行：",shlex.join(command),"\nCtrl-C返回菜单")
    process=subprocess.Popen(command)
    try: process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.terminate()


def main():
    options={
        "1":("黑色单线/环岛/人行道识别，不动车",lambda:run(["ros2","launch","hh","hh_visual_test.launch.py","enable_motion:=false"])),
        "2":("黑色单线三区低速循迹",lambda:run(["ros2","launch","hh","hh_visual_test.launch.py","enable_motion:=true"],True)),
        "3":("台阶深度检测，不动车",lambda:run(["ros2","launch","wheel_robot","race_step_support.launch.py","publish_debug_image:=true"])),
        "4":("台阶接近和带速跳跃",lambda:run(["ros2","launch","hh","hh_step_test.launch.py","start_base:=true","jump_enabled:=true"],True)),
        "5":("底盘键盘测试",lambda:run(["ros2","run","wheel_robot","keyboard_teleop.py"],True)),
        "6":("查看状态机初始状态",lambda:run(["ros2","run","hh","hh_state_machine"])),
    }
    while True:
        print("\n===== hh 任务三独立测试 =====")
        for key,(label,_) in options.items(): print(f"{key}. {label}")
        print("0. 退出")
        choice=input("选择：").strip()
        if choice=="0":
            subprocess.run(["ros2","topic","pub","--once","/cmd_vel","geometry_msgs/msg/Twist","{}"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); return
        if choice in options: options[choice][1]()

