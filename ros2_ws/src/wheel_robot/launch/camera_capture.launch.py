"""Source-launched stationary RGB capture with existing stand-only posture."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    package=Path(__file__).resolve().parents[1]
    bridge=Node(package="wheeled_legged_pkg",executable="wl_base_node",name="wl_base_node",
                parameters=[{"serial_port":"/dev/ttyUSB0"}],output="screen",emulate_tty=True)
    camera=ExecuteProcess(cmd=["python3",str(package/"scripts/camera_capture_diag.py"),"--ros-args",
          "-p","stand_height_m:=0.15","-p","stand_pitch_deg:=25.0"],
          output="screen",emulate_tty=True,
          additional_env={"DISPLAY":":0","XAUTHORITY":"/home/orangepi/.Xauthority","PYTHONUNBUFFERED":"1","CAPTURE_TOPIC_MODE":"1"})
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=camera,on_exit=[EmitEvent(event=Shutdown(reason="capture exited"))])),
        RegisterEventHandler(OnProcessExit(target_action=bridge,on_exit=[EmitEvent(event=Shutdown(reason="bridge exited"))])),
        bridge,camera])
