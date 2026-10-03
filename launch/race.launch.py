"""Single-command task-three launch with exclusive camera ownership."""
import importlib.util
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_settings_path = Path(__file__).with_name("race_settings.py")
_settings_spec = importlib.util.spec_from_file_location("wheel_robot_race_settings", _settings_path)
_settings_module = importlib.util.module_from_spec(_settings_spec)
_settings_spec.loader.exec_module(_settings_module)
RACE_SETTINGS = _settings_module.RACE_SETTINGS
LAUNCH_SETTINGS = _settings_module.LAUNCH_SETTINGS


def _bool_text(value):
    return "true" if value else "false"


def generate_launch_description():
    share = FindPackageShare("wheel_robot")
    race_params = PathJoinSubstitution([share, "config", "race_controller.yaml"])
    camera_params = PathJoinSubstitution([share, "config", "astra_camera.yaml"])
    step_params = PathJoinSubstitution([share, "config", "step_approach_test.yaml"])
    motion = LaunchConfiguration("enable_motion")
    step_detector = LaunchConfiguration("start_step_detector")
    screen_debug = LaunchConfiguration("show_debug")
    local_display = LaunchConfiguration("local_display")
    speed_scale = LaunchConfiguration("speed_scale")
    # Astra's vendor SDK is installed beside the executable.  RPATH handles
    # normal installations; this explicit process-local path is a second,
    # harmless guard for deployments that strip RPATH while copying files.
    camera_library_dir = str(Path(get_package_prefix("wheel_robot")) / "lib" / "wheel_robot")
    inherited_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    astra_library_path = camera_library_dir + (
        os.pathsep + inherited_ld_path if inherited_ld_path else ""
    )
    race_node = Node(package="wheel_robot", executable="race_controller.py", name="race_controller", output="screen", emulate_tty=True, additional_env={"DISPLAY": local_display}, parameters=[race_params, RACE_SETTINGS, {"enable_motion": ParameterValue(motion, value_type=bool), "enable_step_jump": ParameterValue(LaunchConfiguration("enable_step_jump"), value_type=bool), "show_debug_window": ParameterValue(screen_debug, value_type=bool), "speed_scale": ParameterValue(speed_scale, value_type=float)}])
    astra_node = Node(package="wheel_robot", executable="astra_camera_node", name="astra_camera", namespace="camera", output="screen", emulate_tty=True, condition=IfCondition(step_detector), additional_env={"LD_LIBRARY_PATH": astra_library_path}, parameters=[camera_params, {"enable_color": False, "enable_depth": True}])
    step_node = Node(package="wheel_robot", executable="step_detector_node.py", name="step_detector", output="screen", emulate_tty=True, condition=IfCondition(step_detector), parameters=[step_params, {"publish_debug_image": False}])
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("enable_motion", default_value=_bool_text(RACE_SETTINGS["enable_motion"]), description="Override enable_motion in race_settings.py"),
        DeclareLaunchArgument("enable_step_jump", default_value=_bool_text(RACE_SETTINGS["enable_step_jump"]), description="Override enable_step_jump in race_settings.py"),
        DeclareLaunchArgument("start_step_detector", default_value=_bool_text(LAUNCH_SETTINGS["start_step_detector"]), description="Depth-only Astra; does not take RGB /dev/video0"),
        DeclareLaunchArgument("show_debug", default_value=_bool_text(RACE_SETTINGS["show_debug_window"]), description="Override show_debug_window in race_settings.py"),
        DeclareLaunchArgument("speed_scale", default_value=str(RACE_SETTINGS["speed_scale"]), description="Override speed_scale in race_settings.py"),
        DeclareLaunchArgument("local_display", default_value=LAUNCH_SETTINGS["local_display"], description="Onboard HDMI X11 display"),
        Node(package="wheeled_legged_pkg", executable="wl_base_node", name="wl_base_node", output="screen", emulate_tty=True, parameters=[{"serial_port": LaunchConfiguration("serial_port")}]),
        # This is the sole owner of RGB /dev/video0 and /cmd_vel.
        race_node,
        # Astra is depth-only, so it can coexist with the RGB follower.
        astra_node,
        # Sensor-only: publishes /step_detector/detection and never /cmd_vel.
        step_node,
        RegisterEventHandler(OnProcessExit(target_action=race_node, on_exit=[Shutdown(reason="race controller stopped")])),
        # A race with a requested depth/step pipeline but no Astra camera
        # cannot safely complete the mandatory step.  End it clearly instead
        # of silently continuing with no detector input.
        RegisterEventHandler(OnProcessExit(target_action=astra_node, on_exit=[Shutdown(reason="Astra depth camera stopped")])),
        RegisterEventHandler(OnProcessExit(target_action=step_node, on_exit=[Shutdown(reason="step detector stopped")])),
    ])
