"""Start the safe stand-only posture together with a robot-local camera preview."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    camera_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "astra_camera.yaml"]
    )
    workspace_library_path = PathJoinSubstitution(
        [FindPackagePrefix("wheel_robot"), "lib"]
    )

    camera_node = Node(
        package="wheel_robot",
        executable="astra_camera_node",
        name="astra_camera",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("camera_params_file"),
            {"enable_color": False, "enable_depth": True},
        ],
    )

    depth_distance = Node(
        package="wheel_robot",
        executable="depth_distance_node",
        name="depth_distance",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("camera_params_file")],
        additional_env={
            "LD_LIBRARY_PATH": [
                workspace_library_path,
                TextSubstitution(text=":"),
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ]
        },
    )

    preview = Node(
        package="wheel_robot",
        executable="depth_preview.py",
        name="stand_camera_preview",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("show_camera_preview")),
        parameters=[{"show_display": True}],
        additional_env={
            "DISPLAY": LaunchConfiguration("local_display"),
            "XAUTHORITY": LaunchConfiguration("local_xauthority"),
            "QT_X11_NO_MITSHM": "1",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Type-C serial device connected to the ESP32",
            ),
            DeclareLaunchArgument(
                "stand_height_m",
                default_value="0.15",
                description="Competition posture leg height in metres",
            ),
            DeclareLaunchArgument(
                "stand_roll_deg",
                default_value="0.0",
                description="Competition posture roll in degrees",
            ),
            DeclareLaunchArgument(
                "stand_pitch_deg",
                default_value="28.0",
                description="Competition posture pitching in degrees",
            ),
            DeclareLaunchArgument(
                "stand_slide_m",
                default_value="0.0",
                description="Competition posture slide in metres",
            ),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="Astra camera parameter file",
            ),
            DeclareLaunchArgument(
                "local_display",
                default_value=":0",
                description="Orange Pi local HDMI X11 display",
            ),
            DeclareLaunchArgument(
                "local_xauthority",
                default_value="/home/orangepi/.Xauthority",
                description="Orange Pi local X11 authorization file",
            ),
            DeclareLaunchArgument(
                "show_camera_preview",
                default_value="true",
                description="Show the stand-only depth preview on the robot HDMI screen",
            ),
            Node(
                package="wheeled_legged_pkg",
                executable="wl_base_node",
                name="wl_base_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"serial_port": LaunchConfiguration("serial_port")}
                ],
            ),
            camera_node,
            depth_distance,
            preview,
            Node(
                package="wheel_robot",
                executable="stand_only.py",
                name="wheelleg_stand_only",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "stand_height_m": LaunchConfiguration("stand_height_m"),
                        "stand_roll_deg": LaunchConfiguration("stand_roll_deg"),
                        "stand_pitch_deg": LaunchConfiguration("stand_pitch_deg"),
                        "stand_slide_m": LaunchConfiguration("stand_slide_m"),
                    }
                ],
            ),
        ]
    )
