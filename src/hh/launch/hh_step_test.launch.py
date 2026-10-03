from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wheel=FindPackageShare("wheel_robot")
    params=PathJoinSubstitution([wheel,"config","step_approach_test.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("start_base",default_value="false"),
        DeclareLaunchArgument("serial_port",default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("jump_enabled",default_value="false"),
        Node(package="wheeled_legged_pkg",executable="wl_base_node",name="wl_base_node",
             output="screen",condition=IfCondition(LaunchConfiguration("start_base")),
             parameters=[{"serial_port":LaunchConfiguration("serial_port")}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([wheel,"launch","astra_camera.launch.py"])),launch_arguments={"show_display":"false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([wheel,"launch","step_approach_test.launch.py"])),launch_arguments={"params_file":params,"jump_enabled":LaunchConfiguration("jump_enabled"),"show_display":"true"}.items()),
    ])
