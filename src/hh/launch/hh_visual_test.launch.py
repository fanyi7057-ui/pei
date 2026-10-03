from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params=PathJoinSubstitution([FindPackageShare("hh"),"config","hh_test.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("enable_motion",default_value="false"),
        Node(package="hh",executable="hh_visual",name="hh_visual",output="screen",
             parameters=[params,{"enable_motion":ParameterValue(LaunchConfiguration("enable_motion"),value_type=bool)}])])
