"""Convenience entry point for the shared ManiSkill USB cable scene."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("maniskill_scene", default_value="usb_cable", choices=("usb_cable",)),
        DeclareLaunchArgument("capabilities", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dual_fr3_moveit_config"), "launch/maniskill.launch.py")),
        ),
    ])
