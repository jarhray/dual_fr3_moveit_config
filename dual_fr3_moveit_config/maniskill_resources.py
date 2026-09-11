"""One final robot model for ManiSkill, MoveIt, TF and scene inspection."""
from dataclasses import replace
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import xacro

from dual_fr3_maniskill.scenes import extend_scene_description, scene_spec
from .moveit_resources import build_moveit_resources


def build_maniskill_description(*, scene="robot", cable_config=None,
                                load_gripper="true", ee_id="franka_hand"):
    scene_spec(scene)
    if scene in ("usb_cable", "trunking_cable") and (str(load_gripper).lower() not in ("true", "1")
                                  or ee_id != "franka_hand"):
        raise ValueError("USB cable scene requires load_gripper:=true and ee_id:=franka_hand")
    share = Path(get_package_share_directory("dual_fr3_moveit_config"))
    mappings = {"load_left_ros2_control": "false", "load_right_ros2_control": "false",
                "load_gripper": str(load_gripper).lower(), "ee_id": ee_id}
    description = xacro.process_file(str(share / "config/dual_fr3.urdf.xacro"), mappings=mappings).toxml()
    semantic = xacro.process_file(str(share / "config/dual_fr3.srdf.xacro")).toxml()
    return extend_scene_description(description, semantic, scene=scene, cable_config=cable_config)


def build_maniskill_resources(**kwargs):
    description, semantic = build_maniskill_description(**kwargs)
    return replace(
        build_moveit_resources("dual_fr3.urdf.xacro", {}),
        robot_description={"robot_description": ParameterValue(description, value_type=str)},
        robot_description_semantic={"robot_description_semantic": ParameterValue(semantic, value_type=str)},
    )
