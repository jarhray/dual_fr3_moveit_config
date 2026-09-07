from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_PACKAGE = "dual_fr3_moveit_config"
OMPL_REQUEST_ADAPTERS = (
    "default_planner_request_adapters/AddTimeOptimalParameterization "
    "default_planner_request_adapters/ResolveConstraintFrames "
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints"
)


@dataclass(frozen=True)
class MoveItResources:
    """Robot model and planning parameters shared by launch files."""

    robot_description: dict
    robot_description_semantic: dict
    kinematics: dict
    planning_pipeline: dict

    def as_parameters(self) -> list[dict]:
        return [
            self.robot_description,
            self.robot_description_semantic,
            self.kinematics,
            self.planning_pipeline,
        ]


def load_yaml(
    package_name: str,
    file_path: str,
) -> dict:
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def build_robot_description(
    urdf_file: str,
    mappings: Mapping[str, object],
    package_name: str = DEFAULT_PACKAGE,
) -> dict:
    package_path = get_package_share_directory(package_name)
    command: list[object] = [
        FindExecutable(name="xacro"),
        " ",
        os.path.join(package_path, "config", urdf_file),
    ]
    for name, value in mappings.items():
        command.extend([f" {name}:=", value])
    return {
        "robot_description": ParameterValue(
            Command(command),
            value_type=str,
        )
    }


def build_moveit_resources(
    urdf_file: str,
    urdf_mappings: Mapping[str, object],
    package_name: str = DEFAULT_PACKAGE,
    srdf_file: str = "dual_fr3.srdf.xacro",
) -> MoveItResources:
    package_path = get_package_share_directory(package_name)
    robot_description = build_robot_description(
        urdf_file,
        urdf_mappings,
        package_name=package_name,
    )
    semantic_command = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(package_path, "config", srdf_file),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            semantic_command,
            value_type=str,
        )
    }
    planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": OMPL_REQUEST_ADAPTERS,
            "start_state_max_bounds_error": 0.1,
            "path_tolerance": 0.001,
            "resample_dt": 0.02,
        }
    }
    planning_pipeline["move_group"].update(
        load_yaml(package_name, "config/ompl_planning.yaml")
    )
    return MoveItResources(
        robot_description=robot_description,
        robot_description_semantic=robot_description_semantic,
        kinematics=load_yaml(package_name, "config/kinematics.yaml"),
        planning_pipeline=planning_pipeline,
    )
