"""MoveIt and RViz driven by one ManiSkill physics/action bridge."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dual_fr3_maniskill.launch_support import create_bridge_node
from dual_fr3_maniskill.cable.backends import CABLE_SOLVERS
from dual_fr3_maniskill.scenes import SCENES, resolve_cable_config, scene_spec
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_resources
from dual_fr3_moveit_config.moveit_resources import load_yaml


def launch_setup(context):
    share = get_package_share_directory("dual_fr3_moveit_config")
    scene = LaunchConfiguration("maniskill_scene").perform(context)
    cable_config = (resolve_cable_config(LaunchConfiguration("cable_config").perform(context), scene=scene)
                    if scene in ("usb_cable", "trunking_cable") else "")
    resources = build_maniskill_resources(
        scene=scene, cable_config=cable_config,
        load_gripper=LaunchConfiguration("load_gripper").perform(context),
        ee_id=LaunchConfiguration("ee_id").perform(context),
    )
    # These are ROS action names, independent of the physics engine.
    controllers = load_yaml("dual_fr3_moveit_config", "config/moveit_controllers_gazebo.yaml")
    bridge = create_bridge_node(
        scene=scene, robot_description=resources.robot_description,
        robot_description_semantic=resources.robot_description_semantic,
        python=LaunchConfiguration("maniskill_python"), viewer=LaunchConfiguration("maniskill_viewer"),
        simulation_config=LaunchConfiguration("maniskill_config").perform(context),
        cable_solver=LaunchConfiguration("cable_solver").perform(context),
        load_cable=LaunchConfiguration("load_cable"),
        cable_config=cable_config,
        cable_trace_dir=LaunchConfiguration("cable_trace_dir").perform(context),
        leader_orientation_direction=LaunchConfiguration("leader_orientation_direction").perform(context),
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[resources.robot_description, {"use_sim_time": True}])
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=resources.as_parameters() + [{
            "use_sim_time": True,
            "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
            "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": controllers,
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": ParameterValue(
                LaunchConfiguration("trajectory_execution_duration_scaling"), value_type=float),
            "trajectory_execution.allowed_goal_duration_margin": ParameterValue(
                LaunchConfiguration("trajectory_execution_goal_margin"), value_type=float),
            "trajectory_execution.allowed_start_tolerance": 0.01,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }],
    )
    rviz_config = (LaunchConfiguration("rviz_config").perform(context)
                   or scene_spec(scene).rviz_config or os.path.join(share, "rviz/dual_fr3.rviz"))
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                arguments=["-d", rviz_config],
                parameters=resources.as_parameters() + [{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("use_rviz")))
    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=bridge,
        on_exit=[EmitEvent(event=Shutdown(reason="ManiSkill simulation exited"))]))
    actions = [shutdown, bridge, rsp, move_group, rviz]
    if scene == "usb_cable":
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="USB cable RViz closed"))])))
    return actions


def generate_launch_description():
    python = os.environ.get("MANISKILL_PYTHON", str(Path.cwd() / ".venv/bin/python"))
    return LaunchDescription([
        DeclareLaunchArgument("maniskill_scene", default_value="robot", choices=SCENES),
        DeclareLaunchArgument("cable_solver", default_value="mpm", choices=CABLE_SOLVERS),
        DeclareLaunchArgument("load_cable", default_value="true", choices=("true", "false"),
                              description="false: USB contact-grasp debugging without a cable backend."),
        DeclareLaunchArgument("leader_orientation_direction", default_value="reverse",
                              choices=("forward", "reverse")),
        DeclareLaunchArgument("cable_trace_dir", default_value="",
                              description="Optional Rope-Actor diagnostic output directory; empty disables recording."),
        DeclareLaunchArgument("cable_config", default_value="",
                              description="USB/cable material and initial-layout YAML; empty uses the scene default."),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("ee_id", default_value="franka_hand"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        DeclareLaunchArgument("maniskill_viewer", default_value="true"),
        DeclareLaunchArgument("maniskill_python", default_value=python),
        DeclareLaunchArgument("maniskill_config", default_value="",
                              description="ROS bridge parameter YAML; empty uses the selected scene's defaults."),
        DeclareLaunchArgument("trajectory_execution_duration_scaling", default_value="10.0"),
        DeclareLaunchArgument("trajectory_execution_goal_margin", default_value="5.0"),
        DeclareLaunchArgument("capabilities", default_value="move_group/ExecuteTaskSolutionCapability"),
        DeclareLaunchArgument("disable_capabilities", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
