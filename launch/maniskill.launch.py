"""MoveIt and RViz driven by one ManiSkill physics/action bridge."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dual_fr3_moveit_config.moveit_resources import build_moveit_resources, load_yaml


def generate_launch_description():
    share = get_package_share_directory("dual_fr3_moveit_config")
    sim_share = get_package_share_directory("dual_fr3_maniskill")
    python = os.environ.get("MANISKILL_PYTHON", str(Path.cwd() / ".venv/bin/python"))
    arguments = [
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("ee_id", default_value="franka_hand"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("maniskill_viewer", default_value="true"),
        DeclareLaunchArgument("maniskill_python", default_value=python),
        DeclareLaunchArgument("maniskill_config", default_value=os.path.join(sim_share, "config/simulation.yaml")),
        DeclareLaunchArgument("trajectory_execution_duration_scaling", default_value="10.0"),
        DeclareLaunchArgument("trajectory_execution_goal_margin", default_value="5.0"),
        DeclareLaunchArgument("capabilities", default_value="move_group/ExecuteTaskSolutionCapability"),
    ]
    resources = build_moveit_resources("dual_fr3.urdf.xacro", {
        "load_left_ros2_control": "false", "load_right_ros2_control": "false",
        "load_gripper": LaunchConfiguration("load_gripper"),
        "ee_id": LaunchConfiguration("ee_id"),
    })
    # These are ROS action names, independent of the physics engine.
    controllers = load_yaml("dual_fr3_moveit_config", "config/moveit_controllers_gazebo.yaml")
    bridge = Node(
        package="dual_fr3_maniskill", executable="maniskill_bridge.py",
        prefix=[LaunchConfiguration("maniskill_python")], output="screen",
        parameters=[LaunchConfiguration("maniskill_config"), resources.robot_description,
                    resources.robot_description_semantic,
                    {"viewer": ParameterValue(LaunchConfiguration("maniskill_viewer"), value_type=bool)}],
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[resources.robot_description, {"use_sim_time": True}])
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=resources.as_parameters() + [{
            "use_sim_time": True,
            "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
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
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                arguments=["-d", os.path.join(share, "rviz/dual_fr3.rviz")],
                parameters=resources.as_parameters() + [{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("use_rviz")))
    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=bridge,
        on_exit=[EmitEvent(event=Shutdown(reason="ManiSkill simulation exited"))]))
    return LaunchDescription(arguments + [shutdown, bridge, rsp, move_group, rviz])
