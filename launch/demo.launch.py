import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    package_name = "dual_fr3_moveit_config"

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = get_package_share_directory(package_name)
    gripper_config = os.path.join(
        get_package_share_directory("franka_gripper"),
        "config",
        "franka_gripper_node.yaml",
    )
    urdf_xacro = os.path.join(package_share, "config", "dual_fr3.urdf.xacro")
    srdf_xacro = os.path.join(package_share, "config", "dual_fr3.srdf.xacro")

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " left_robot_ip:=",
            left_robot_ip,
            " right_robot_ip:=",
            right_robot_ip,
            " load_gripper:=",
            load_gripper,
            " ee_id:=",
            ee_id,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_config, value_type=str)
    }

    robot_description_semantic_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            srdf_xacro,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }

    kinematics_yaml = load_yaml(package_name, "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(package_name, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        package_name, "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "source_list": [
                    "franka/joint_states",
                    "left_franka_gripper/joint_states",
                    "right_franka_gripper/joint_states",
                ],
                "rate": 30,
            },
        ],
    )

    ros2_controllers_path = os.path.join(
        package_share, "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[("joint_states", "franka/joint_states")],
        output="both",
    )

    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_fr3_arm_controller",
        "right_fr3_arm_controller",
    ]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        )

    franka_state_broadcasters = []
    for controller in [
        "left_franka_robot_state_broadcaster",
        "right_franka_robot_state_broadcaster",
    ]:
        franka_state_broadcasters.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
                condition=UnlessCondition(use_fake_hardware),
            )
        )

    left_gripper = Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name="left_franka_gripper",
        output="screen",
        parameters=[
            {
                "robot_ip": left_robot_ip,
                "joint_names": [
                    "left_fr3_finger_joint1",
                    "left_fr3_finger_joint2",
                ],
            },
            gripper_config,
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    right_gripper = Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name="right_franka_gripper",
        output="screen",
        parameters=[
            {
                "robot_ip": right_robot_ip,
                "joint_names": [
                    "right_fr3_finger_joint1",
                    "right_fr3_finger_joint2",
                ],
            },
            gripper_config,
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    left_fake_gripper = Node(
        package=package_name,
        executable="fake_gripper_action_server.py",
        name="left_franka_gripper",
        output="screen",
        prefix="/usr/bin/python3",
        parameters=[
            {
                "joint_names": [
                    "left_fr3_finger_joint1",
                    "left_fr3_finger_joint2",
                ],
            },
            gripper_config,
        ],
        condition=IfCondition(use_fake_hardware),
    )

    right_fake_gripper = Node(
        package=package_name,
        executable="fake_gripper_action_server.py",
        name="right_franka_gripper",
        output="screen",
        prefix="/usr/bin/python3",
        parameters=[
            {
                "joint_names": [
                    "right_fr3_finger_joint1",
                    "right_fr3_finger_joint2",
                ],
            },
            gripper_config,
        ],
        condition=IfCondition(use_fake_hardware),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(package_share, "rviz", "dual_fr3.rviz")],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use ros2_control mock hardware.",
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="Enable mock sensor commands when using fake hardware.",
            ),
            DeclareLaunchArgument(
                "left_robot_ip",
                default_value="",
                description="Left FR3 hostname or IP address for real hardware.",
            ),
            DeclareLaunchArgument(
                "right_robot_ip",
                default_value="",
                description="Right FR3 hostname or IP address for real hardware.",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="true",
                description="Load Franka hand geometry.",
            ),
            DeclareLaunchArgument(
                "ee_id",
                default_value="franka_hand",
                description="End-effector id.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz.",
            ),
            robot_state_publisher,
            joint_state_publisher,
            move_group_node,
            ros2_control_node,
            rviz_node,
            left_gripper,
            right_gripper,
            left_fake_gripper,
            right_fake_gripper,
        ]
        + load_controllers
        + franka_state_broadcasters
    )
