import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dual_fr3_moveit_config.moveit_resources import (
    build_moveit_resources,
    build_robot_description,
    load_yaml,
)


def generate_launch_description():
    package_name = "dual_fr3_moveit_config"

    trajectory_execution_duration_scaling = LaunchConfiguration(
        "trajectory_execution_duration_scaling"
    )
    trajectory_execution_goal_margin = LaunchConfiguration(
        "trajectory_execution_goal_margin"
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    start_gripper = LaunchConfiguration("start_gripper")
    ee_id = LaunchConfiguration("ee_id")
    use_rviz = LaunchConfiguration("use_rviz")
    capabilities = LaunchConfiguration("capabilities")
    disable_capabilities = LaunchConfiguration("disable_capabilities")

    package_share = get_package_share_directory(package_name)
    gripper_config = os.path.join(
        get_package_share_directory("franka_gripper"),
        "config",
        "franka_gripper_node.yaml",
    )
    common_urdf_mappings = {
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "left_robot_ip": left_robot_ip,
        "right_robot_ip": right_robot_ip,
        "load_gripper": load_gripper,
        "ee_id": ee_id,
    }
    resources = build_moveit_resources(
        "dual_fr3.urdf.xacro",
        {
            **common_urdf_mappings,
            "load_left_ros2_control": "true",
            "load_right_ros2_control": "true",
        },
    )
    robot_description = resources.robot_description
    robot_description_semantic = resources.robot_description_semantic
    kinematics_yaml = resources.kinematics
    ompl_planning_pipeline_config = resources.planning_pipeline
    left_hardware_description = build_robot_description(
        "dual_fr3.urdf.xacro",
        {
            **common_urdf_mappings,
            "load_left_ros2_control": "true",
            "load_right_ros2_control": "false",
        },
    )
    right_hardware_description = build_robot_description(
        "dual_fr3.urdf.xacro",
        {
            **common_urdf_mappings,
            "load_left_ros2_control": "false",
            "load_right_ros2_control": "true",
        },
    )

    moveit_simple_controllers_yaml = load_yaml(
        package_name, "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": (
            trajectory_execution_duration_scaling
        ),
        "trajectory_execution.allowed_goal_duration_margin": trajectory_execution_goal_margin,
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
            {
                "capabilities": ParameterValue(capabilities, value_type=str),
                "disable_capabilities": ParameterValue(
                    disable_capabilities, value_type=str
                ),
            },
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
                    "/left/franka/joint_states",
                    "/right/franka/joint_states",
                    "left_franka_gripper/joint_states",
                    "right_franka_gripper/joint_states",
                ],
                "rate": 30,
                "use_robot_description": False,
            },
        ],
    )

    ros2_controllers_path = os.path.join(
        package_share, "config", "ros2_controllers.yaml"
    )
    left_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="left",
        parameters=[left_hardware_description, ros2_controllers_path],
        remappings=[("joint_states", "franka/joint_states")],
        output="both",
        on_exit=Shutdown(reason="Left controller manager exited."),
    )
    right_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="right",
        parameters=[right_hardware_description, ros2_controllers_path],
        remappings=[("joint_states", "franka/joint_states")],
        output="both",
        on_exit=Shutdown(reason="Right controller manager exited."),
    )

    load_controllers = []
    for namespace, controller in [
        ("left", "joint_state_broadcaster"),
        ("left", "left_fr3_arm_controller"),
        ("right", "joint_state_broadcaster"),
        ("right", "right_fr3_arm_controller"),
    ]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    controller,
                    "--controller-manager",
                    f"/{namespace}/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        )

    franka_state_broadcasters = []
    for namespace, controller in [
        ("left", "left_franka_robot_state_broadcaster"),
        ("right", "right_franka_robot_state_broadcaster"),
    ]:
        franka_state_broadcasters.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    controller,
                    "--controller-manager",
                    f"/{namespace}/controller_manager",
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
        condition=IfCondition(
            AndSubstitution(start_gripper, NotSubstitution(use_fake_hardware))
        ),
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
        condition=IfCondition(
            AndSubstitution(start_gripper, NotSubstitution(use_fake_hardware))
        ),
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
        condition=IfCondition(AndSubstitution(start_gripper, use_fake_hardware)),
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
        condition=IfCondition(AndSubstitution(start_gripper, use_fake_hardware)),
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
                "start_gripper",
                default_value="true",
                description="Start the Franka gripper drivers/action servers.",
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
            DeclareLaunchArgument(
                "trajectory_execution_duration_scaling",
                default_value="1.2",
                description="Allowed execution duration scaling for move_group.",
            ),
            DeclareLaunchArgument(
                "trajectory_execution_goal_margin",
                default_value="0.5",
                description="Allowed goal duration margin for move_group.",
            ),
            DeclareLaunchArgument(
                "capabilities",
                default_value="",
                description="Additional MoveGroup capabilities.",
            ),
            DeclareLaunchArgument(
                "disable_capabilities",
                default_value="",
                description="Disabled MoveGroup capabilities.",
            ),
            robot_state_publisher,
            joint_state_publisher,
            move_group_node,
            left_ros2_control_node,
            right_ros2_control_node,
            rviz_node,
            left_gripper,
            right_gripper,
            left_fake_gripper,
            right_fake_gripper,
        ]
        + load_controllers
        + franka_state_broadcasters
    )
