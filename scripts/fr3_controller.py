#!/usr/bin/env python3
"""
MoveIt2 action client controller for the dual FR3 MoveIt scene.

The public API mirrors the single-arm helpers in src/my_controller, with an
extra arm argument: "left" or "right". Targets are expressed in the left arm
base frame by default. Use frame_id="world" for table coordinates or
frame_id="<arm>_fr3_link0" for arm-local commands.
"""

import time
from dataclasses import dataclass
from math import cos, pi, sin
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp, Homing, Move
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


@dataclass(frozen=True)
class ArmConfig:
    name: str
    prefix: str
    planning_group: str
    base_link: str
    tcp_link: str
    hand_link: str
    gripper_node: str
    mount_xy: np.ndarray


class DualFR3Controller(Node):
    """Convenience controller for two FR3 arms in dual_fr3_moveit_config."""

    ARM_CONFIGS: Dict[str, ArmConfig] = {
        "left": ArmConfig(
            name="left",
            prefix="left_fr3_",
            planning_group="left_fr3_arm",
            base_link="left_fr3_link0",
            tcp_link="left_fr3_hand_tcp",
            hand_link="left_fr3_hand",
            gripper_node="left_franka_gripper",
            mount_xy=np.array([0.175, 0.35]),
        ),
        "right": ArmConfig(
            name="right",
            prefix="right_fr3_",
            planning_group="right_fr3_arm",
            base_link="right_fr3_link0",
            tcp_link="right_fr3_hand_tcp",
            hand_link="right_fr3_hand",
            gripper_node="right_franka_gripper",
            mount_xy=np.array([0.175, 0.95]),
        ),
    }

    READY_JOINTS = [0.0, -pi / 4.0, 0.0, -3.0 * pi / 4.0, 0.0, pi / 2.0, pi / 4.0]

    def __init__(
        self,
        node_name: str = "dual_fr3_action_controller",
        default_frame: str = "left_fr3_link0",
    ):
        super().__init__(node_name)

        self.default_frame = default_frame
        self.current_joint_state: Optional[JointState] = None
        self.current_gripper_joint_states: Dict[str, Optional[JointState]] = {
            "left": None,
            "right": None,
        }
        self.joint7_home_angle: Dict[str, Optional[float]] = {"left": None, "right": None}
        self.gripper_is_initialized: Dict[str, bool] = {"left": False, "right": False}

        self.callback_group = ReentrantCallbackGroup()

        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self.callback_group
        )
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.callback_group
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.gripper_move_clients = {}
        self.gripper_grasp_clients = {}
        self.gripper_homing_clients = {}
        self.gripper_command_clients = {}
        for arm, cfg in self.ARM_CONFIGS.items():
            self.gripper_move_clients[arm] = ActionClient(
                self, Move, f"/{cfg.gripper_node}/move", callback_group=self.callback_group
            )
            self.gripper_grasp_clients[arm] = ActionClient(
                self, Grasp, f"/{cfg.gripper_node}/grasp", callback_group=self.callback_group
            )
            self.gripper_homing_clients[arm] = ActionClient(
                self, Homing, f"/{cfg.gripper_node}/homing", callback_group=self.callback_group
            )
            self.gripper_command_clients[arm] = ActionClient(
                self,
                GripperCommand,
                f"/{cfg.gripper_node}/gripper_action",
                callback_group=self.callback_group,
            )
            self.create_subscription(
                JointState,
                f"/{cfg.gripper_node}/joint_states",
                lambda msg, arm_name=arm: self.gripper_joint_state_callback(arm_name, msg),
                10,
                callback_group=self.callback_group,
            )

        self.get_logger().info("Waiting for MoveIt2 action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup action server not available")
        self.get_logger().info("MoveGroup action server connected")

        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("IK service not available - IK helpers may be limited")

        self._report_gripper_servers()
        self.get_logger().info("Dual FR3 controller initialized")

    def _arm(self, arm: str) -> ArmConfig:
        if arm not in self.ARM_CONFIGS:
            raise ValueError(f"Unknown arm '{arm}', expected one of: left, right")
        return self.ARM_CONFIGS[arm]

    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def gripper_joint_state_callback(self, arm: str, msg: JointState):
        self.current_gripper_joint_states[arm] = msg

    def _report_gripper_servers(self):
        for arm in ("left", "right"):
            real_count = 0
            if self.gripper_move_clients[arm].wait_for_server(timeout_sec=0.5):
                real_count += 1
            if self.gripper_homing_clients[arm].wait_for_server(timeout_sec=0.5):
                real_count += 1
            command_ok = self.gripper_command_clients[arm].wait_for_server(timeout_sec=0.5)
            if real_count:
                self.get_logger().info(f"{arm} gripper real action servers connected")
            elif command_ok:
                self.get_logger().info(f"{arm} gripper command action server connected")
            else:
                self.get_logger().warn(f"{arm} gripper action servers are not available")

    def _wait_for_joint_state(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while self.current_joint_state is None and time.time() - start < timeout:
            self.get_logger().info("Waiting for /joint_states...")
            time.sleep(0.1)
        return self.current_joint_state is not None

    def _joint_names(self, arm: str) -> List[str]:
        cfg = self._arm(arm)
        return [f"{cfg.prefix}joint{i}" for i in range(1, 8)]

    def _create_pose_target(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = pi,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> Pose:
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        pose.orientation = Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )
        return pose

    def _create_pose_constraints(
        self,
        arm: str,
        target_pose: Pose,
        frame_id: str,
        position_tolerance: float = 0.01,
        orientation_tolerance: float = 0.02,
    ) -> Constraints:
        cfg = self._arm(arm)
        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = cfg.tcp_link
        pos_constraint.constraint_region.primitives = [SolidPrimitive()]
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        pos_constraint.constraint_region.primitives[0].dimensions = [
            position_tolerance,
            position_tolerance,
            position_tolerance,
        ]
        pos_constraint.constraint_region.primitive_poses = [target_pose]
        pos_constraint.weight = 1.0

        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = frame_id
        orient_constraint.link_name = cfg.tcp_link
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orient_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orient_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orient_constraint.weight = 1.0

        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [orient_constraint]
        return constraints

    def _create_motion_plan_request(
        self,
        group_name: str,
        goal_constraints: Sequence[Constraints],
        frame_id: str,
        planner_id: str = "PTP",
        velocity_scale: float = 0.1,
        acceleration_scale: float = 0.1,
        planning_time: float = 10.0,
    ) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = group_name
        request.planner_id = planner_id
        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = frame_id
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -2.0
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0
        if self.current_joint_state:
            request.start_state.joint_state = self.current_joint_state
        request.goal_constraints = list(goal_constraints)
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = acceleration_scale
        return request

    def _send_move_group_request(self, request: MotionPlanRequest, execute: bool) -> bool:
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = not execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error("MoveGroup did not return a result")
            return False

        error_code = result.result.error_code.val
        if error_code == MoveItErrorCodes.SUCCESS:
            return True
        self.get_logger().warn(f"MoveIt failed with error code {error_code}")
        return False

    def _get_workspace_bounds(self):
        return {
            "r_min": 0.20,
            "r_max": 0.90,
            "z_min": 0.01,
            "z_max": 0.85,
        }

    def _arm_origin_xy_in_frame(self, arm: str, frame_id: str):
        cfg = self._arm(arm)
        if frame_id == "world":
            return cfg.mount_xy
        if frame_id in (self.ARM_CONFIGS["left"].base_link, self.ARM_CONFIGS["right"].base_link):
            frame_arm = "left" if frame_id == self.ARM_CONFIGS["left"].base_link else "right"
            return cfg.mount_xy - self.ARM_CONFIGS[frame_arm].mount_xy
        return None

    def _is_pose_reachable(self, arm: str, x: float, y: float, z: float, frame_id: str) -> bool:
        bounds = self._get_workspace_bounds()
        origin = self._arm_origin_xy_in_frame(arm, frame_id)
        if origin is None:
            return True
        dx = x - origin[0]
        dy = y - origin[1]
        r = float((dx**2 + dy**2) ** 0.5)
        return bounds["r_min"] <= r <= bounds["r_max"] and bounds["z_min"] <= z <= bounds["z_max"]

    def move_to(
        self,
        arm: str,
        x: float,
        y: float,
        z: float,
        roll: float = pi,
        pitch: float = 0.0,
        yaw: float = 0.0,
        execute: bool = True,
        frame_id: Optional[str] = None,
    ) -> bool:
        cfg = self._arm(arm)
        frame_id = frame_id or self.default_frame

        if not self._is_pose_reachable(arm, x, y, z, frame_id):
            self.get_logger().warn(f"{arm} target appears outside the nominal FR3 workspace")

        if not self._wait_for_joint_state():
            self.get_logger().error("Timeout waiting for joint state")
            return False

        target_pose = self._create_pose_target(x, y, z, roll, pitch, yaw)
        constraints = self._create_pose_constraints(arm, target_pose, frame_id)
        request = self._create_motion_plan_request(
            cfg.planning_group, [constraints], frame_id=frame_id, planner_id="PTP"
        )

        self.get_logger().info(
            f"Planning {arm} arm to [{x:.3f}, {y:.3f}, {z:.3f}] in {frame_id}"
        )
        return self._send_move_group_request(request, execute)

    def move_left_to(self, *args, **kwargs) -> bool:
        return self.move_to("left", *args, **kwargs)

    def move_right_to(self, *args, **kwargs) -> bool:
        return self.move_to("right", *args, **kwargs)

    def move_dual_to(
        self,
        left_pose: Sequence[float],
        right_pose: Sequence[float],
        execute: bool = True,
        frame_id: Optional[str] = None,
    ) -> bool:
        frame_id = frame_id or self.default_frame
        if not self._wait_for_joint_state():
            self.get_logger().error("Timeout waiting for joint state")
            return False

        left_target = self._create_pose_target(*left_pose)
        right_target = self._create_pose_target(*right_pose)
        constraints = Constraints()
        left_constraints = self._create_pose_constraints("left", left_target, frame_id)
        right_constraints = self._create_pose_constraints("right", right_target, frame_id)
        constraints.position_constraints = (
            left_constraints.position_constraints + right_constraints.position_constraints
        )
        constraints.orientation_constraints = (
            left_constraints.orientation_constraints
            + right_constraints.orientation_constraints
        )

        request = self._create_motion_plan_request(
            "dual_fr3_arms", [constraints], frame_id=frame_id, planner_id="PTP"
        )
        self.get_logger().info("Planning coordinated dual-arm motion")
        return self._send_move_group_request(request, execute)

    def _send_action_goal(self, client: ActionClient, goal, timeout: float = 5.0) -> bool:
        if not client.wait_for_server(timeout_sec=timeout):
            return False
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            return False
        action_result = result.result
        if hasattr(action_result, "success"):
            return bool(action_result.success)
        if hasattr(action_result, "reached_goal"):
            return bool(action_result.reached_goal)
        return True

    def reset_gripper(self, arm: str) -> bool:
        self._arm(arm)
        goal = Homing.Goal()
        if self._send_action_goal(self.gripper_homing_clients[arm], goal, timeout=2.0):
            self.gripper_is_initialized[arm] = True
            return True
        self.get_logger().warn(f"{arm} homing action unavailable, opening via gripper_action")
        return self.open_gripper(arm, width=0.08)

    def open_gripper(self, arm: str, width: float = 0.08, speed: float = 0.08) -> bool:
        self._arm(arm)
        width = float(np.clip(width, 0.0, 0.08))

        move_goal = Move.Goal()
        move_goal.width = width
        move_goal.speed = speed
        if self._send_action_goal(self.gripper_move_clients[arm], move_goal, timeout=1.0):
            return True

        command_goal = GripperCommand.Goal()
        command_goal.command.position = width / 2.0
        command_goal.command.max_effort = 20.0
        return self._send_action_goal(self.gripper_command_clients[arm], command_goal, timeout=2.0)

    def close_gripper(self, arm: str, width: float = 0.0, speed: float = 0.08) -> bool:
        return self.open_gripper(arm, width=width, speed=speed)

    def set_gripper_action(self, arm: str, grip_action: float) -> bool:
        grip_action = float(np.clip(grip_action, 0.0, 1.0))
        return self.open_gripper(arm, width=0.08 * grip_action)

    def grasp_object(
        self,
        arm: str,
        width: float = 0.02,
        speed: float = 0.05,
        force: float = 50.0,
        inner_tolerance: float = 0.005,
        outer_tolerance: float = 0.01,
    ) -> bool:
        self._arm(arm)
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = inner_tolerance
        goal.epsilon.outer = outer_tolerance
        if self._send_action_goal(self.gripper_grasp_clients[arm], goal, timeout=1.0):
            return True
        self.get_logger().warn(f"{arm} grasp action unavailable, closing via gripper_action")
        command_goal = GripperCommand.Goal()
        command_goal.command.position = float(np.clip(width / 2.0, 0.0, 0.04))
        command_goal.command.max_effort = force
        return self._send_action_goal(self.gripper_command_clients[arm], command_goal, timeout=2.0)

    def get_gripper_state(self, arm: str) -> dict:
        self._arm(arm)
        msg = self.current_gripper_joint_states[arm]
        if msg is None:
            return {"available": False, "width": None, "position": None, "joint_names": None}
        width = 2.0 * msg.position[0] if len(msg.position) >= 1 else None
        return {
            "available": True,
            "width": width,
            "position": list(msg.position),
            "joint_names": list(msg.name),
        }

    def print_gripper_state(self, arm: str):
        state = self.get_gripper_state(arm)
        if not state["available"]:
            self.get_logger().info(f"{arm} gripper state not available")
            return
        self.get_logger().info(f"{arm} gripper width: {state['width']:.4f} m")
        self.get_logger().info(f"{arm} gripper joints: {state['joint_names']}")
        self.get_logger().info(f"{arm} gripper positions: {state['position']}")

    def reset_arm(self, arm: str, execute: bool = True) -> bool:
        cfg = self._arm(arm)
        if not self._wait_for_joint_state():
            return False
        constraints = self._joint_constraints(self._joint_names(arm), self.READY_JOINTS)
        request = self._create_motion_plan_request(
            cfg.planning_group, [constraints], frame_id=cfg.base_link, planner_id="PTP"
        )
        ok = self._send_move_group_request(request, execute)
        if ok and self.current_joint_state:
            name = f"{cfg.prefix}joint7"
            if name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(name)
                self.joint7_home_angle[arm] = self.current_joint_state.position[idx]
        return ok

    def reset(self, execute: bool = True, reset_grippers: bool = True) -> bool:
        arm_ok = self.reset_both_arms(execute=execute)
        grip_ok = True
        if reset_grippers:
            grip_ok = self.reset_gripper("left") and self.reset_gripper("right")
        return arm_ok and grip_ok

    def reset_both_arms(self, execute: bool = True) -> bool:
        if not self._wait_for_joint_state():
            return False
        names = self._joint_names("left") + self._joint_names("right")
        values = self.READY_JOINTS + self.READY_JOINTS
        constraints = self._joint_constraints(names, values)
        request = self._create_motion_plan_request(
            "dual_fr3_arms", [constraints], frame_id=self.default_frame, planner_id="PTP"
        )
        return self._send_move_group_request(request, execute)

    def _joint_constraints(
        self,
        names: Iterable[str],
        values: Iterable[float],
        tolerance: float = 0.002,
    ) -> Constraints:
        constraints = Constraints()
        for name, value in zip(names, values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(value)
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    def rotate(
        self,
        arm: str,
        z: float,
        execute: bool = True,
        relative: bool = True,
        abs_tol_fixed: float = 0.002,
        abs_tol_j7: float = 0.002,
    ) -> bool:
        cfg = self._arm(arm)
        if not self._wait_for_joint_state():
            return False
        start_js = self.current_joint_state
        joint7 = f"{cfg.prefix}joint7"
        if joint7 not in start_js.name:
            self.get_logger().error(f"{joint7} not found in /joint_states")
            return False

        j7_idx = start_js.name.index(joint7)
        current_j7 = start_js.position[j7_idx]
        target_j7 = current_j7 + z if relative else z
        target_j7 = float(np.clip(target_j7, -np.deg2rad(170), np.deg2rad(170)))

        constraints = Constraints()
        for name in self._joint_names(arm):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            if name == joint7:
                joint_constraint.position = target_j7
                tol = abs_tol_j7
            else:
                joint_constraint.position = start_js.position[start_js.name.index(name)]
                tol = abs_tol_fixed
            joint_constraint.tolerance_above = tol
            joint_constraint.tolerance_below = tol
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request = self._create_motion_plan_request(
            cfg.planning_group,
            [constraints],
            frame_id=cfg.base_link,
            planner_id="PTP",
            velocity_scale=0.2,
            acceleration_scale=0.2,
            planning_time=5.0,
        )
        return self._send_move_group_request(request, execute)

    def rotate_to_home(self, arm: str, execute: bool = True) -> bool:
        self._arm(arm)
        if self.joint7_home_angle[arm] is None:
            self.get_logger().error(f"{arm} joint7 home angle is not set; call reset_arm() first")
            return False
        return self.rotate(arm, self.joint7_home_angle[arm], execute=execute, relative=False)

    def move_to_named_target(self, arm: str, target_name: str, execute: bool = True) -> bool:
        cfg = self._arm(arm)
        local_named_poses = {
            "home": [0.4, 0.0, 0.5, pi, 0.0, 0.0],
            "look_down": [0.4, 0.0, 0.4, pi, 0.0, 0.0],
            "test1": [0.5, 0.0, 0.4, pi, 0.0, 0.0],
            "test2": [0.6, 0.0, 0.2, pi, 0.0, 0.0],
            "test3": [0.4, 0.3, 0.4, pi, 0.0, 0.0],
            "test4": [0.4, -0.3, 0.5, pi, 0.0, 0.0],
        }
        if target_name not in local_named_poses:
            self.get_logger().error(f"Unknown named target: {target_name}")
            return False
        pose = local_named_poses[target_name]
        return self.move_to(arm, *pose, execute=execute, frame_id=cfg.base_link)


def demo():
    rclpy.init()
    executor = MultiThreadedExecutor()
    controller = None
    try:
        controller = DualFR3Controller()
        executor.add_node(controller)

        import threading

        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        controller.reset_both_arms(execute=False)
        controller.close_gripper("left")
        controller.open_gripper("left")

        controller.close_gripper("right")

        controller.move_left_to(0.4, 0.0, 0.5, execute=True)
        print("Dual FR3 controller demo completed")
    finally:
        if controller is not None:
            controller.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    demo()
