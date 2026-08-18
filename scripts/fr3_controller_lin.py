#!/usr/bin/env python3
"""
Cartesian linear controller for the dual FR3 MoveIt scene.

This file extends fr3_controller.py with /compute_cartesian_path and
/execute_trajectory support. The API keeps the same arm argument convention:
arm="left" or arm="right".
"""

import time
from math import asin, atan2, copysign, pi
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.transform_listener import TransformListener

from fr3_controller import DualFR3Controller


def quat_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = copysign(pi / 2.0, sinp)
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class DualFR3LinearController(DualFR3Controller):
    """Dual-arm controller with Cartesian path support."""

    def __init__(
        self,
        node_name: str = "dual_fr3_linear_controller",
        default_frame: str = "left_fr3_link0",
        use_cartesian_lin_control: bool = True,
    ):
        super().__init__(node_name=node_name, default_frame=default_frame)

        self.use_cartesian_lin_control = use_cartesian_lin_control
        self.current_pose = {"left": None, "right": None}
        self.fail_count = {"left": 0, "right": 0}

        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
            callback_group=self.callback_group,
        )
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
            callback_group=self.callback_group,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pose_timer = self.create_timer(
            0.02,
            self._update_eef_poses,
            callback_group=ReentrantCallbackGroup(),
        )

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("ExecuteTrajectory action server not available")
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Cartesian path service not available")

    def _update_eef_poses(self):
        for arm, cfg in self.ARM_CONFIGS.items():
            try:
                if not self.tf_buffer.can_transform(
                    self.default_frame,
                    cfg.tcp_link,
                    Time(),
                    timeout=Duration(seconds=0.01),
                ):
                    continue
                trans = self.tf_buffer.lookup_transform(
                    self.default_frame,
                    cfg.tcp_link,
                    Time(),
                    timeout=Duration(seconds=0.01),
                )
                pose = Pose()
                pose.position.x = trans.transform.translation.x
                pose.position.y = trans.transform.translation.y
                pose.position.z = trans.transform.translation.z
                pose.orientation = trans.transform.rotation
                self.current_pose[arm] = pose
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

    def get_current_pose(self, arm: str):
        self._arm(arm)
        return self.current_pose[arm]

    def get_current_pose_rpy(self, arm: str):
        pose = self.get_current_pose(arm)
        if pose is None:
            return None
        roll, pitch, yaw = quat_to_rpy(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            roll,
            pitch,
            yaw,
        ]

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
        if self.use_cartesian_lin_control:
            return self.move_to_cartesian(
                arm, x, y, z, roll, pitch, yaw, execute=execute, frame_id=frame_id
            )
        return self.move_to_ompl(
            arm, x, y, z, roll, pitch, yaw, execute=execute, frame_id=frame_id
        )

    def move_to_ompl(
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
        return super().move_to(
            arm, x, y, z, roll, pitch, yaw, execute=execute, frame_id=frame_id
        )

    def move_to_cartesian(
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
        max_step: float = 0.005,
        min_fraction: float = 0.9,
    ) -> bool:
        cfg = self._arm(arm)
        frame_id = frame_id or self.default_frame

        if not self._wait_for_joint_state():
            self.get_logger().error("Timeout waiting for joint state")
            return False

        if not self.cartesian_path_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Cartesian path service unavailable, falling back to OMPL")
            return self.move_to_ompl(
                arm, x, y, z, roll, pitch, yaw, execute=execute, frame_id=frame_id
            )

        target_pose = self._create_pose_target(x, y, z, roll, pitch, yaw)
        request = GetCartesianPath.Request()
        request.header.frame_id = frame_id
        request.start_state.joint_state = self.current_joint_state
        request.group_name = cfg.planning_group
        request.link_name = cfg.tcp_link
        request.waypoints = [target_pose]
        request.max_step = max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        self.get_logger().info(
            f"Planning Cartesian path for {arm} arm to "
            f"[{x:.3f}, {y:.3f}, {z:.3f}] in {frame_id}"
        )

        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("Cartesian path service returned no response")
            return False

        if response.fraction < min_fraction:
            self.get_logger().warn(
                f"Cartesian path for {arm} is only {response.fraction * 100.0:.1f}% feasible"
            )
            return self.move_to_ompl(
                arm, x, y, z, roll, pitch, yaw, execute=execute, frame_id=frame_id
            )

        if not execute:
            self.get_logger().info(
                f"Cartesian path planned for {arm}: {response.fraction * 100.0:.1f}%"
            )
            return True

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Cartesian trajectory execution was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Cartesian trajectory returned no result")
            return False

        error_code = result.result.error_code.val
        if error_code == MoveItErrorCodes.SUCCESS:
            self.fail_count[arm] = 0
            return True

        self.fail_count[arm] += 1
        self.get_logger().warn(f"Cartesian trajectory failed with error code {error_code}")
        if self.fail_count[arm] >= 4:
            return False
        time.sleep(0.2)
        return self.move_to_cartesian(
            arm,
            x,
            y,
            z,
            roll,
            pitch,
            yaw,
            execute=execute,
            frame_id=frame_id,
            max_step=max_step,
            min_fraction=min_fraction,
        )

    def move_dual_to_cartesian(
        self,
        left_pose: Sequence[float],
        right_pose: Sequence[float],
        execute: bool = True,
        frame_id: Optional[str] = None,
    ) -> bool:
        self.get_logger().warn(
            "Coordinated dual-arm Cartesian execution is not provided by this helper; "
            "using the dual-arm OMPL planner instead."
        )
        return self.move_dual_to(left_pose, right_pose, execute=execute, frame_id=frame_id)


def demo():
    rclpy.init()
    executor = MultiThreadedExecutor()
    controller = None
    try:
        controller = DualFR3LinearController()
        executor.add_node(controller)

        import threading

        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        controller.move_to_named_target("left", "home", execute=False)
        controller.move_to_named_target("right", "home", execute=False)
        controller.open_gripper("left")
        controller.open_gripper("right")
        print("Dual FR3 linear controller demo completed")
    finally:
        if controller is not None:
            controller.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    demo()
