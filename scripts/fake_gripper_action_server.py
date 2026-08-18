#!/usr/bin/env python3

import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


class FakeGripperActionServer(Node):
    def __init__(self):
        super().__init__("fake_gripper_action_server")

        self.declare_parameter("joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("state_publish_rate", 15)
        self.declare_parameter("max_finger_position", 0.04)

        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        if len(self.joint_names) != 2:
            raise ValueError("Parameter 'joint_names' must contain exactly two joints")

        self.position = float(self.get_parameter("max_finger_position").value)
        self.max_finger_position = float(self.get_parameter("max_finger_position").value)
        publish_rate = float(self.get_parameter("state_publish_rate").value)

        self.joint_state_publisher = self.create_publisher(JointState, "~/joint_states", 1)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_state)
        self.action_server = ActionServer(
            self,
            GripperCommand,
            "~/gripper_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        target = goal_request.command.position
        if target < 0.0 or target > self.max_finger_position:
            self.get_logger().warn(
                f"Rejecting gripper target {target:.4f}; valid range is "
                f"[0.0, {self.max_finger_position:.4f}]"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        start_position = self.position
        target_position = goal_handle.request.command.position
        steps = 10

        for step in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                result = GripperCommand.Result()
                result.position = self.position * 2.0
                result.effort = 0.0
                result.stalled = False
                result.reached_goal = False
                goal_handle.canceled()
                return result

            ratio = step / steps
            self.position = start_position + (target_position - start_position) * ratio

            feedback = GripperCommand.Feedback()
            feedback.position = self.position * 2.0
            feedback.effort = 0.0
            goal_handle.publish_feedback(feedback)
            self.publish_state()
            time.sleep(0.03)

        result = GripperCommand.Result()
        result.position = self.position * 2.0
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        goal_handle.succeed()
        return result

    def publish_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_names)
        joint_state.position = [self.position, self.position]
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        self.joint_state_publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGripperActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
