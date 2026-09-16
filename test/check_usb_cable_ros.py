"""Check the same planning/execution services used by RViz, plus measured TF."""
import json
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
from dual_fr3_maniskill.cable.model import load_config


def main():
    assert os.environ.get('ROS_DOMAIN_ID', '0') != '0', 'Use an isolated ROS domain'
    config = load_config(Path(get_package_share_directory('dual_fr3_maniskill')) / 'config/usb_cable.yaml')
    rclpy.init()
    node = rclpy.create_node('usb_cable_demo_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    messages = {}
    subscriptions = []
    for key, topic, cls in [
        ('joints', '/joint_states', JointState), ('cable', '/usb_cable_demo/markers', MarkerArray),
        ('diagnostics', '/usb_cable_demo/diagnostics', DiagnosticArray),
        ('tcp', '/maniskill/left_tcp_pose', PoseStamped)]:
        def receive(msg, key=key):
            if key == 'cable':
                lines = [m for m in msg.markers if m.type == Marker.LINE_STRIP and m.action == Marker.ADD]
                if not lines:
                    return
                msg = MarkerArray(markers=lines)
            messages[key] = msg
        subscriptions.append(node.create_subscription(cls, topic, receive, 10))
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def wait(predicate, timeout=300):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            diagnostic = messages.get('diagnostics')
            if diagnostic is not None:
                assert all((ord(s.level) if isinstance(s.level, bytes) else s.level) < 2
                           for s in diagnostic.status), diagnostic
        assert predicate(), 'ROS demo check timed out'

    def result(future):
        wait(future.done)
        return future.result()

    try:
        wait(lambda: len(messages) == 4)
        if os.environ.get('USB_DEMO_CHECK_VIEWER') == '1':
            wait(lambda: 'rviz2' in node.get_node_names())
        assert node.count_publishers('/joint_states') == 1
        planning = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        assert planning.wait_for_service(timeout_sec=30)
        joint_state = messages['joints']
        initial = dict(zip(joint_state.name, joint_state.position))
        finger_positions = [initial[f'left_fr3_finger_joint{i}'] for i in (1, 2)]
        assert max(abs(q - config['usb'].get('open_finger_position', .02)) for q in finger_positions) < 1e-4
        wait(lambda: buffer.can_transform('world', 'usb_cable_demo_plug', Time()))
        before = buffer.lookup_transform('world', 'usb_cable_demo_plug', Time()).transform.translation
        initial_usb = np.array([before.x, before.y, before.z])
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = 'left_fr3_arm'
        plan.num_planning_attempts, plan.allowed_planning_time = 5, 5.
        plan.max_velocity_scaling_factor, plan.max_acceleration_scaling_factor = .05, .05
        plan.start_state.joint_state = joint_state
        target = [JointConstraint(joint_name=f'left_fr3_joint{i}',
            position=initial[f'left_fr3_joint{i}'] + (.04 if i == 1 else 0.),
            tolerance_above=.0001, tolerance_below=.0001, weight=1.) for i in range(1, 8)]
        plan.goal_constraints = [Constraints(joint_constraints=target)]
        response = result(planning.call_async(request)).motion_plan_response
        assert response.error_code.val == 1, response.error_code
        assert len(response.trajectory.joint_trajectory.points) > 1
        print('PASS: MoveIt left-arm plan while USB is externally supported in world', flush=True)
        client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        assert client.wait_for_server(timeout_sec=30)
        goal = ExecuteTrajectory.Goal(trajectory=response.trajectory)
        handle = result(client.send_goal_async(goal))
        assert handle.accepted
        finished = result(handle.get_result_async())
        assert finished.status == 4 and finished.result.error_code.val == 1, finished
        wait(lambda: abs(dict(zip(messages['joints'].name, messages['joints'].position))['left_fr3_joint1'] - target[0].position) < .005)
        print('PASS: ExecuteTrajectory completed from measured ManiSkill joint feedback', flush=True)
        diagnostics = {v.key: json.loads(v.value) for v in messages['diagnostics'].status[0].values}
        assert diagnostics['max_rigid_penetration_m'] <= .0001, diagnostics

        def tf_ready():
            header = messages['cable'].markers[0].header
            return buffer.can_transform('world', 'usb_cable_demo_plug', Time.from_msg(header.stamp))

        wait(tf_ready)
        marker = messages['cable'].markers[0]
        transform = buffer.lookup_transform('world', 'usb_cable_demo_plug', Time.from_msg(marker.header.stamp)).transform
        q, p = transform.rotation, transform.translation
        expected = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(config['usb']['attachment']) + [p.x,p.y,p.z]
        root = marker.points[0]
        error = np.linalg.norm(expected - [root.x,root.y,root.z])
        assert error < 1e-4, error
        world_drift = np.linalg.norm(np.array([p.x,p.y,p.z]) - initial_usb)
        assert world_drift < 1e-4, world_drift
        tcp = messages['tcp']
        wait(lambda: buffer.can_transform('world', 'left_fr3_hand_tcp', Time.from_msg(tcp.header.stamp)))
        tf_tcp = buffer.lookup_transform('world', 'left_fr3_hand_tcp', Time.from_msg(tcp.header.stamp)).transform.translation
        tcp_error = np.linalg.norm(np.array([tf_tcp.x,tf_tcp.y,tf_tcp.z]) - [tcp.pose.position.x,tcp.pose.position.y,tcp.pose.position.z])
        assert tcp_error < 1e-4, tcp_error
        gripper = ActionClient(node, GripperCommand, '/left_franka_gripper/gripper_cmd')
        assert gripper.wait_for_server(timeout_sec=10)
        opening = GripperCommand.Goal()
        opening.command.position, opening.command.max_effort = .04, 10.
        opening_handle = result(gripper.send_goal_async(opening))
        assert opening_handle.accepted
        opened = result(opening_handle.get_result_async())
        assert opened.status == 4 and opened.result.reached_goal
        reset = node.create_client(Trigger, '/usb_cable_demo/reset')
        assert reset.wait_for_service(timeout_sec=10)
        assert result(reset.call_async(Trigger.Request())).success
        status = node.create_client(Trigger, '/maniskill/usb/status')
        assert status.wait_for_service(timeout_sec=10)
        cleared = result(status.call_async(Trigger.Request()))
        assert json.loads(cleared.message)['state'] == 'not_created'
        spawn = node.create_client(Trigger, '/maniskill/cable/spawn')
        assert spawn.wait_for_service(timeout_sec=10)
        assert result(spawn.call_async(Trigger.Request())).success
        supported = json.loads(result(status.call_async(Trigger.Request())).message)
        assert supported['external_support'] and supported['state'] == 'supported'
        report = {'usb_root_tf_error_m': float(error), 'tcp_tf_error_m': float(tcp_error),
                  'usb_world_support_drift_m': float(world_drift),
                  'left_gripper_opening_m': float(sum(finger_positions)),
                  'left_joint1_motion_rad': target[0].position - initial['left_fr3_joint1'],
                  'marker_diameter_m': marker.scale.x,
                  'diagnostics': {v.key: json.loads(v.value) for v in messages['diagnostics'].status[0].values}}
        print(json.dumps(report, indent=2), flush=True)
        print('PASS: MPM/USB TF, fixed world support, accepted opening, reset/respawn; contact grasp not tested here', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
