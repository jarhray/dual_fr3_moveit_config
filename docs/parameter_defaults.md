# 参数默认值与声明位置（自动生成）

由 `check_parameter_docs.py --write-defaults` 从当前源码生成。
同名参数的不同入口逐行保留；表达式表示运行时求值，不代表唯一有效值。
作用、单位、消费模块及验证见 [参数索引](parameters.md)。YAML 列出当前文件值，不继承其他预设。

| 参数 | 源字段（区分实例） | 声明文件 | 默认值 / 表达式 |
| --- | --- | --- | --- |
| `/left/left_fr3_arm_controller.action_ns` | `/left/left_fr3_arm_controller.action_ns` | [config/moveit_controllers.yaml:8](../config/moveit_controllers.yaml#L8) | `'follow_joint_trajectory'` |
| `/left/left_fr3_arm_controller.default` | `/left/left_fr3_arm_controller.default` | [config/moveit_controllers.yaml:10](../config/moveit_controllers.yaml#L10) | `True` |
| `/left/left_fr3_arm_controller.joints` | `/left/left_fr3_arm_controller.joints` | [config/moveit_controllers.yaml:12](../config/moveit_controllers.yaml#L12) | `['left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3', 'left_fr3_joint4', 'left_fr3_joint5', 'left_fr3_joint6', 'left_fr3_joint7']` |
| `/left/left_fr3_arm_controller.type` | `/left/left_fr3_arm_controller.type` | [config/moveit_controllers.yaml:9](../config/moveit_controllers.yaml#L9) | `'FollowJointTrajectory'` |
| `/right/right_fr3_arm_controller.action_ns` | `/right/right_fr3_arm_controller.action_ns` | [config/moveit_controllers.yaml:21](../config/moveit_controllers.yaml#L21) | `'follow_joint_trajectory'` |
| `/right/right_fr3_arm_controller.default` | `/right/right_fr3_arm_controller.default` | [config/moveit_controllers.yaml:23](../config/moveit_controllers.yaml#L23) | `True` |
| `/right/right_fr3_arm_controller.joints` | `/right/right_fr3_arm_controller.joints` | [config/moveit_controllers.yaml:25](../config/moveit_controllers.yaml#L25) | `['right_fr3_joint1', 'right_fr3_joint2', 'right_fr3_joint3', 'right_fr3_joint4', 'right_fr3_joint5', 'right_fr3_joint6', 'right_fr3_joint7']` |
| `/right/right_fr3_arm_controller.type` | `/right/right_fr3_arm_controller.type` | [config/moveit_controllers.yaml:22](../config/moveit_controllers.yaml#L22) | `'FollowJointTrajectory'` |
| `allow_stalling` | `/**.left_franka_gripper.ros__parameters.allow_stalling` | [config/gazebo_ros2_controllers.yaml:62](../config/gazebo_ros2_controllers.yaml#L62) | `True` |
| `allow_stalling` | `/**.right_franka_gripper.ros__parameters.allow_stalling` | [config/gazebo_ros2_controllers.yaml:70](../config/gazebo_ros2_controllers.yaml#L70) | `True` |
| `arm_prefix` | `/**.left_franka_robot_state_broadcaster.ros__parameters.arm_prefix` | [config/ros2_controllers.yaml:73](../config/ros2_controllers.yaml#L73) | `'left'` |
| `arm_prefix` | `/**.right_franka_robot_state_broadcaster.ros__parameters.arm_prefix` | [config/ros2_controllers.yaml:79](../config/ros2_controllers.yaml#L79) | `'right'` |
| `arm_yaw` | `arm_yaw` | [config/dual_fr3.gazebo.urdf.xacro:1](../config/dual_fr3.gazebo.urdf.xacro#L1) | `0` |
| `arm_yaw` | `arm_yaw` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `0` |
| `cable_config` | `cable_config` | [launch/demo.launch.py:368](../launch/demo.launch.py#L368) | `''` |
| `cable_config` | `cable_config` | [launch/maniskill.launch.py:170](../launch/maniskill.launch.py#L170) | `''` |
| `cable_solver` | `cable_solver` | [launch/demo.launch.py:366](../launch/demo.launch.py#L366) | `'mpm'` |
| `cable_solver` | `cable_solver` | [launch/maniskill.launch.py:148](../launch/maniskill.launch.py#L148) | `'mpm'` |
| `cable_trace_dir` | `cable_trace_dir` | [launch/demo.launch.py:369](../launch/demo.launch.py#L369) | `''` |
| `cable_trace_dir` | `cable_trace_dir` | [launch/maniskill.launch.py:165](../launch/maniskill.launch.py#L165) | `''` |
| `capabilities` | `capabilities` | [launch/demo.launch.py:425](../launch/demo.launch.py#L425) | `''` |
| `capabilities` | `capabilities` | [launch/gazebo.launch.py:255](../launch/gazebo.launch.py#L255) | `''` |
| `capabilities` | `capabilities` | [launch/maniskill.launch.py:192](../launch/maniskill.launch.py#L192) | `'move_group/ExecuteTaskSolutionCapability'` |
| `capabilities` | `capabilities` | [launch/usb_cable.launch.py:13](../launch/usb_cable.launch.py#L13) | `''` |
| `command_interfaces` | `/**.left_fr3_arm_controller.ros__parameters.command_interfaces` | [config/gazebo_ros2_controllers.yaml:29](../config/gazebo_ros2_controllers.yaml#L29) | `['position']` |
| `command_interfaces` | `/**.right_fr3_arm_controller.ros__parameters.command_interfaces` | [config/gazebo_ros2_controllers.yaml:45](../config/gazebo_ros2_controllers.yaml#L45) | `['position']` |
| `command_interfaces` | `/**.left_fr3_arm_controller.ros__parameters.command_interfaces` | [config/ros2_controllers.yaml:25](../config/ros2_controllers.yaml#L25) | `['effort']` |
| `command_interfaces` | `/**.right_fr3_arm_controller.ros__parameters.command_interfaces` | [config/ros2_controllers.yaml:49](../config/ros2_controllers.yaml#L49) | `['effort']` |
| `controller_names` | `controller_names` | [config/moveit_controllers.yaml:2](../config/moveit_controllers.yaml#L2) | `['/left/left_fr3_arm_controller', '/right/right_fr3_arm_controller', 'left_franka_gripper', 'right_franka_gripper']` |
| `controller_names` | `controller_names` | [config/moveit_controllers_gazebo.yaml:2](../config/moveit_controllers_gazebo.yaml#L2) | `['left_fr3_arm_controller', 'right_fr3_arm_controller', 'left_franka_gripper', 'right_franka_gripper']` |
| `convenience_publish_rate` | `/**.left_franka_robot_state_broadcaster.ros__parameters.convenience_publish_rate` | [config/ros2_controllers.yaml:74](../config/ros2_controllers.yaml#L74) | `1000` |
| `convenience_publish_rate` | `/**.right_franka_robot_state_broadcaster.ros__parameters.convenience_publish_rate` | [config/ros2_controllers.yaml:80](../config/ros2_controllers.yaml#L80) | `1000` |
| `disable_capabilities` | `disable_capabilities` | [launch/demo.launch.py:430](../launch/demo.launch.py#L430) | `''` |
| `disable_capabilities` | `disable_capabilities` | [launch/gazebo.launch.py:260](../launch/gazebo.launch.py#L260) | `''` |
| `disable_capabilities` | `disable_capabilities` | [launch/maniskill.launch.py:195](../launch/maniskill.launch.py#L195) | `''` |
| `dual_fr3_arms.longest_valid_segment_fraction` | `dual_fr3_arms.longest_valid_segment_fraction` | [config/ompl_planning.yaml:29](../config/ompl_planning.yaml#L29) | `0.002` |
| `dual_fr3_arms.planner_configs` | `dual_fr3_arms.planner_configs` | [config/ompl_planning.yaml:31](../config/ompl_planning.yaml#L31) | `['RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'PRMkConfigDefault']` |
| `ee_id` | `ee_id` | [config/dual_fr3.gazebo.urdf.xacro:1](../config/dual_fr3.gazebo.urdf.xacro#L1) | `franka_hand` |
| `ee_id` | `ee_id` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `franka_hand` |
| `ee_id` | `ee_id` | [launch/demo.launch.py:399](../launch/demo.launch.py#L399) | `'franka_hand'` |
| `ee_id` | `ee_id` | [launch/gazebo.launch.py:225](../launch/gazebo.launch.py#L225) | `'franka_hand'` |
| `ee_id` | `ee_id` | [launch/maniskill.launch.py:176](../launch/maniskill.launch.py#L176) | `'franka_hand'` |
| `fake_sensor_commands` | `fake_sensor_commands` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `false` |
| `fake_sensor_commands` | `fake_sensor_commands` | [launch/demo.launch.py:374](../launch/demo.launch.py#L374) | `'false'` |
| `gains.left_fr3_joint1.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint1.d` | [config/ros2_controllers.yaml:38](../config/ros2_controllers.yaml#L38) | `30.0` |
| `gains.left_fr3_joint1.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint1.i` | [config/ros2_controllers.yaml:38](../config/ros2_controllers.yaml#L38) | `0.0` |
| `gains.left_fr3_joint1.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint1.i_clamp` | [config/ros2_controllers.yaml:38](../config/ros2_controllers.yaml#L38) | `1.0` |
| `gains.left_fr3_joint1.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint1.p` | [config/ros2_controllers.yaml:38](../config/ros2_controllers.yaml#L38) | `600.0` |
| `gains.left_fr3_joint2.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint2.d` | [config/ros2_controllers.yaml:39](../config/ros2_controllers.yaml#L39) | `30.0` |
| `gains.left_fr3_joint2.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint2.i` | [config/ros2_controllers.yaml:39](../config/ros2_controllers.yaml#L39) | `0.0` |
| `gains.left_fr3_joint2.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint2.i_clamp` | [config/ros2_controllers.yaml:39](../config/ros2_controllers.yaml#L39) | `1.0` |
| `gains.left_fr3_joint2.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint2.p` | [config/ros2_controllers.yaml:39](../config/ros2_controllers.yaml#L39) | `600.0` |
| `gains.left_fr3_joint3.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint3.d` | [config/ros2_controllers.yaml:40](../config/ros2_controllers.yaml#L40) | `30.0` |
| `gains.left_fr3_joint3.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint3.i` | [config/ros2_controllers.yaml:40](../config/ros2_controllers.yaml#L40) | `0.0` |
| `gains.left_fr3_joint3.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint3.i_clamp` | [config/ros2_controllers.yaml:40](../config/ros2_controllers.yaml#L40) | `1.0` |
| `gains.left_fr3_joint3.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint3.p` | [config/ros2_controllers.yaml:40](../config/ros2_controllers.yaml#L40) | `600.0` |
| `gains.left_fr3_joint4.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint4.d` | [config/ros2_controllers.yaml:41](../config/ros2_controllers.yaml#L41) | `30.0` |
| `gains.left_fr3_joint4.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint4.i` | [config/ros2_controllers.yaml:41](../config/ros2_controllers.yaml#L41) | `0.0` |
| `gains.left_fr3_joint4.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint4.i_clamp` | [config/ros2_controllers.yaml:41](../config/ros2_controllers.yaml#L41) | `1.0` |
| `gains.left_fr3_joint4.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint4.p` | [config/ros2_controllers.yaml:41](../config/ros2_controllers.yaml#L41) | `600.0` |
| `gains.left_fr3_joint5.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint5.d` | [config/ros2_controllers.yaml:42](../config/ros2_controllers.yaml#L42) | `10.0` |
| `gains.left_fr3_joint5.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint5.i` | [config/ros2_controllers.yaml:42](../config/ros2_controllers.yaml#L42) | `0.0` |
| `gains.left_fr3_joint5.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint5.i_clamp` | [config/ros2_controllers.yaml:42](../config/ros2_controllers.yaml#L42) | `1.0` |
| `gains.left_fr3_joint5.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint5.p` | [config/ros2_controllers.yaml:42](../config/ros2_controllers.yaml#L42) | `250.0` |
| `gains.left_fr3_joint6.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint6.d` | [config/ros2_controllers.yaml:43](../config/ros2_controllers.yaml#L43) | `10.0` |
| `gains.left_fr3_joint6.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint6.i` | [config/ros2_controllers.yaml:43](../config/ros2_controllers.yaml#L43) | `0.0` |
| `gains.left_fr3_joint6.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint6.i_clamp` | [config/ros2_controllers.yaml:43](../config/ros2_controllers.yaml#L43) | `1.0` |
| `gains.left_fr3_joint6.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint6.p` | [config/ros2_controllers.yaml:43](../config/ros2_controllers.yaml#L43) | `150.0` |
| `gains.left_fr3_joint7.d` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint7.d` | [config/ros2_controllers.yaml:44](../config/ros2_controllers.yaml#L44) | `5.0` |
| `gains.left_fr3_joint7.i` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint7.i` | [config/ros2_controllers.yaml:44](../config/ros2_controllers.yaml#L44) | `0.0` |
| `gains.left_fr3_joint7.i_clamp` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint7.i_clamp` | [config/ros2_controllers.yaml:44](../config/ros2_controllers.yaml#L44) | `1.0` |
| `gains.left_fr3_joint7.p` | `/**.left_fr3_arm_controller.ros__parameters.gains.left_fr3_joint7.p` | [config/ros2_controllers.yaml:44](../config/ros2_controllers.yaml#L44) | `50.0` |
| `gains.right_fr3_joint1.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint1.d` | [config/ros2_controllers.yaml:62](../config/ros2_controllers.yaml#L62) | `30.0` |
| `gains.right_fr3_joint1.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint1.i` | [config/ros2_controllers.yaml:62](../config/ros2_controllers.yaml#L62) | `0.0` |
| `gains.right_fr3_joint1.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint1.i_clamp` | [config/ros2_controllers.yaml:62](../config/ros2_controllers.yaml#L62) | `1.0` |
| `gains.right_fr3_joint1.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint1.p` | [config/ros2_controllers.yaml:62](../config/ros2_controllers.yaml#L62) | `600.0` |
| `gains.right_fr3_joint2.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint2.d` | [config/ros2_controllers.yaml:63](../config/ros2_controllers.yaml#L63) | `30.0` |
| `gains.right_fr3_joint2.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint2.i` | [config/ros2_controllers.yaml:63](../config/ros2_controllers.yaml#L63) | `0.0` |
| `gains.right_fr3_joint2.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint2.i_clamp` | [config/ros2_controllers.yaml:63](../config/ros2_controllers.yaml#L63) | `1.0` |
| `gains.right_fr3_joint2.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint2.p` | [config/ros2_controllers.yaml:63](../config/ros2_controllers.yaml#L63) | `600.0` |
| `gains.right_fr3_joint3.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint3.d` | [config/ros2_controllers.yaml:64](../config/ros2_controllers.yaml#L64) | `30.0` |
| `gains.right_fr3_joint3.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint3.i` | [config/ros2_controllers.yaml:64](../config/ros2_controllers.yaml#L64) | `0.0` |
| `gains.right_fr3_joint3.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint3.i_clamp` | [config/ros2_controllers.yaml:64](../config/ros2_controllers.yaml#L64) | `1.0` |
| `gains.right_fr3_joint3.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint3.p` | [config/ros2_controllers.yaml:64](../config/ros2_controllers.yaml#L64) | `600.0` |
| `gains.right_fr3_joint4.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint4.d` | [config/ros2_controllers.yaml:65](../config/ros2_controllers.yaml#L65) | `30.0` |
| `gains.right_fr3_joint4.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint4.i` | [config/ros2_controllers.yaml:65](../config/ros2_controllers.yaml#L65) | `0.0` |
| `gains.right_fr3_joint4.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint4.i_clamp` | [config/ros2_controllers.yaml:65](../config/ros2_controllers.yaml#L65) | `1.0` |
| `gains.right_fr3_joint4.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint4.p` | [config/ros2_controllers.yaml:65](../config/ros2_controllers.yaml#L65) | `600.0` |
| `gains.right_fr3_joint5.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint5.d` | [config/ros2_controllers.yaml:66](../config/ros2_controllers.yaml#L66) | `10.0` |
| `gains.right_fr3_joint5.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint5.i` | [config/ros2_controllers.yaml:66](../config/ros2_controllers.yaml#L66) | `0.0` |
| `gains.right_fr3_joint5.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint5.i_clamp` | [config/ros2_controllers.yaml:66](../config/ros2_controllers.yaml#L66) | `1.0` |
| `gains.right_fr3_joint5.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint5.p` | [config/ros2_controllers.yaml:66](../config/ros2_controllers.yaml#L66) | `250.0` |
| `gains.right_fr3_joint6.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint6.d` | [config/ros2_controllers.yaml:67](../config/ros2_controllers.yaml#L67) | `10.0` |
| `gains.right_fr3_joint6.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint6.i` | [config/ros2_controllers.yaml:67](../config/ros2_controllers.yaml#L67) | `0.0` |
| `gains.right_fr3_joint6.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint6.i_clamp` | [config/ros2_controllers.yaml:67](../config/ros2_controllers.yaml#L67) | `1.0` |
| `gains.right_fr3_joint6.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint6.p` | [config/ros2_controllers.yaml:67](../config/ros2_controllers.yaml#L67) | `150.0` |
| `gains.right_fr3_joint7.d` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint7.d` | [config/ros2_controllers.yaml:68](../config/ros2_controllers.yaml#L68) | `5.0` |
| `gains.right_fr3_joint7.i` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint7.i` | [config/ros2_controllers.yaml:68](../config/ros2_controllers.yaml#L68) | `0.0` |
| `gains.right_fr3_joint7.i_clamp` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint7.i_clamp` | [config/ros2_controllers.yaml:68](../config/ros2_controllers.yaml#L68) | `1.0` |
| `gains.right_fr3_joint7.p` | `/**.right_fr3_arm_controller.ros__parameters.gains.right_fr3_joint7.p` | [config/ros2_controllers.yaml:68](../config/ros2_controllers.yaml#L68) | `50.0` |
| `gazebo_effort` | `gazebo_effort` | [config/dual_fr3.gazebo.urdf.xacro:1](../config/dual_fr3.gazebo.urdf.xacro#L1) | `true` |
| `gazebo_effort` | `gazebo_effort` | [launch/demo.launch.py:363](../launch/demo.launch.py#L363) | `'false'` |
| `gazebo_effort` | `gazebo_effort` | [launch/gazebo.launch.py:230](../launch/gazebo.launch.py#L230) | `'false'` |
| `goal_tolerance` | `/**.left_franka_gripper.ros__parameters.goal_tolerance` | [config/gazebo_ros2_controllers.yaml:61](../config/gazebo_ros2_controllers.yaml#L61) | `0.001` |
| `goal_tolerance` | `/**.right_franka_gripper.ros__parameters.goal_tolerance` | [config/gazebo_ros2_controllers.yaml:69](../config/gazebo_ros2_controllers.yaml#L69) | `0.001` |
| `gz_args` | `gz_args` | [launch/demo.launch.py:362](../launch/demo.launch.py#L362) | `'empty.sdf -r'` |
| `gz_args` | `gz_args` | [launch/gazebo.launch.py:265](../launch/gazebo.launch.py#L265) | `'empty.sdf -r'` |
| `insertion_enabled` | `insertion_enabled` | [launch/maniskill.launch.py:151](../launch/maniskill.launch.py#L151) | `'true'` |
| `joint` | `/**.left_franka_gripper.ros__parameters.joint` | [config/gazebo_ros2_controllers.yaml:60](../config/gazebo_ros2_controllers.yaml#L60) | `'left_fr3_finger_joint1'` |
| `joint` | `/**.right_franka_gripper.ros__parameters.joint` | [config/gazebo_ros2_controllers.yaml:68](../config/gazebo_ros2_controllers.yaml#L68) | `'right_fr3_finger_joint1'` |
| `joint_state_broadcaster.type` | `/**.controller_manager.ros__parameters.joint_state_broadcaster.type` | [config/gazebo_ros2_controllers.yaml:8](../config/gazebo_ros2_controllers.yaml#L8) | `'joint_state_broadcaster/JointStateBroadcaster'` |
| `joint_state_broadcaster.type` | `/**.controller_manager.ros__parameters.joint_state_broadcaster.type` | [config/ros2_controllers.yaml:8](../config/ros2_controllers.yaml#L8) | `'joint_state_broadcaster/JointStateBroadcaster'` |
| `joints` | `/**.left_fr3_arm_controller.ros__parameters.joints` | [config/gazebo_ros2_controllers.yaml:34](../config/gazebo_ros2_controllers.yaml#L34) | `['left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3', 'left_fr3_joint4', 'left_fr3_joint5', 'left_fr3_joint6', 'left_fr3_joint7']` |
| `joints` | `/**.right_fr3_arm_controller.ros__parameters.joints` | [config/gazebo_ros2_controllers.yaml:50](../config/gazebo_ros2_controllers.yaml#L50) | `['right_fr3_joint1', 'right_fr3_joint2', 'right_fr3_joint3', 'right_fr3_joint4', 'right_fr3_joint5', 'right_fr3_joint6', 'right_fr3_joint7']` |
| `joints` | `/**.left_fr3_arm_controller.ros__parameters.joints` | [config/ros2_controllers.yaml:30](../config/ros2_controllers.yaml#L30) | `['left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3', 'left_fr3_joint4', 'left_fr3_joint5', 'left_fr3_joint6', 'left_fr3_joint7']` |
| `joints` | `/**.right_fr3_arm_controller.ros__parameters.joints` | [config/ros2_controllers.yaml:54](../config/ros2_controllers.yaml#L54) | `['right_fr3_joint1', 'right_fr3_joint2', 'right_fr3_joint3', 'right_fr3_joint4', 'right_fr3_joint5', 'right_fr3_joint6', 'right_fr3_joint7']` |
| `leader_orientation_direction` | `leader_orientation_direction` | [launch/maniskill.launch.py:160](../launch/maniskill.launch.py#L160) | `'reverse'` |
| `left_fr3_arm.kinematics_solver` | `left_fr3_arm.kinematics_solver` | [config/kinematics.yaml:2](../config/kinematics.yaml#L2) | `'lma_kinematics_plugin/LMAKinematicsPlugin'` |
| `left_fr3_arm.kinematics_solver_search_resolution` | `left_fr3_arm.kinematics_solver_search_resolution` | [config/kinematics.yaml:3](../config/kinematics.yaml#L3) | `0.005` |
| `left_fr3_arm.kinematics_solver_timeout` | `left_fr3_arm.kinematics_solver_timeout` | [config/kinematics.yaml:4](../config/kinematics.yaml#L4) | `0.01` |
| `left_fr3_arm.longest_valid_segment_fraction` | `left_fr3_arm.longest_valid_segment_fraction` | [config/ompl_planning.yaml:15](../config/ompl_planning.yaml#L15) | `0.002` |
| `left_fr3_arm.planner_configs` | `left_fr3_arm.planner_configs` | [config/ompl_planning.yaml:17](../config/ompl_planning.yaml#L17) | `['RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'PRMkConfigDefault']` |
| `left_fr3_arm_controller.action_ns` | `left_fr3_arm_controller.action_ns` | [config/moveit_controllers_gazebo.yaml:8](../config/moveit_controllers_gazebo.yaml#L8) | `'follow_joint_trajectory'` |
| `left_fr3_arm_controller.default` | `left_fr3_arm_controller.default` | [config/moveit_controllers_gazebo.yaml:10](../config/moveit_controllers_gazebo.yaml#L10) | `True` |
| `left_fr3_arm_controller.joints` | `left_fr3_arm_controller.joints` | [config/moveit_controllers_gazebo.yaml:12](../config/moveit_controllers_gazebo.yaml#L12) | `['left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3', 'left_fr3_joint4', 'left_fr3_joint5', 'left_fr3_joint6', 'left_fr3_joint7']` |
| `left_fr3_arm_controller.type` | `/**.controller_manager.ros__parameters.left_fr3_arm_controller.type` | [config/gazebo_ros2_controllers.yaml:11](../config/gazebo_ros2_controllers.yaml#L11) | `'joint_trajectory_controller/JointTrajectoryController'` |
| `left_fr3_arm_controller.type` | `left_fr3_arm_controller.type` | [config/moveit_controllers_gazebo.yaml:9](../config/moveit_controllers_gazebo.yaml#L9) | `'FollowJointTrajectory'` |
| `left_fr3_arm_controller.type` | `/**.controller_manager.ros__parameters.left_fr3_arm_controller.type` | [config/ros2_controllers.yaml:11](../config/ros2_controllers.yaml#L11) | `'joint_trajectory_controller/JointTrajectoryController'` |
| `left_franka_gripper.action_ns` | `left_franka_gripper.action_ns` | [config/moveit_controllers.yaml:34](../config/moveit_controllers.yaml#L34) | `'gripper_action'` |
| `left_franka_gripper.action_ns` | `left_franka_gripper.action_ns` | [config/moveit_controllers_gazebo.yaml:34](../config/moveit_controllers_gazebo.yaml#L34) | `'gripper_cmd'` |
| `left_franka_gripper.default` | `left_franka_gripper.default` | [config/moveit_controllers.yaml:36](../config/moveit_controllers.yaml#L36) | `True` |
| `left_franka_gripper.default` | `left_franka_gripper.default` | [config/moveit_controllers_gazebo.yaml:36](../config/moveit_controllers_gazebo.yaml#L36) | `True` |
| `left_franka_gripper.joints` | `left_franka_gripper.joints` | [config/moveit_controllers.yaml:38](../config/moveit_controllers.yaml#L38) | `['left_fr3_finger_joint1', 'left_fr3_finger_joint2']` |
| `left_franka_gripper.joints` | `left_franka_gripper.joints` | [config/moveit_controllers_gazebo.yaml:38](../config/moveit_controllers_gazebo.yaml#L38) | `['left_fr3_finger_joint1', 'left_fr3_finger_joint2']` |
| `left_franka_gripper.type` | `/**.controller_manager.ros__parameters.left_franka_gripper.type` | [config/gazebo_ros2_controllers.yaml:17](../config/gazebo_ros2_controllers.yaml#L17) | `'position_controllers/GripperActionController'` |
| `left_franka_gripper.type` | `left_franka_gripper.type` | [config/moveit_controllers.yaml:35](../config/moveit_controllers.yaml#L35) | `'GripperCommand'` |
| `left_franka_gripper.type` | `left_franka_gripper.type` | [config/moveit_controllers_gazebo.yaml:35](../config/moveit_controllers_gazebo.yaml#L35) | `'GripperCommand'` |
| `left_franka_robot_state_broadcaster.type` | `/**.controller_manager.ros__parameters.left_franka_robot_state_broadcaster.type` | [config/ros2_controllers.yaml:17](../config/ros2_controllers.yaml#L17) | `'franka_robot_state_broadcaster/FrankaRobotStateBroadcaster'` |
| `left_robot_ip` | `left_robot_ip` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `` |
| `left_robot_ip` | `left_robot_ip` | [launch/demo.launch.py:379](../launch/demo.launch.py#L379) | `''` |
| `load_cable` | `load_cable` | [launch/demo.launch.py:367](../launch/demo.launch.py#L367) | `'true'` |
| `load_cable` | `load_cable` | [launch/maniskill.launch.py:154](../launch/maniskill.launch.py#L154) | `'true'` |
| `load_gripper` | `load_gripper` | [config/dual_fr3.gazebo.urdf.xacro:1](../config/dual_fr3.gazebo.urdf.xacro#L1) | `true` |
| `load_gripper` | `load_gripper` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `true` |
| `load_gripper` | `load_gripper` | [launch/demo.launch.py:389](../launch/demo.launch.py#L389) | `'true'` |
| `load_gripper` | `load_gripper` | [launch/gazebo.launch.py:220](../launch/gazebo.launch.py#L220) | `'true'` |
| `load_gripper` | `load_gripper` | [launch/maniskill.launch.py:175](../launch/maniskill.launch.py#L175) | `'true'` |
| `load_left_ros2_control` | `load_left_ros2_control` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `true` |
| `load_right_ros2_control` | `load_right_ros2_control` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `true` |
| `maniskill_config` | `maniskill_config` | [launch/demo.launch.py:370](../launch/demo.launch.py#L370) | `''` |
| `maniskill_config` | `maniskill_config` | [launch/maniskill.launch.py:181](../launch/maniskill.launch.py#L181) | `''` |
| `maniskill_python` | `maniskill_python` | [launch/demo.launch.py:372](../launch/demo.launch.py#L372) | `os.environ.get('MANISKILL_PYTHON', str(Path.cwd() / '.venv/bin/python'))` |
| `maniskill_python` | `maniskill_python` | [launch/maniskill.launch.py:180](../launch/maniskill.launch.py#L180) | `python` |
| `maniskill_scene` | `maniskill_scene` | [launch/demo.launch.py:365](../launch/demo.launch.py#L365) | `'robot'` |
| `maniskill_scene` | `maniskill_scene` | [launch/maniskill.launch.py:145](../launch/maniskill.launch.py#L145) | `'robot'` |
| `maniskill_scene` | `maniskill_scene` | [launch/usb_cable.launch.py:12](../launch/usb_cable.launch.py#L12) | `'usb_cable'` |
| `maniskill_viewer` | `maniskill_viewer` | [launch/demo.launch.py:364](../launch/demo.launch.py#L364) | `'true'` |
| `maniskill_viewer` | `maniskill_viewer` | [launch/maniskill.launch.py:179](../launch/maniskill.launch.py#L179) | `'true'` |
| `planner_configs.PRMkConfigDefault.max_nearest_neighbors` | `planner_configs.PRMkConfigDefault.max_nearest_neighbors` | [config/ompl_planning.yaml:12](../config/ompl_planning.yaml#L12) | `10` |
| `planner_configs.PRMkConfigDefault.type` | `planner_configs.PRMkConfigDefault.type` | [config/ompl_planning.yaml:11](../config/ompl_planning.yaml#L11) | `'geometric::PRM'` |
| `planner_configs.RRTConnectkConfigDefault.range` | `planner_configs.RRTConnectkConfigDefault.range` | [config/ompl_planning.yaml:4](../config/ompl_planning.yaml#L4) | `0.0` |
| `planner_configs.RRTConnectkConfigDefault.type` | `planner_configs.RRTConnectkConfigDefault.type` | [config/ompl_planning.yaml:3](../config/ompl_planning.yaml#L3) | `'geometric::RRTConnect'` |
| `planner_configs.RRTstarkConfigDefault.delay_collision_checking` | `planner_configs.RRTstarkConfigDefault.delay_collision_checking` | [config/ompl_planning.yaml:9](../config/ompl_planning.yaml#L9) | `1` |
| `planner_configs.RRTstarkConfigDefault.goal_bias` | `planner_configs.RRTstarkConfigDefault.goal_bias` | [config/ompl_planning.yaml:8](../config/ompl_planning.yaml#L8) | `0.05` |
| `planner_configs.RRTstarkConfigDefault.range` | `planner_configs.RRTstarkConfigDefault.range` | [config/ompl_planning.yaml:7](../config/ompl_planning.yaml#L7) | `0.0` |
| `planner_configs.RRTstarkConfigDefault.type` | `planner_configs.RRTstarkConfigDefault.type` | [config/ompl_planning.yaml:6](../config/ompl_planning.yaml#L6) | `'geometric::RRTstar'` |
| `right_fr3_arm.kinematics_solver` | `right_fr3_arm.kinematics_solver` | [config/kinematics.yaml:7](../config/kinematics.yaml#L7) | `'lma_kinematics_plugin/LMAKinematicsPlugin'` |
| `right_fr3_arm.kinematics_solver_search_resolution` | `right_fr3_arm.kinematics_solver_search_resolution` | [config/kinematics.yaml:8](../config/kinematics.yaml#L8) | `0.005` |
| `right_fr3_arm.kinematics_solver_timeout` | `right_fr3_arm.kinematics_solver_timeout` | [config/kinematics.yaml:9](../config/kinematics.yaml#L9) | `0.01` |
| `right_fr3_arm.longest_valid_segment_fraction` | `right_fr3_arm.longest_valid_segment_fraction` | [config/ompl_planning.yaml:22](../config/ompl_planning.yaml#L22) | `0.002` |
| `right_fr3_arm.planner_configs` | `right_fr3_arm.planner_configs` | [config/ompl_planning.yaml:24](../config/ompl_planning.yaml#L24) | `['RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'PRMkConfigDefault']` |
| `right_fr3_arm_controller.action_ns` | `right_fr3_arm_controller.action_ns` | [config/moveit_controllers_gazebo.yaml:21](../config/moveit_controllers_gazebo.yaml#L21) | `'follow_joint_trajectory'` |
| `right_fr3_arm_controller.default` | `right_fr3_arm_controller.default` | [config/moveit_controllers_gazebo.yaml:23](../config/moveit_controllers_gazebo.yaml#L23) | `True` |
| `right_fr3_arm_controller.joints` | `right_fr3_arm_controller.joints` | [config/moveit_controllers_gazebo.yaml:25](../config/moveit_controllers_gazebo.yaml#L25) | `['right_fr3_joint1', 'right_fr3_joint2', 'right_fr3_joint3', 'right_fr3_joint4', 'right_fr3_joint5', 'right_fr3_joint6', 'right_fr3_joint7']` |
| `right_fr3_arm_controller.type` | `/**.controller_manager.ros__parameters.right_fr3_arm_controller.type` | [config/gazebo_ros2_controllers.yaml:14](../config/gazebo_ros2_controllers.yaml#L14) | `'joint_trajectory_controller/JointTrajectoryController'` |
| `right_fr3_arm_controller.type` | `right_fr3_arm_controller.type` | [config/moveit_controllers_gazebo.yaml:22](../config/moveit_controllers_gazebo.yaml#L22) | `'FollowJointTrajectory'` |
| `right_fr3_arm_controller.type` | `/**.controller_manager.ros__parameters.right_fr3_arm_controller.type` | [config/ros2_controllers.yaml:14](../config/ros2_controllers.yaml#L14) | `'joint_trajectory_controller/JointTrajectoryController'` |
| `right_franka_gripper.action_ns` | `right_franka_gripper.action_ns` | [config/moveit_controllers.yaml:42](../config/moveit_controllers.yaml#L42) | `'gripper_action'` |
| `right_franka_gripper.action_ns` | `right_franka_gripper.action_ns` | [config/moveit_controllers_gazebo.yaml:42](../config/moveit_controllers_gazebo.yaml#L42) | `'gripper_cmd'` |
| `right_franka_gripper.default` | `right_franka_gripper.default` | [config/moveit_controllers.yaml:44](../config/moveit_controllers.yaml#L44) | `True` |
| `right_franka_gripper.default` | `right_franka_gripper.default` | [config/moveit_controllers_gazebo.yaml:44](../config/moveit_controllers_gazebo.yaml#L44) | `True` |
| `right_franka_gripper.joints` | `right_franka_gripper.joints` | [config/moveit_controllers.yaml:46](../config/moveit_controllers.yaml#L46) | `['right_fr3_finger_joint1', 'right_fr3_finger_joint2']` |
| `right_franka_gripper.joints` | `right_franka_gripper.joints` | [config/moveit_controllers_gazebo.yaml:46](../config/moveit_controllers_gazebo.yaml#L46) | `['right_fr3_finger_joint1', 'right_fr3_finger_joint2']` |
| `right_franka_gripper.type` | `/**.controller_manager.ros__parameters.right_franka_gripper.type` | [config/gazebo_ros2_controllers.yaml:20](../config/gazebo_ros2_controllers.yaml#L20) | `'position_controllers/GripperActionController'` |
| `right_franka_gripper.type` | `right_franka_gripper.type` | [config/moveit_controllers.yaml:43](../config/moveit_controllers.yaml#L43) | `'GripperCommand'` |
| `right_franka_gripper.type` | `right_franka_gripper.type` | [config/moveit_controllers_gazebo.yaml:43](../config/moveit_controllers_gazebo.yaml#L43) | `'GripperCommand'` |
| `right_franka_robot_state_broadcaster.type` | `/**.controller_manager.ros__parameters.right_franka_robot_state_broadcaster.type` | [config/ros2_controllers.yaml:20](../config/ros2_controllers.yaml#L20) | `'franka_robot_state_broadcaster/FrankaRobotStateBroadcaster'` |
| `right_robot_ip` | `right_robot_ip` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `` |
| `right_robot_ip` | `right_robot_ip` | [launch/demo.launch.py:384](../launch/demo.launch.py#L384) | `''` |
| `robot_type` | `/**.left_franka_robot_state_broadcaster.ros__parameters.robot_type` | [config/ros2_controllers.yaml:72](../config/ros2_controllers.yaml#L72) | `'fr3'` |
| `robot_type` | `/**.right_franka_robot_state_broadcaster.ros__parameters.robot_type` | [config/ros2_controllers.yaml:78](../config/ros2_controllers.yaml#L78) | `'fr3'` |
| `rviz_config` | `rviz_config` | [launch/demo.launch.py:371](../launch/demo.launch.py#L371) | `''` |
| `rviz_config` | `rviz_config` | [launch/maniskill.launch.py:178](../launch/maniskill.launch.py#L178) | `''` |
| `simulation_backend` | `simulation_backend` | [launch/demo.launch.py:356](../launch/demo.launch.py#L356) | `DEFAULT_SIMULATION_BACKEND` |
| `stall_timeout` | `/**.left_franka_gripper.ros__parameters.stall_timeout` | [config/gazebo_ros2_controllers.yaml:64](../config/gazebo_ros2_controllers.yaml#L64) | `1.0` |
| `stall_timeout` | `/**.right_franka_gripper.ros__parameters.stall_timeout` | [config/gazebo_ros2_controllers.yaml:72](../config/gazebo_ros2_controllers.yaml#L72) | `1.0` |
| `stall_velocity_threshold` | `/**.left_franka_gripper.ros__parameters.stall_velocity_threshold` | [config/gazebo_ros2_controllers.yaml:63](../config/gazebo_ros2_controllers.yaml#L63) | `0.001` |
| `stall_velocity_threshold` | `/**.right_franka_gripper.ros__parameters.stall_velocity_threshold` | [config/gazebo_ros2_controllers.yaml:71](../config/gazebo_ros2_controllers.yaml#L71) | `0.001` |
| `start_gripper` | `start_gripper` | [launch/demo.launch.py:394](../launch/demo.launch.py#L394) | `'true'` |
| `state_interfaces` | `/**.left_fr3_arm_controller.ros__parameters.state_interfaces` | [config/gazebo_ros2_controllers.yaml:31](../config/gazebo_ros2_controllers.yaml#L31) | `['position', 'velocity']` |
| `state_interfaces` | `/**.right_fr3_arm_controller.ros__parameters.state_interfaces` | [config/gazebo_ros2_controllers.yaml:47](../config/gazebo_ros2_controllers.yaml#L47) | `['position', 'velocity']` |
| `state_interfaces` | `/**.left_fr3_arm_controller.ros__parameters.state_interfaces` | [config/ros2_controllers.yaml:27](../config/ros2_controllers.yaml#L27) | `['position', 'velocity']` |
| `state_interfaces` | `/**.right_fr3_arm_controller.ros__parameters.state_interfaces` | [config/ros2_controllers.yaml:51](../config/ros2_controllers.yaml#L51) | `['position', 'velocity']` |
| `thread_priority` | `/**.controller_manager.ros__parameters.thread_priority` | [config/ros2_controllers.yaml:5](../config/ros2_controllers.yaml#L5) | `98` |
| `trajectory_execution_duration_scaling` | `trajectory_execution_duration_scaling` | [launch/demo.launch.py:409](../launch/demo.launch.py#L409) | `PythonExpression(["'10.0' if '", backend, "' == 'maniskill' and '", LaunchConfiguration('maniskill_scene'), "' in ('usb_cable', 'trunking_cable') else '1.2'"])` |
| `trajectory_execution_duration_scaling` | `trajectory_execution_duration_scaling` | [launch/gazebo.launch.py:245](../launch/gazebo.launch.py#L245) | `'1.2'` |
| `trajectory_execution_duration_scaling` | `trajectory_execution_duration_scaling` | [launch/maniskill.launch.py:186](../launch/maniskill.launch.py#L186) | `'10.0'` |
| `trajectory_execution_goal_margin` | `trajectory_execution_goal_margin` | [launch/demo.launch.py:417](../launch/demo.launch.py#L417) | `PythonExpression(["'5.0' if '", backend, "' == 'maniskill' and '", LaunchConfiguration('maniskill_scene'), "' in ('usb_cable', 'trunking_cable') else '0.5'"])` |
| `trajectory_execution_goal_margin` | `trajectory_execution_goal_margin` | [launch/gazebo.launch.py:250](../launch/gazebo.launch.py#L250) | `'0.5'` |
| `trajectory_execution_goal_margin` | `trajectory_execution_goal_margin` | [launch/maniskill.launch.py:189](../launch/maniskill.launch.py#L189) | `'5.0'` |
| `update_rate` | `/**.controller_manager.ros__parameters.update_rate` | [config/gazebo_ros2_controllers.yaml:4](../config/gazebo_ros2_controllers.yaml#L4) | `250` |
| `update_rate` | `/**.controller_manager.ros__parameters.update_rate` | [config/ros2_controllers.yaml:4](../config/ros2_controllers.yaml#L4) | `1000` |
| `use_fake_hardware` | `use_fake_hardware` | [config/dual_fr3.urdf.xacro:1](../config/dual_fr3.urdf.xacro#L1) | `true` |
| `use_local_topics` | `/**.joint_state_broadcaster.ros__parameters.use_local_topics` | [config/gazebo_ros2_controllers.yaml:24](../config/gazebo_ros2_controllers.yaml#L24) | `True` |
| `use_rviz` | `use_rviz` | [launch/demo.launch.py:404](../launch/demo.launch.py#L404) | `'true'` |
| `use_rviz` | `use_rviz` | [launch/gazebo.launch.py:240](../launch/gazebo.launch.py#L240) | `'true'` |
| `use_rviz` | `use_rviz` | [launch/maniskill.launch.py:177](../launch/maniskill.launch.py#L177) | `'true'` |
| `use_sim_time` | `/**.controller_manager.ros__parameters.use_sim_time` | [config/gazebo_ros2_controllers.yaml:5](../config/gazebo_ros2_controllers.yaml#L5) | `True` |
| `use_sim_time` | `use_sim_time` | [launch/gazebo.launch.py:235](../launch/gazebo.launch.py#L235) | `'true'` |
