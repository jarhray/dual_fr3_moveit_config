# MoveIt 与控制器参数索引

本轮不修改模型、OMPL 数值或控制器设置。全部 YAML 叶字段、xacro arg 和直接 launch 参数逐项列出。各值及左右关节/控制器实例见 [默认值来源](parameter_defaults.md)。

消费代码：[moveit_resources.py](../dual_fr3_moveit_config/moveit_resources.py) 装配 URDF/SRDF、IK 和 OMPL；[demo.launch.py](../launch/demo.launch.py) 选择控制器及后端；[maniskill_resources.py](../dual_fr3_moveit_config/maniskill_resources.py) 装配物理资源。控制器参数由 launch 加载后交给对应 ROS 插件消费，包内没有另一份 PID/OMPL 算法。

验证：test/test_maniskill_scene_launch.py；模型变更比较生成 URDF/SRDF，规划变更运行整段 MTC 预检；控制器变更检查实际 action 名、跟踪误差和取消。禁止通过放宽碰撞/误差限制替代验证。

覆盖关系：入口显式参数 → 当前入口默认 → build_moveit_resources / YAML / xacro。MoveIt demo 默认 gazebo，MTC 上层明确传入 maniskill。ManiSkill 继承参数作用见 [物理参数](../../dual_fr3_maniskill/docs/parameters.md)，相机/SAM2 共用参数由 ros/launch.py 注入。

| 参数 | 单位 / 坐标 | 作用及关联 |
| --- | --- | --- |
| `/left/left_fr3_arm_controller.action_ns` | ROS action 后缀 | 与控制器名合成执行 action 路径，Gazebo/ManiSkill gripper_cmd 与真机映射不同。 |
| `/left/left_fr3_arm_controller.default` | 布尔 | MoveIt 为对应关节选择控制器时的默认候选。 |
| `/left/left_fr3_arm_controller.joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `/left/left_fr3_arm_controller.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `/right/right_fr3_arm_controller.action_ns` | ROS action 后缀 | 与控制器名合成执行 action 路径，Gazebo/ManiSkill gripper_cmd 与真机映射不同。 |
| `/right/right_fr3_arm_controller.default` | 布尔 | MoveIt 为对应关节选择控制器时的默认候选。 |
| `/right/right_fr3_arm_controller.joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `/right/right_fr3_arm_controller.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `allow_stalling` | 布尔 | 接触停滞是否作为夹爪 action 可接受结束；不证明物体抓持稳定。 |
| `arm_prefix` | 名称前缀 | 左右广播器资源前缀，避免关节/状态命名冲突。 |
| `arm_yaw` | rad，基座 Z | 双臂安装 yaw 的 xacro 输入，影响所有任务坐标变换和可达性。 |
| `cable_config` | 文件路径 | MoveIt/物理共享的普通场景 YAML；空值选简化 2 mm。 |
| `cable_solver` | mpm/rope_actor | 此包独立物理入口保留 mpm 缺省；MTC 上层传入 rope_actor。 |
| `cable_trace_dir` | 目录 | Rope 子步诊断，默认空；日志影响墙钟耗时。 |
| `capabilities` | 插件名串 | MoveGroup 额外 capabilities；MTC 入口加入执行 task solution 能力。 |
| `command_interfaces` | 接口名列表 | position 或 effort 命令接口，必须与 URDF 硬件后端一致。 |
| `controller_names` | 控制器名列表 | MoveIt 可选执行器，后续 type/action_ns/joints 定义其接口。 |
| `convenience_publish_rate` | Hz | Franka 状态广播器便捷话题发布频率，独立于控制循环。 |
| `disable_capabilities` | 插件名串 | 禁用指定 MoveGroup capabilities，可能令 MTC 执行接口不可用。 |
| `dual_fr3_arms.longest_valid_segment_fraction` | 状态空间最大距离比例 | OMPL 离散碰撞检查段长上限，越小检查更细且更慢；不是 MTC cartesian_step_size。 |
| `dual_fr3_arms.planner_configs` | 名称列表 | 该规划组允许的 OMPL 配置，需与 planner_configs 字典一致。 |
| `ee_id` | 模型标识 | 末端型号选择，决定 TCP/手指模型。 |
| `fake_sensor_commands` | 布尔 | mock 硬件传感器命令通道，不用于 ManiSkill 物理反馈。 |
| `gains.left_fr3_joint1.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint1.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint1.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint1.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.left_fr3_joint2.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint2.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint2.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint2.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.left_fr3_joint3.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint3.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint3.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint3.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.left_fr3_joint4.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint4.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint4.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint4.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.left_fr3_joint5.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint5.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint5.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint5.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.left_fr3_joint6.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint6.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint6.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint6.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.left_fr3_joint7.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.left_fr3_joint7.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.left_fr3_joint7.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.left_fr3_joint7.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint1.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint1.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint1.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint1.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint2.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint2.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint2.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint2.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint3.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint3.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint3.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint3.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint4.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint4.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint4.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint4.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint5.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint5.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint5.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint5.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint6.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint6.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint6.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint6.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gains.right_fr3_joint7.d` | N·m·s/rad | 该关节 effort PID 微分增益，速度误差转力矩。 |
| `gains.right_fr3_joint7.i` | N·m/(rad·s) | 该关节 effort PID 积分增益，0 禁用积分贡献。 |
| `gains.right_fr3_joint7.i_clamp` | N·m | 该关节 PID 积分输出限幅，关联 i 增益。 |
| `gains.right_fr3_joint7.p` | N·m/rad | 该关节 effort PID 比例增益，位置误差转力矩。 |
| `gazebo_effort` | 布尔 | Gazebo 使用力矩或位置控制接口，不修改 ManiSkill 控制方式。 |
| `goal_tolerance` | m，每指 | 夹爪控制器到位误差容差，独立于 ManiSkill bridge 同名含义。 |
| `gz_args` | 命令参数串 | Gazebo 启动世界与运行选项，仅 gazebo 后端使用。 |
| `insertion_enabled` | 布尔 | ManiSkill 接触夹持模式默认启用插座与插入前连续预检；显式值优先。 |
| `joint` | 关节名 | 夹爪控制器主指关节，另一手指由模型 mimic 对应。 |
| `joint_state_broadcaster.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `leader_orientation_direction` | forward/reverse | leader yaw 沿点索引正向或反向路径切线，初始 USB 朝向使用同一值。 |
| `left_fr3_arm.kinematics_solver` | plugin 名 | 该规划组的 IK 插件；本地配置使用 LMA。 |
| `left_fr3_arm.kinematics_solver_search_resolution` | rad，插件搜索分辨率 | 传给 IK 插件的搜索步长；实际使用由插件实现决定。 |
| `left_fr3_arm.kinematics_solver_timeout` | s | 规划组默认 IK 求解预算，调用方可另设超时。 |
| `left_fr3_arm.longest_valid_segment_fraction` | 状态空间最大距离比例 | OMPL 离散碰撞检查段长上限，越小检查更细且更慢；不是 MTC cartesian_step_size。 |
| `left_fr3_arm.planner_configs` | 名称列表 | 该规划组允许的 OMPL 配置，需与 planner_configs 字典一致。 |
| `left_fr3_arm_controller.action_ns` | ROS action 后缀 | 与控制器名合成执行 action 路径，Gazebo/ManiSkill gripper_cmd 与真机映射不同。 |
| `left_fr3_arm_controller.default` | 布尔 | MoveIt 为对应关节选择控制器时的默认候选。 |
| `left_fr3_arm_controller.joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `left_fr3_arm_controller.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `left_franka_gripper.action_ns` | ROS action 后缀 | 与控制器名合成执行 action 路径，Gazebo/ManiSkill gripper_cmd 与真机映射不同。 |
| `left_franka_gripper.default` | 布尔 | MoveIt 为对应关节选择控制器时的默认候选。 |
| `left_franka_gripper.joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `left_franka_gripper.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `left_franka_robot_state_broadcaster.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `left_robot_ip` | IP | 真机左臂连接地址，仿真不连接该机器人。 |
| `load_cable` | 布尔 | 默认 true；false 为 USB-only 调试，运输运动仍保留。 |
| `load_gripper` | 布尔 | 模型加载夹爪；接触夹持任务要求启用。 |
| `load_left_ros2_control` | 布尔 | 是否在模型中加入左臂 ros2_control 硬件定义。 |
| `load_right_ros2_control` | 布尔 | 是否在模型中加入右臂 ros2_control 硬件定义。 |
| `maniskill_config` | 文件路径 | 替代所选场景的 simulation YAML；机器人 100/500/50 Hz，线缆 50/500/25 Hz 不合并。 |
| `maniskill_python` | 解释器路径 | 物理 Python，优先环境 MANISKILL_PYTHON，否则工作目录 .venv/bin/python。 |
| `maniskill_scene` | 枚举 | robot / usb_cable / trunking_cable 选择唯一物理桥接及默认配置。 |
| `maniskill_viewer` | 布尔 | 物理查看器；MTC launch 默认 true。 |
| `planner_configs.PRMkConfigDefault.max_nearest_neighbors` | 个 | PRM 邻居连接数量，影响路网连通与建图成本。 |
| `planner_configs.PRMkConfigDefault.type` | 插件/算法标识 | OMPL 算法类型。 |
| `planner_configs.RRTConnectkConfigDefault.range` | 关节状态空间距离 | OMPL 树的单次扩展距离，0 交由 OMPL 自动选择。 |
| `planner_configs.RRTConnectkConfigDefault.type` | 插件/算法标识 | OMPL 算法类型。 |
| `planner_configs.RRTstarkConfigDefault.delay_collision_checking` | 0/1 | RRTstar 延迟碰撞检查开关，影响候选父节点检查顺序和耗时。 |
| `planner_configs.RRTstarkConfigDefault.goal_bias` | 0–1 概率 | RRTstar 对目标的采样偏置，影响搜索效率。 |
| `planner_configs.RRTstarkConfigDefault.range` | 关节状态空间距离 | OMPL 树的单次扩展距离，0 交由 OMPL 自动选择。 |
| `planner_configs.RRTstarkConfigDefault.type` | 插件/算法标识 | OMPL 算法类型。 |
| `right_fr3_arm.kinematics_solver` | plugin 名 | 该规划组的 IK 插件；本地配置使用 LMA。 |
| `right_fr3_arm.kinematics_solver_search_resolution` | rad，插件搜索分辨率 | 传给 IK 插件的搜索步长；实际使用由插件实现决定。 |
| `right_fr3_arm.kinematics_solver_timeout` | s | 规划组默认 IK 求解预算，调用方可另设超时。 |
| `right_fr3_arm.longest_valid_segment_fraction` | 状态空间最大距离比例 | OMPL 离散碰撞检查段长上限，越小检查更细且更慢；不是 MTC cartesian_step_size。 |
| `right_fr3_arm.planner_configs` | 名称列表 | 该规划组允许的 OMPL 配置，需与 planner_configs 字典一致。 |
| `right_fr3_arm_controller.action_ns` | ROS action 后缀 | 与控制器名合成执行 action 路径，Gazebo/ManiSkill gripper_cmd 与真机映射不同。 |
| `right_fr3_arm_controller.default` | 布尔 | MoveIt 为对应关节选择控制器时的默认候选。 |
| `right_fr3_arm_controller.joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `right_fr3_arm_controller.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `right_franka_gripper.action_ns` | ROS action 后缀 | 与控制器名合成执行 action 路径，Gazebo/ManiSkill gripper_cmd 与真机映射不同。 |
| `right_franka_gripper.default` | 布尔 | MoveIt 为对应关节选择控制器时的默认候选。 |
| `right_franka_gripper.joints` | 关节名列表 | 控制器管理的关节及顺序，需与规划组和硬件一致。 |
| `right_franka_gripper.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `right_franka_robot_state_broadcaster.type` | 插件/算法标识 | 对应控制器/action/广播器插件类型，须与硬件接口和 action 类型一致。 |
| `right_robot_ip` | IP | 真机右臂连接地址，与左侧分开配置。 |
| `robot_type` | 型号标识 | Franka 状态广播器机器人型号，用于状态接口解析。 |
| `rviz_config` | 配置路径 | RViz 布局与显示配置，不更改规划或物理。 |
| `simulation_backend` | 枚举 | 此包 demo 缺省 gazebo；MTC 推荐入口显式传入 maniskill，决定硬件插件、控制器和时钟。 |
| `stall_timeout` | s | 夹爪低速停滞保持时间，配合 allow_stalling 判断动作结果。 |
| `stall_velocity_threshold` | m/s，每指 | 夹爪停滞判定速度门限，与 stall_timeout 配套。 |
| `start_gripper` | 布尔 | 启动/检查夹爪控制接口，需和 load_gripper 配套。 |
| `state_interfaces` | 接口名列表 | 轨迹控制器读回的关节状态类型，如 position/velocity。 |
| `thread_priority` | 优先级 | 控制器实时线程调度优先级，需宿主支持。 |
| `trajectory_execution_duration_scaling` | 倍数 | MoveIt 允许执行时间相对规划时长的缩放，不改变仿真速度。 |
| `trajectory_execution_goal_margin` | s | MoveIt 执行期限的额外宽限，关联 duration_scaling。 |
| `update_rate` | Hz | ros2_control 控制器管理器更新频率；Gazebo 250、常规 1000，保留各后端配置。 |
| `use_fake_hardware` | 布尔 | xacro 选择模拟硬件插件；由 simulation_backend 映射，不控制 ManiSkill 物理。 |
| `use_local_topics` | 布尔 | joint_state_broadcaster 使用本地命名空间话题，关联 joint_states 汇聚/remap。 |
| `use_rviz` | 布尔 | 启动 RViz 窗口，仅影响显示和性能。 |
| `use_sim_time` | 布尔 | ROS 时间使用 /clock；输入和结果墙钟超时仍独立。 |

代码固定的规划流水线字段位于 moveit_resources.py：`planning_plugin=ompl_interface/OMPLPlanner`（规划插件）、`request_adapters=OMPL_REQUEST_ADAPTERS`（按列出的顺序处理请求及时间参数化）、`start_state_max_bounds_error=0.1`（起始关节越界修正容忍）、`path_tolerance=0.001`（m，时间参数化路径偏差）、`resample_dt=0.02`（s，轨迹重采样）。修改后检查完整轨迹碰撞、速度及执行误差。它们不是新增 launch 参数。
