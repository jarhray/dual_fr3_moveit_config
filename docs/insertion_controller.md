# FR3 插入关节目标执行器（P3 开发件）

本包的 `controllers` 模块提供 `dual_fr3_moveit_config/JointTargetController`，将上层 50 Hz
局部 IK 目标转换为 ros2_control effort 指令。算法仍为关节位置/速度跟踪；没有
笛卡尔阻抗搜孔，也不估计插入成功。未连接机器人测试，示例参数不代表已验证的接触限值。

## 配置和接管契约

[config/left_insertion_controller.example.yaml](../config/left_insertion_controller.example.yaml) 给出完整参数。三道激活门禁
`enabled`、`calibration_verified`、`runtime_limits_verified` 默认均为 false。
数组和限值必须显式配置，七个关节顺序必须与实际 `robot_state.q` 一致。
配置在 configure 时读取；更改任何增益/限值应先停稳、deactivate/cleanup/configure，
不能把运行中参数值的修改当作实际控制参数已更新。

使用现有控制器完成孔前接近并停稳后，通过 STRICT switch 将插入臂 JTC 切换到
本控制器。它独占该臂七个 effort 接口，并读取同一 hardware 的 `robot_state`
指针接口（例如 `left_fr3/robot_state`）。不支持把普通 mock 的数值字段冒充此指针。
切换前应取消旧轨迹；本包不管理 MoveIt 的任务所有权或碰撞检查。

激活仅允许低速、有效反馈、配置范围内的载荷和力矩。初始 q_ref 来自实测 q，
初始 effort 来自机器人 `tau_J_d`。每次激活生成新的 session，需从状态主题获取。
初始 `HOLDING` 有 `command_timeout_s` 的首包等待期。

## ROS 接口

- `~/target` (`JointTarget`)，reliable、keep-last 1。`stamp` 使用与控制器相同的 ROS
  时钟；`sequence` 从 1 开始严格递增，`session` 必须匹配激活会话；`valid_for_s`
  大于零且不超过 `command_timeout_s`。数组分别为 rad、rad/s。
- 首包必须发送**刚收到的实测 q 与零 dq**（位置容差为 `max_velocity * max_period_s`）；
  后续包同时检查位置变化率和 dq 的变化率。重复发送相同 q/零 dq 可保持 TRACKING。
  运行时实测 dq、跟踪误差和关节范围也受限。目标既检查 ROS stamp，又检查单调接收时间。
- `~/state` (`JointTargetState`)，默认 50 Hz：`positions/velocities` 是实测 q/dq；
  `last_sequence` 是 RT 核心已接受的序号。`HOLDING` 为首次等待，`TRACKING` 为接收目标，
  `STOPPING` 为锁存停止过程，`STOPPED` 表示测得低速并且内部参考已停稳至少
  `settle_time_s`。`STOPPED` 仍保持 effort 接管。扰动使其不再停稳时会回到 STOPPING。
- `~/stop` (`std_srvs/Trigger`) 请求锁存停止；成功响应仅确认请求入队，需继续等待
  `STOPPED`。stop、过期、非法目标、过载等均不会因新目标自动恢复，只能重新生命周期激活。
  本包不提供自动回退、自动重试或自动松爪。

`reason` 可能为 `none`、`cancelled`、`command_timeout`、`invalid_target`、
`wrong_session`、`nonmonotonic_sequence`、`invalid_timestamp`、`joint_limit`、
`velocity_limit`、`acceleration_limit`、`tracking_error`、`external_load_limit`、
`invalid_robot_state`、`stale_robot_state`、`invalid_control_period`、`torque_limit`、
`clock_discontinuity`。保留第一个故障原因，避免后来的超时覆盖原始原因。

## 力矩、停止和反馈边界

跟踪采用 `tau = kp * (q_ref - q) + kd * (dq_ref - dq)`，与工作区现有 effort JTC
相同的 PD 思路；不额外加入重力项（FCI 中机器人已处理重力补偿），也没有积分或模型补偿。
参考速度/加速度有界，力矩有幅值和变化率限制，变化率以实际 `tau_J_d` 为起点。
若硬件报告的旧力矩已超过配置幅值，控制器锁存并按变化率回落；此时不能同时保证立即回到
新幅值界内与力矩连续。激活时会拒绝这种初始状态。

停止捕获当时实测 q，丢弃上层前进目标；内部参考以受限速度/加速度返回捕获点，
保持 PD 控制和力矩连续。它不是卸力、回撤或急停，不能保证零停止位移；实际停止距离、
瞬态载荷以及保持增益必须先做无接触验证。不要通过 deactivate 放弃未知接触状态下的保持。
切回 JTC 必须明确后继控制器从当前测量姿态保持，且没有旧 trajectory 等待执行。

RT 直接读取 `q/dq/tau_J_d/O_F_ext_hat_K/time`。载荷检查用未扣基线的外力估计合力
和 K 参考点力矩的范数 (`max_force_norm_n/max_moment_norm_nm`)，与上层换算到孔口
的策略反馈相互独立。静态基线不能消除变化的线缆拉力，内部估计也不等于独立六维传感器。
设备时间不推进超过 `state_timeout_s`、倒退或非有限值会锁存；不可信反馈时不继续积分，
保留最后有限 effort，不宣称已停稳。此时必须依赖硬件自身的 FCI watchdog/reflex
及既有停机手段；本控制器不能在反馈丢失时证明物理停止。

RT 路径不调用 ROS 服务、不写常规日志，目标经 realtime buffer 传入，状态经 realtime
publisher 发布。字符串容量预分配。生命周期和订阅回调在非 RT 线程执行。

## 离线构建和测试

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --packages-select dual_fr3_moveit_config --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon test --packages-select dual_fr3_moveit_config --event-handlers console_direct+
```

测试验证纯 C++ guard 的首包握手、时间/session/序号、数据无效、载荷、速率限制、
超时不能恢复、取消保持和力矩连续。离线通过不代表控制器切换、实时延迟或接触行为已在实机通过。
