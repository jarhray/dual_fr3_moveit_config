# 双夹爪接口与测试

两侧夹爪使用独立节点和 action 名称。本文示例以左侧为主，右侧将路径中的 `left` 改为 `right`。运行前按 [README](../README.md) 启动所需后端，每个终端加载 ROS 与工作区。

## 接口与宽度

| 后端 | 标准夹爪 action | 额外接口 |
| --- | --- | --- |
| `real` | `/left_franka_gripper/gripper_action` | `/homing`、`/move`、`/grasp` action 及 `/stop` 服务，均在夹爪节点名下 |
| `fake` | `/left_franka_gripper/gripper_action` | 轻量虚拟动作服务，无真实回零和接触 |
| `gazebo` / `maniskill` | `/left_franka_gripper/gripper_cmd` | 由对应仿真执行 |

`franka_msgs/action/Move`、`Grasp` 的 `width` 是**总开口**；`control_msgs/action/GripperCommand` 的 `position` 是**单个手指位移**，总开口约为两倍。比如 `position=0.02` 对应总开口约 0.04 m。

```bash
ros2 action list -t
ros2 topic echo /joint_states --once
```

状态中应包含 `left_fr3_finger_joint1/2` 与 `right_fr3_finger_joint1/2`。接口未出现时先查看启动日志，确认驱动或控制器正常。

## 真机回零与开合

真机启动示例，替换为实际 IP：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=real \
  left_robot_ip:=192.168.1.2 right_robot_ip:=192.168.2.2 \
  load_gripper:=true start_gripper:=true
```

Homing 会寻找机械行程，执行前夹爪内部应为空。先逐侧回零，再做低速开合：

```bash
ros2 action send_goal /left_franka_gripper/homing franka_msgs/action/Homing '{}' --feedback
ros2 action send_goal /right_franka_gripper/homing franka_msgs/action/Homing '{}' --feedback

ros2 action send_goal /left_franka_gripper/move franka_msgs/action/Move \
  '{width: 0.04, speed: 0.02}' --feedback
```

`width` 不能超过实测最大开口。返回结果需检查 `success` 与错误文本；action 目标被接收不等于运动成功。

## 真机抓取

以下是参数格式示例，总宽度与夹持力应按实际工件调整：

```bash
ros2 action send_goal /left_franka_gripper/grasp franka_msgs/action/Grasp \
  '{width: 0.02, speed: 0.02, force: 10.0, epsilon: {inner: 0.003, outer: 0.005}}' \
  --feedback
```

`epsilon` 决定抓取宽度的允许区间。空夹爪或尺寸不匹配时，夹爪即使运动也可能返回抓取失败；不要据此直接增大夹持力。

## 标准 GripperCommand

真机或虚拟硬件：

```bash
ros2 action send_goal /left_franka_gripper/gripper_action \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.02, max_effort: 10.0}}' --feedback
```

Gazebo 或普通 ManiSkill 机器人场景：

```bash
ros2 action send_goal /left_franka_gripper/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.02, max_effort: 10.0}}' --feedback
```

独立 USB 场景固定左夹爪，拒绝其动作；MTC 线缆激活后也不允许打开任一夹爪。此时被拒绝是场景约束，见 [MTC 线缆说明](../../dual_fr3_maniskill/docs/mtc_cable.md)。

## 停止与排查

真机夹爪停止服务：

```bash
ros2 service call /left_franka_gripper/stop std_srvs/srv/Trigger '{}'
```

它只停止夹爪，不是机械臂急停。

| 现象 | 检查方向 |
| --- | --- |
| 没有夹爪 action | 后端路径、`load_gripper` / `start_gripper`、IP 和驱动日志 |
| `Goal rejected` | 同侧是否有未结束目标，或是否处于固定线缆场景 |
| `Commanding out of range width` | 是否完成 Homing，是否混淆单指位移和总宽度 |
| 抓取运动完成但返回失败 | 实际物体尺寸、目标宽度和 epsilon |

MTC 的 profile 参数及 Homing 开关见[任务配置](../../dual_fr3_trunking_mtc/docs/configuration.md)和[执行说明](../../dual_fr3_trunking_mtc/docs/execution.md)。
