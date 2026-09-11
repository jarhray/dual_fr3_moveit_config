# 双 FR3 夹爪 Action 测试指南

本文档用于通过 ROS 2 Action 分别测试左右 Franka Hand 的回零、开合和抓取功能，
同时给出 MoveIt 使用的标准 `GripperCommand` 接口和紧急停止方法。

## 1. 安全要求

测试前请确认：

- 夹爪、机械臂周围没有人员和无关物体。
- 执行 Homing 时，夹爪内部没有物体。
- 已准备好急停，并能随时在 Desk 中停止机器人。
- 首次测试使用较低速度和较低抓取力。
- 两台机器人的 Desk 均已清除错误、解锁机器人并启用 FCI。

如果启动日志出现以下错误，必须先在 Desk 中启用 FCI，Action Server 才会存在：

```text
Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.
```

## 2. 启动真机和夹爪节点

在终端 A 中执行：

```bash
cd ~/ws_franka
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=real \
  left_robot_ip:=192.168.1.2 \
  right_robot_ip:=192.168.2.2 \
  load_gripper:=true \
  start_gripper:=true
```

其中：

- `load_gripper:=true` 将夹爪几何模型加入机器人描述。
- `start_gripper:=true` 启动左右两个真实夹爪驱动节点。

保持终端 A 运行。打开终端 B，并加载工作区：

```bash
cd ~/ws_franka
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. 检查 Action Server

```bash
ros2 action list -t | grep franka_gripper
```

正常应至少看到：

```text
/left_franka_gripper/homing [franka_msgs/action/Homing]
/left_franka_gripper/move [franka_msgs/action/Move]
/left_franka_gripper/grasp [franka_msgs/action/Grasp]
/left_franka_gripper/gripper_action [control_msgs/action/GripperCommand]
/right_franka_gripper/homing [franka_msgs/action/Homing]
/right_franka_gripper/move [franka_msgs/action/Move]
/right_franka_gripper/grasp [franka_msgs/action/Grasp]
/right_franka_gripper/gripper_action [control_msgs/action/GripperCommand]
```

也可以检查节点：

```bash
ros2 node list | grep franka_gripper
```

预期结果：

```text
/left_franka_gripper
/right_franka_gripper
```

如果这些接口不存在，先检查终端 A 中的夹爪连接错误，不要继续发送目标。

## 4. Homing 测试

Homing 会寻找夹爪的机械行程并确定最大开口。执行前必须移除夹爪中的物体。

左夹爪：

```bash
ros2 action send_goal \
  /left_franka_gripper/homing \
  franka_msgs/action/Homing \
  "{}" \
  --feedback
```

右夹爪：

```bash
ros2 action send_goal \
  /right_franka_gripper/homing \
  franka_msgs/action/Homing \
  "{}" \
  --feedback
```

预期结果中应包含：

```text
Goal accepted
success: true
```

首次上电、夹爪状态不确定或夹爪运动范围异常时，应重新执行 Homing。

## 5. Move 开合测试

`franka_msgs/action/Move` 适合测试无物体时的夹爪开合：

- `width`：两个手指之间的总开口宽度，单位为米。
- `speed`：夹爪运动速度，单位为米每秒。
- Franka Hand 的常用总开口范围约为 `0.0 ~ 0.08 m`。

### 5.1 左夹爪

张开到 8 cm：

```bash
ros2 action send_goal \
  /left_franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.08, speed: 0.03}" \
  --feedback
```

闭合到 4 cm：

```bash
ros2 action send_goal \
  /left_franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.04, speed: 0.03}" \
  --feedback
```

### 5.2 右夹爪

张开到 8 cm：

```bash
ros2 action send_goal \
  /right_franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.08, speed: 0.03}" \
  --feedback
```

闭合到 4 cm：

```bash
ros2 action send_goal \
  /right_franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.04, speed: 0.03}" \
  --feedback
```

建议先逐个测试夹爪。两个夹爪单独工作正常后，可以在两个终端中分别发送左右
`Move` 目标，检查它们能否同时运动。

## 6. Grasp 抓取测试

`franka_msgs/action/Grasp` 用于抓取真实物体。它不仅执行闭合，还会根据最终宽度
和容差判断是否抓取成功。

参数说明：

- `width`：成功抓住物体时预期的总开口宽度，单位为米。
- `speed`：夹爪闭合速度，单位为米每秒。
- `force`：最大抓取力，单位为牛顿。
- `epsilon.inner`：允许最终宽度小于目标宽度的误差。
- `epsilon.outer`：允许最终宽度大于目标宽度的误差。

首次测试建议使用尺寸已知、表面稳定的物体，并从较低抓取力开始。

左夹爪抓取目标宽度约 3 cm 的物体：

```bash
ros2 action send_goal \
  /left_franka_gripper/grasp \
  franka_msgs/action/Grasp \
  "{width: 0.03, speed: 0.03, force: 20.0, epsilon: {inner: 0.005, outer: 0.005}}" \
  --feedback
```

右夹爪：

```bash
ros2 action send_goal \
  /right_franka_gripper/grasp \
  franka_msgs/action/Grasp \
  "{width: 0.03, speed: 0.03, force: 20.0, epsilon: {inner: 0.005, outer: 0.005}}" \
  --feedback
```

夹爪发生闭合但 Action 返回 `success: false`，通常表示最终宽度不在指定容差内，
不一定代表通信或驱动故障。应检查物体宽度以及 `width`、`epsilon` 设置。

## 7. MoveIt 标准 GripperCommand 测试

MoveIt 通过以下接口控制两个夹爪：

```text
/left_franka_gripper/gripper_action
/right_franka_gripper/gripper_action
```

接口类型为：

```text
control_msgs/action/GripperCommand
```

本驱动中的 `command.position` 是单个手指关节的位置。驱动会将它乘以 2，转换为
两个手指之间的总开口宽度。因此：

```text
position = 0.04 m  -> 总开口约 0.08 m
position = 0.02 m  -> 总开口约 0.04 m
position = 0.01 m  -> 总开口约 0.02 m
```

打开左夹爪：

```bash
ros2 action send_goal \
  /left_franka_gripper/gripper_action \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 20.0}}" \
  --feedback
```

打开右夹爪：

```bash
ros2 action send_goal \
  /right_franka_gripper/gripper_action \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 20.0}}" \
  --feedback
```

抓取目标总宽度约 2 cm 的物体：

```bash
ros2 action send_goal \
  /left_franka_gripper/gripper_action \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.01, max_effort: 20.0}}" \
  --feedback
```

注意：当目标宽度小于当前宽度时，当前驱动会把 `GripperCommand` 转换为抓取操作，
因此无物体时可能返回抓取失败。无物体的纯开合测试应优先使用 `Move` Action。

## 8. 停止夹爪

当前 `franka_gripper_node` 的停止接口是 Service，不是 Action。

停止左夹爪：

```bash
ros2 service call \
  /left_franka_gripper/stop \
  std_srvs/srv/Trigger \
  "{}"
```

停止右夹爪：

```bash
ros2 service call \
  /right_franka_gripper/stop \
  std_srvs/srv/Trigger \
  "{}"
```

该 Service 只停止夹爪运动，不等同于机械臂急停。异常情况下仍应优先使用实体急停
或 Desk 的停止功能。

## 9. 推荐测试顺序

1. 检查两侧夹爪节点和 Action Server 是否存在。
2. 左夹爪 Homing。
3. 左夹爪执行 `Move: 0.08 m -> 0.04 m -> 0.08 m`。
4. 右夹爪 Homing。
5. 右夹爪执行 `Move: 0.08 m -> 0.04 m -> 0.08 m`。
6. 在两个终端中同时发送左右 `Move`，测试双夹爪并行运动。
7. 放入尺寸已知的测试物体，分别执行左右 `Grasp`。
8. 最后测试 MoveIt 使用的 `GripperCommand` 接口。

## 10. 常见问题

### Action 列表中没有夹爪接口

检查启动终端是否出现 FCI 拒绝连接、IP 地址错误或夹爪节点退出。确认启动参数为：

```text
start_gripper:=true
```

### `Goal rejected` 或前一个目标还在运行

等待前一个目标完成，或调用对应夹爪的 `/stop` Service 后再重新发送。

### `Commanding out of range width`

检查宽度范围。对 `Move`/`Grasp`，`width` 是总开口；对 `GripperCommand`，
`position` 是单侧手指关节位置。

### 夹爪运动了但 Grasp 返回失败

检查物体实际宽度、目标 `width` 和 `epsilon`。纯闭合运动测试请改用 `Move`。

### 只测试夹爪时机械臂通信错误导致整个 launch 退出

当前真机 launch 会在任一机械臂控制管理器退出时停止全部节点。这种情况下先解决
机械臂的 `communication_constraints_violation`，或者单独启动夹爪节点做隔离测试。

## 11. 相关配置和实现

- MoveIt 夹爪映射：`config/moveit_controllers.yaml`
- 双夹爪启动配置：`launch/demo.launch.py`
- 夹爪 Action 客户端示例：`scripts/fr3_controller.py`
- Franka 夹爪 Action Server：`../../franka_gripper/src/gripper_action_server.cpp`
