# Dual FR3 MoveIt 配置包

ManiSkill2 / SAPIEN 2 物理执行后端支持双臂、夹爪和 MTC，环境安装、构建与验证步骤见
[ManiSkill 使用说明](docs/maniskill.md)。
普通双臂与 USB 线缆场景共用 `maniskill.launch.py`，通过
`maniskill_scene:=robot|usb_cable` 选择；线缆物理实现位于 `dual_fr3_maniskill`。

这是一个基础的双臂 FR3 MoveIt 2 配置包。它把两台 FR3、两个 Franka hand
夹爪和一个固定工作台放在同一个 `robot_description` 里，并启动一个共享的
`move_group` 和规划场景。

这样做的重点是：左右臂在同一个 MoveIt planning scene 中规划，所以双臂之间、
机械臂与工作台之间、后续添加的障碍物与机械臂之间，都可以由同一个碰撞场景
统一处理。

## 场景布局

默认场景包含两台 FR3，安装在一张墨绿色工作台上。

- 台面坐标系就是 `world`
- 原点位于左上角
- `x` 轴为宽度方向，向前为正
- `y` 轴为长边方向，向右为正
- 台面尺寸：`0.75 x 1.5 x 0.05 m`
- 台面上表面：`z = 0`
- 台面实体：位于上表面下方

机械臂安装位置：

- 左臂安装点：`x = 0.175`，`y = 0.35`，`z = 0`
- 右臂安装点：`x = 0.175`，`y = 0.95`，`z = 0`
- 两台机械臂的坐标系方向和台面坐标系相同

工作台作为 URDF 中的固定 link 存在，并带有 collision geometry。工作台
collision 稍微低于可视化上表面，避免启动时和机械臂底座产生不必要的初始碰撞。

## 包结构

```text
dual_fr3_moveit_config/
  config/
    dual_fr3.urdf.xacro       # 双臂 FR3、双夹爪和工作台模型
    dual_fr3.srdf.xacro       # MoveIt 规划组、末端执行器和碰撞矩阵
    kinematics.yaml           # 左右臂运动学求解器
    moveit_controllers.yaml   # 真机/双 controller manager 的 MoveIt 映射
    moveit_controllers_gazebo.yaml
                              # Gazebo/单 controller manager 的 MoveIt 映射
    ompl_planning.yaml        # OMPL 规划配置
    ros2_controllers.yaml     # ros2_control 控制器配置
  launch/
    demo.launch.py            # MoveIt、RSP、ros2_control、夹爪和 RViz 启动文件
  scripts/
    fake_gripper_action_server.py
                                # fake hardware 下使用的夹爪 action server
    fr3_controller.py
                                # 双臂 OMPL Python 控制接口
    fr3_controller_lin.py
                                # 双臂笛卡尔直线 Python 控制接口
  rviz/
    dual_fr3.rviz             # RViz 配置
```

## MoveIt 规划组

SRDF 中定义了这些规划组：

- `left_fr3_arm`：左臂，从 `left_fr3_link0` 到 `left_fr3_hand_tcp`
- `right_fr3_arm`：右臂，从 `right_fr3_link0` 到 `right_fr3_hand_tcp`
- `dual_fr3_arms`：左右臂联合规划组
- `left_fr3_hand`：左夹爪 links 和 finger joints
- `right_fr3_hand`：右夹爪 links 和 finger joints

单臂规划时使用 `left_fr3_arm` 或 `right_fr3_arm`。

双臂协同规划时使用 `dual_fr3_arms`。这个组包含左右两条运动链，因此 MoveIt
可以在同一个规划请求里同时考虑左右臂关节，并进行双臂之间的碰撞检查。

## 双夹爪控制

自定义手指当前使用 `finger1.STL`；新版 CAD 导出坐标系的补偿、截面对比及回退方式
见 [手指 mesh 替换记录](docs/research_finger_mesh.md)。

MoveIt controller 映射中暴露了两个独立的 `GripperCommand` action：

- 左夹爪：`/left_franka_gripper/gripper_action`
- 右夹爪：`/right_franka_gripper/gripper_action`

两个夹爪使用不同节点名和不同 action 路径，避免左右夹爪订阅到同一个 action
endpoint 后发生命令冲突。

在 fake hardware 模式下，本包启动两个轻量级 fake gripper action server，
用于 RViz / MoveIt 中测试夹爪执行。

在真实硬件模式下，本包分别启动两个官方 `franka_gripper_node`：

- `left_franka_gripper`：连接 `left_robot_ip`
- `right_franka_gripper`：连接 `right_robot_ip`

夹爪 joint states 会和机械臂 joint states 一起合并到 `/joint_states`，状态链路
与 Franka 官方单臂 MoveIt 启动方式保持一致。

Franka hand 中，`GripperCommand.command.position` 对应单个 finger joint 位置。
物理开口宽度大约是该值的两倍。

## Python 控制接口

本包新增了两个可直接调用的 Python 控制脚本：

- `scripts/fr3_controller.py`
- `scripts/fr3_controller_lin.py`

它们的使用方式和 `src/my_controller/fr3_controller.py`、
`src/my_controller/fr3_controller_lin.py` 很接近，只是多了一个 `arm` 参数。
支持的 `arm` 值是：

- `left`
- `right`

### 坐标系约定

场景构建时使用台面坐标系 `world`。Python 控制接口中，默认目标坐标系是左臂
基坐标系：

```text
default_frame = "left_fr3_link0"
```

因此直接调用 `move_to(...)` 时，`x/y/z` 默认表示相对于左臂基座的坐标。
由于左右臂坐标系方向一致，右臂在左臂坐标系下约位于：

```text
x = 0.0
y = 0.6
z = 0.0
```

如果希望使用台面坐标系，显式传入：

```python
frame_id="world"
```

如果希望使用某一台机械臂自己的基坐标系，显式传入：

```python
frame_id="left_fr3_link0"
frame_id="right_fr3_link0"
```

### `fr3_controller.py`

该文件提供 OMPL 点到点规划接口。

类名：

```python
DualFR3Controller
```

构造函数：

```python
DualFR3Controller(
    node_name: str = "dual_fr3_action_controller",
    default_frame: str = "left_fr3_link0",
)
```

常用接口：

```python
move_to(arm, x, y, z, roll=pi, pitch=0.0, yaw=0.0, execute=True, frame_id=None) -> bool
move_left_to(x, y, z, roll=pi, pitch=0.0, yaw=0.0, execute=True, frame_id=None) -> bool
move_right_to(x, y, z, roll=pi, pitch=0.0, yaw=0.0, execute=True, frame_id=None) -> bool
```

控制单臂 TCP 到目标位姿。`frame_id=None` 时使用 `default_frame`。

```python
move_dual_to(left_pose, right_pose, execute=True, frame_id=None) -> bool
```

使用 `dual_fr3_arms` 规划组做双臂协同规划。`left_pose` 和 `right_pose` 格式为：

```text
[x, y, z, roll, pitch, yaw]
```

```python
reset_arm(arm, execute=True) -> bool
reset_both_arms(execute=True) -> bool
reset(execute=True, reset_grippers=True) -> bool
```

复位单臂、双臂，或同时复位双臂和双夹爪。

```python
rotate(arm, z, execute=True, relative=True, abs_tol_fixed=0.002, abs_tol_j7=0.002) -> bool
rotate_to_home(arm, execute=True) -> bool
```

旋转指定机械臂的第 7 轴。`relative=True` 时 `z` 是增量角，`relative=False` 时
`z` 是绝对目标角。

```python
open_gripper(arm, width=0.08, speed=0.08) -> bool
close_gripper(arm, width=0.0, speed=0.08) -> bool
set_gripper_action(arm, grip_action) -> bool
grasp_object(arm, width=0.02, speed=0.05, force=50.0,
             inner_tolerance=0.005, outer_tolerance=0.01) -> bool
get_gripper_state(arm) -> dict
print_gripper_state(arm)
```

`grip_action=0.0` 表示完全闭合，`grip_action=1.0` 表示完全打开。真实硬件下
优先使用官方 `franka_gripper` 的 `/move`、`/grasp`、`/homing`；fake hardware
下会自动使用 `/gripper_action`。

### `fr3_controller_lin.py`

该文件提供笛卡尔直线规划接口，并继承 `DualFR3Controller` 的大部分方法。

类名：

```python
DualFR3LinearController
```

构造函数：

```python
DualFR3LinearController(
    node_name: str = "dual_fr3_linear_controller",
    default_frame: str = "left_fr3_link0",
    use_cartesian_lin_control: bool = True,
)
```

额外接口：

```python
move_to(arm, x, y, z, roll=pi, pitch=0.0, yaw=0.0, execute=True, frame_id=None) -> bool
move_to_cartesian(arm, x, y, z, roll=pi, pitch=0.0, yaw=0.0,
                  execute=True, frame_id=None, max_step=0.005,
                  min_fraction=0.9) -> bool
move_to_ompl(arm, x, y, z, roll=pi, pitch=0.0, yaw=0.0,
             execute=True, frame_id=None) -> bool
```

`move_to()` 默认调用 `move_to_cartesian()`；如果笛卡尔路径比例低于
`min_fraction`，会回退到 `move_to_ompl()`。

```python
get_current_pose(arm)
get_current_pose_rpy(arm)
move_dual_to_cartesian(left_pose, right_pose, execute=True, frame_id=None) -> bool
```

`get_current_pose*` 返回指定机械臂 TCP 在 `default_frame` 下的当前位姿。
当前没有真正的双臂同步笛卡尔直线插补，`move_dual_to_cartesian()` 会回退到
`move_dual_to()`，即使用 `dual_fr3_arms` 做双臂 OMPL 协同规划。

### 调用示例

先启动双臂 MoveIt：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py
```

再运行控制脚本：

```bash
python3 src/dual_fr3_moveit_config/scripts/fr3_controller.py
python3 src/dual_fr3_moveit_config/scripts/fr3_controller_lin.py
```

在自己的 Python 脚本中调用：

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from fr3_controller_lin import DualFR3LinearController

rclpy.init()
controller = DualFR3LinearController()
executor = MultiThreadedExecutor()
executor.add_node(controller)

# 默认使用 left_fr3_link0 坐标系
controller.move_to("left", 0.4, 0.0, 0.4)
controller.move_to("right", 0.4, 0.6, 0.4)

# 显式使用台面坐标系 world
controller.move_to("left", 0.35, 0.35, 0.4, frame_id="world")
controller.move_to("right", 0.35, 0.95, 0.4, frame_id="world")

controller.open_gripper("left")
controller.close_gripper("right")

controller.destroy_node()
rclpy.shutdown()
```

如果你的脚本不在 `scripts/` 目录下，需要把
`src/dual_fr3_moveit_config/scripts` 加入 `PYTHONPATH`，或者直接把你的脚本也放在
同一个目录下运行。

## 构建

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_fr3_moveit_config
source install/setup.bash
```

如果 Franka 相关包还没有构建过，建议先构建整个工作空间，确保
`franka_description`、`franka_hardware` 和 `franka_gripper` 可用。

如果需要使用 Gazebo 仿真，还需要确保 `franka_gazebo_hardware` 也已经构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to franka_gazebo_hardware dual_fr3_moveit_config
source install/setup.bash
```

## 运行 Fake Hardware Demo

显式选择 `fake`（省略后端参数时默认启动 Gazebo）：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=fake
```

不启动 RViz：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=fake use_rviz:=false
```

默认启动内容：

- `robot_state_publisher`
- `joint_state_publisher`
- 一个 `move_group`
- 左右各一个 `ros2_control_node`
- `joint_state_broadcaster`
- `left_fr3_arm_controller`
- `right_fr3_arm_controller`
- fake hardware 下的左右夹爪 action server
- RViz，除非设置 `use_rviz:=false`

## 运行 Gazebo 仿真

默认后端是 Gazebo，可直接使用统一入口（也可保留使用专用的 `gazebo.launch.py`）：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py
```

不启动 RViz：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=gazebo use_rviz:=false
```

传递 Gazebo 参数：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=gazebo gz_args:="empty.sdf -r"
```

Gazebo 模式启动内容：

- `ros_gz_sim`
- `ros_gz_bridge`（将 Gazebo 的 `/clock` 接入 ROS）
- `robot_state_publisher`
- 将双臂 FR3 + 工作台模型 spawn 到 Gazebo
- Gazebo 内部的 `gz_ros2_control`
- `joint_state_broadcaster`
- `left_fr3_arm_controller`
- `right_fr3_arm_controller`
- 一个共享 `move_group`
- `left_franka_gripper`
- `right_franka_gripper`
- RViz，除非设置 `use_rviz:=false`

Gazebo 启动默认启用 `use_sim_time`，因此 MoveIt 的轨迹超时按照仿真时钟计算；这对
复杂模型导致仿真频率低于实时的情况是必要的。

Gazebo 模式不会单独启动 `ros2_control_node`。控制器管理器由 Gazebo 模型里的
`gz_ros2_control` 插件创建。

Gazebo 使用的主要配置文件：

- `config/dual_fr3.gazebo.urdf.xacro`
- `config/gazebo_ros2_controllers.yaml`
- `config/moveit_controllers_gazebo.yaml`
- `launch/gazebo.launch.py`

当前 Gazebo 配置默认使用原生 `gz_ros2_control/GazeboSimSystem` 和 position
trajectory controller，以便 MoveIt 的轨迹直接写入 Gazebo 关节：

```text
command_interfaces: position
```

如需运行官方 effort/model-based 示例控制器，可传入 `gazebo_effort:=true`；当前
MoveIt trajectory controller 仍使用 position 接口。

工作台、安装板、线槽和手指保留原始 visual mesh，但 collision 使用简化 box。
不要把高面数 visual STL 直接用作 Gazebo collision，否则物理更新频率会从
250 Hz 降到个位数，表现为控制器接收轨迹但机械臂几乎不动。

仿真控制器名称为：

```text
left_fr3_arm_controller
right_fr3_arm_controller
left_franka_gripper
right_franka_gripper
```

因此现有 Python 控制脚本可以继续通过 MoveIt 控制仿真双臂：

```bash
python3 src/dual_fr3_moveit_config/scripts/fr3_controller.py
python3 src/dual_fr3_moveit_config/scripts/fr3_controller_lin.py
```

Gazebo 模式下两侧 `position_controllers/GripperActionController` 直接控制
`finger_joint1`，另一个 finger joint 通过 URDF mimic 关系同步。夹爪 action 为：

```text
/left_franka_gripper/gripper_cmd
/right_franka_gripper/gripper_cmd
```

真机仍使用 `/left_franka_gripper/gripper_action` 和
`/right_franka_gripper/gripper_action`；两种后端由各自的 MoveIt controller YAML
明确分流。

## Launch 参数

`demo.launch.py` 使用 `simulation_backend` 统一选择后端，默认 `gazebo`。
支持 `gazebo`（Gazebo）、`maniskill`（ManiSkill）、`fake`（mock hardware）、
`real`（真实硬件）。原 `use_gazebo`、`use_fake_hardware` 启动参数已移除。

```text
simulation_backend:=gazebo
fake_sensor_commands:=false
left_robot_ip:=""
right_robot_ip:=""
load_gripper:=true
ee_id:=franka_hand
use_rviz:=true
```

## 连接真实硬件

真实硬件模式示例：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=real \
  left_robot_ip:=<left_fr3_ip> \
  right_robot_ip:=<right_fr3_ip>
```

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=real \
  left_robot_ip:=192.168.1.2 \
  right_robot_ip:=192.168.2.2
```

真实硬件路径中，URDF 里有两个独立的
`franka_hardware/FrankaHardwareInterface` 实例：

- `left_FrankaHardwareInterface`
- `right_FrankaHardwareInterface`

对应的 arm controller：

- `left_fr3_arm_controller`
- `right_fr3_arm_controller`

对应的 Franka robot state broadcaster：

- `left_franka_robot_state_broadcaster`
- `right_franka_robot_state_broadcaster`

对应的夹爪节点：

- `left_franka_gripper`
- `right_franka_gripper`

在真实机械臂上运动前，请先确认网络、机器人安全状态、控制器状态、急停状态和
recovery 流程都正常。

## 添加静态障碍物

固定障碍物可以直接添加到 `config/dual_fr3.urdf.xacro` 中。优先使用简单
collision geometry，例如 box、cylinder、sphere。

示例：

```xml
<link name="fixture">
  <visual>
    <origin xyz="0.4 0.0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.1 0.2"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0.4 0.0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.1 0.2"/>
    </geometry>
  </collision>
</link>

<joint name="worktable_to_fixture" type="fixed">
  <parent link="worktable"/>
  <child link="fixture"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

如果障碍物来自 mesh 文件，可以放到包内，例如：

```text
meshes/obstacles/my_fixture.stl
```

然后用 package URI 引用：

```xml
<mesh filename="package://dual_fr3_moveit_config/meshes/obstacles/my_fixture.stl"/>
```

用于规划的 collision mesh 建议尽量简化，不要直接使用过密的视觉 mesh。

## 添加动态障碍物

如果障碍物会在运行时出现、移动或消失，建议不要写死在 URDF 里，而是发布到
MoveIt planning scene。

常见方式：

- C++：使用 `planning_scene_interface.addCollisionObjects(...)`
- Python：使用当前 MoveIt 版本支持的 planning scene API
- ROS 接口：向 `move_group` 暴露的 planning scene 相关 topic/service 发布

无论使用哪种方式，障碍物 frame 建议统一使用 `world` 或 `worktable`。

## 验证命令

展开 URDF：

```bash
xacro src/dual_fr3_moveit_config/config/dual_fr3.urdf.xacro > /tmp/dual_fr3.urdf
check_urdf /tmp/dual_fr3.urdf
```

展开 SRDF：

```bash
xacro src/dual_fr3_moveit_config/config/dual_fr3.srdf.xacro > /tmp/dual_fr3.srdf
```

查看 launch 参数：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py --show-args
```

查看 Gazebo launch 参数：

```bash
ros2 launch dual_fr3_moveit_config gazebo.launch.py --show-args
```

检查 Gazebo 的两个夹爪 action 是否存在：

```bash
ros2 action list | grep gripper_cmd
```

期望看到：

```text
/left_franka_gripper/gripper_cmd
/right_franka_gripper/gripper_cmd
```

检查夹爪 joint states 是否合并进 `/joint_states`：

```bash
ros2 topic echo /joint_states --once
```

消息中应该能看到：

```text
left_fr3_finger_joint1
left_fr3_finger_joint2
right_fr3_finger_joint1
right_fr3_finger_joint2
```

直接测试左夹爪：

```bash
ros2 action send_goal /left_franka_gripper/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.02, max_effort: 20.0}}"
```

直接测试右夹爪：

```bash
ros2 action send_goal /right_franka_gripper/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.02, max_effort: 20.0}}"
```

检查 ros2_control 硬件组件：

```bash
ros2 control list_hardware_components
```

检查 ros2_control 控制器：

```bash
ros2 control list_controllers
```

Gazebo 模式下，期望至少看到：

```text
joint_state_broadcaster active
left_fr3_arm_controller active
right_fr3_arm_controller active
left_franka_gripper active
right_franka_gripper active
```

## 注意事项

- 本包故意只启动一个 `move_group`，用于统一维护双臂 planning scene。
- 左右臂之间的碰撞检查保持开启。
- 工作台会参与和运动 links 的碰撞检查。
- 工作台只对安装底座附近的固定/底座 links 放开碰撞。
- `load_gripper:=true` 时默认加载夹爪几何和夹爪规划组。
- fake hardware 模式使用本包内的 fake gripper action server。
- 真实硬件模式使用官方 `franka_gripper_node` 分别控制左右夹爪。
- Gazebo 模式使用独立的 `gazebo.launch.py`，不要和真实硬件 launch 混用。
