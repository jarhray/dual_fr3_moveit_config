# 双 FR3 MoveIt 配置

`dual_fr3_moveit_config` 将两台 FR3、双夹爪、工作台、安装板和线槽放在同一机器人模型中，使用一个 `move_group` 统一规划和检查碰撞。支持 Gazebo、ManiSkill、虚拟硬件和真机。

本包负责机器人环境与运动接口。自动走线使用 [dual_fr3_trunking_mtc](../dual_fr3_trunking_mtc/README.md)，ManiSkill 物理与线缆实现见 [dual_fr3_maniskill](../dual_fr3_maniskill/README.md)。

## 构建

需要 ROS 2 Humble、MoveIt 2 和工作区的 Franka 依赖。Gazebo 后端还需要 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control` 及 `franka_gazebo_hardware`。在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dual_fr3_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

后续每个终端都需加载 ROS 和 `install/setup.bash`。ManiSkill 的额外依赖按[环境安装说明](../dual_fr3_maniskill/docs/setup.md)准备。

## 启动与使用

每次选择一个后端；以下命令都启动完整机器人环境和 RViz。

### 虚拟硬件：快速检查规划

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=fake
```

虚拟硬件可验证规划、控制接口和 RViz 状态更新，不提供物理接触仿真。

### Gazebo：刚体仿真

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=gazebo
```

`gazebo` 是默认后端，也可直接运行 `ros2 launch dual_fr3_moveit_config gazebo.launch.py`。

### ManiSkill：机器人或线缆仿真

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=maniskill maniskill_python:="$PWD/.venv/bin/python"
```

独立 USB 线缆演示：

```bash
ros2 launch dual_fr3_moveit_config usb_cable.launch.py \
  cable_solver:=rope_actor maniskill_python:="$PWD/.venv/bin/python"
```

USB 场景创建世界临时定位的动态插头；夹爪允许接触闭合与重新张开，通过 `/maniskill/usb/release` 和 `/maniskill/usb/verify` 检查实际抓持。追加 `load_cable:=false` 可完全跳过线缆后端。准备阶段夹持线缆并执行走线，应使用 [MTC 入口](../dual_fr3_trunking_mtc/README.md)。

`simulation_backend` 选择机器人环境；`cable_solver:=mpm`（默认）或 `cable_solver:=rope_actor` 选择线缆模型。
本次接触夹持验收使用 Rope-Actor 或 `load_cable:=false` 的 USB-only，MPM 后续完善；默认后端未更改。
两种模型及参数见[线缆后端文档](../dual_fr3_maniskill/docs/cable_backends.md)。

### 真机

将示例 IP 替换为实际机器人地址：

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=real \
  left_robot_ip:=192.168.1.2 right_robot_ip:=192.168.2.2
```

该命令连接驱动和 MoveIt；随后由 RViz 或控制脚本发出运动请求。夹爪测试见[夹爪接口说明](docs/gripper_action_test.md)。

### 在 RViz 中操作

在 `MotionPlanning` 面板选择规划组，拖动末端标记或选择目标状态，点击 **Plan** 检查轨迹，再点击 **Execute** 执行。RViz 中的机器人随对应后端的关节状态更新。

| 规划组 | 用途 |
| --- | --- |
| `left_fr3_arm` / `right_fr3_arm` | 左臂 / 右臂 |
| `dual_fr3_arms` | 双臂联合规划 |
| `left_fr3_hand` / `right_fr3_hand` | 左夹爪 / 右夹爪 |

Python 示例位于 `scripts/fr3_controller.py` 和 `scripts/fr3_controller_lin.py`，默认目标坐标系为 **`left_fr3_link0`**。导入方式、调用示例及笛卡尔回退行为见 [Python 控制接口](docs/python_control.md)。

## 常用参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `simulation_backend` | `gazebo` | `gazebo`、`maniskill`、`fake`、`real` 四选一 |
| `use_rviz` | `true` | 是否打开 RViz |
| `load_gripper` / `start_gripper` | 均为 `true` | 夹爪模型与驱动开关，具体适用范围见后端说明 |
| `left_robot_ip` / `right_robot_ip` | 空 | 真机地址 |
| `gz_args` | `empty.sdf -r` | Gazebo 世界和运行参数 |
| `maniskill_scene` | `robot` | ManiSkill 场景；独立线缆使用 `usb_cable` |
| `maniskill_viewer` | `true` | 是否打开 ManiSkill 窗口 |
| `maniskill_python` | `MANISKILL_PYTHON` 或当前目录 `.venv/bin/python` | 物理桥接使用的解释器 |
| `cable_config` / `maniskill_config` | 按场景选择 | 线缆配置 / 仿真桥接参数文件 |
| `load_cable` | `true` | `false` 仅加载动态 USB，不初始化线缆后端 |

参数可追加到启动命令，例如 `use_rviz:=false`。后端统一使用 `simulation_backend`，旧的 `use_gazebo`、`use_fake_hardware` 启动参数已移除。

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py --show-args
```

## 详细文档

- [后端、模型与排查](docs/backends.md)：坐标布局、控制器、障碍物和检查命令。
- [Python 控制接口](docs/python_control.md)：双臂目标、夹爪和直线运动。
- [夹爪接口说明](docs/gripper_action_test.md)：真机与仿真的 action 差异及测试。
- [ManiSkill 使用导航](docs/maniskill.md)：环境安装、场景和验证入口。
- [研究手指网格说明](docs/research_finger_mesh.md)：网格配准、TCP 和模型验证。

参数的作用、单位、消费文件及验证方法见 [参数索引](docs/parameters.md)；当前声明值和配置差异见 [默认值来源](docs/parameter_defaults.md)。
