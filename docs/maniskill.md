# ManiSkill 双 FR3 执行后端

本后端保留 MoveIt 2、MTC、SRDF、任务关键点和自定义手指，使用 ManiSkill2 / SAPIEN 2
执行物理仿真。桥接节点接收 ROS action，按仿真时间采样轨迹，通过 PD 控制执行，
再把实际关节位置和速度发布给 MoveIt 与 robot_state_publisher。RViz 显示的是仿真反馈。

## 环境与构建

本工作区的 `.venv` 使用 Python 3.10，依赖锁定在
`dual_fr3_maniskill/requirements-maniskill2.txt`：ManiSkill2 0.5.3、SAPIEN 2.2.2、
NumPy 1.23.5、SciPy 1.10.1、Gymnasium 0.29.1、Trimesh 3.23.5。
导入名是 `mani_skill2` 和 `sapien.core`，原 ManiSkill3 的 `mani_skill` 接口不再使用。

安装或修复环境：

```bash
cd /home/jerry/franka_ros2_ws
bash src/dual_fr3_maniskill/scripts/setup_maniskill2.sh
```

脚本在切换旧环境时保留 `.venv.maniskill3-backup-时间戳`，创建不读取用户级
`~/.local` 包的独立 `.venv`，下载固定版本源码到 `.deps/ManiSkill-0.5.3`，
并编译该版本配套的 Warp。不要用 PyPI 的 `warp-lang` 替代这个定制求解器。
默认 CUDA 编译工具链为 `/usr/local/cuda-11.8`，可以设置 `MANISKILL_CUDA_PATH`。
脚本不修改系统 Python、全局 CUDA 链接或其他项目的虚拟环境。

使用前需要 source ROS Humble 和工作区，使 venv 能导入 `rclpy` 和项目包。
双臂刚体物理由 CPU 求解；当前 ManiSkill2 BaseEnv 即使不打开窗口也会创建
SAPIEN renderer，因此需要可用的 Vulkan。MPM 还需要 NVIDIA GPU 和 CUDA Warp。

```bash
cd /home/jerry/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 避免 PATH 中的 Conda Python 3.13 被 CMake 选为 ROS 包安装解释器。
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install \
  --packages-select dual_fr3_maniskill dual_fr3_moveit_config dual_fr3_trunking_mtc \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
export MANISKILL_PYTHON="$PWD/.venv/bin/python"
```

如果这些包此前已经由不同 Python 版本配置过，在该次构建加上 `--cmake-clean-cache`。
`MANISKILL_PYTHON` 也可通过 `maniskill_python:=/绝对路径/.venv/bin/python` 传入。
没有指定时，启动文件使用当前目录下的 `.venv/bin/python`，因此应在工作区根目录启动。

## 启动完整 MTC 任务

```bash
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill \
  maniskill_python:="$MANISKILL_PYTHON" \
  execute:=true \
  preparation_interactive:=false
```

此命令打开 RViz 和 ManiSkill，使用现有 `config/keypoints.yaml`，执行 preparation、
双夹爪操作和正式 MTC 阶段。`preparation_interactive:=false` 让纯仿真测试自动继续；
若需要原来的逐步终端确认，可以设置为 true。

只规划：`execute:=false`。无窗口运行：再加
`maniskill_viewer:=false use_rviz:=false`。

`simulation_backend` 是唯一后端选择参数，支持 `gazebo`、`maniskill`、`fake`、`real`，
默认 `gazebo`。原 `use_gazebo`、`use_fake_hardware` 启动参数及 `auto` 已移除。
MoveIt `demo.launch.py`、MTC `demo.launch.py` 和 `mtc_prototype.launch.py` 均使用此约定。
同一 ROS 域中只能有一套机器人状态源。调试时可以在所有参与终端设置
`ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`，把仿真和其他机器人系统隔离。

## 只启动 MoveIt 与仿真

```bash
ros2 launch dual_fr3_moveit_config demo.launch.py \
  simulation_backend:=maniskill \
  maniskill_python:="$MANISKILL_PYTHON"
```

随后可以使用 RViz 的 Plan & Execute 或既有 MoveIt action 调用。
该启动文件不启动 Gazebo、fake hardware、controller_manager 或 joint_state_publisher。
夹爪 action 使用 Gazebo 分支已采用的 `gripper_cmd` 名称。

## USB 线缆场景

线缆仿真实现已从 `dual_fr3_usb_cable_demo` 合并到 `dual_fr3_maniskill`。
使用同一个 MoveIt 启动入口选择场景：

```bash
ros2 launch dual_fr3_moveit_config maniskill.launch.py \
  maniskill_scene:=usb_cable maniskill_python:="$MANISKILL_PYTHON"
```

也可使用 `ros2 launch dual_fr3_moveit_config usb_cable.launch.py`，或在本包
`demo.launch.py` 中指定 `simulation_backend:=maniskill maniskill_scene:=usb_cable`。
默认 `maniskill_scene:=robot` 保留普通双臂行为。

`maniskill_resources.py` 统一构造最终 URDF/SRDF；仿真、MoveIt、TF 和 RViz 共用该结果。
USB 网格由仿真包提供；仿真包接收上层传入的描述，不反向依赖本配置包。
`cable_config:=...` 指定 USB/线缆参数，`maniskill_config:=...` 指定 ROS 桥接参数。
线缆场景默认控制/物理/发布频率为 50/500/25 Hz，普通场景为 100/500/50 Hz。

USB 仍固定在左 TCP，左夹爪动作仍被拒绝。此次合并未把线缆场景接入 MTC 任务入口。
详细模型说明和独立检查命令见
[线缆说明](../../dual_fr3_maniskill/docs/usb_cable.md)。

## ROS 接口

| 接口 | 类型 / 行为 |
|---|---|
| `/{left,right}_fr3_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/{left,right}_franka_gripper/gripper_cmd` | `control_msgs/action/GripperCommand` |
| `/joint_states` | 全部 18 个关节的实际 position、velocity；不伪造 effort |
| `/{left,right}_fr3_arm_controller/controller_state` | 目标、实际状态和误差，供 MTC readiness gate 和执行器使用 |
| `/clock` | 从物理步数生成的仿真时间 |
| `/maniskill/{left,right}_tcp_pose` | 物理引擎实测 TCP 位姿，world 坐标系，供 TF 一致性检查 |

MoveIt、MTC、readiness gate、robot_state_publisher 与 RViz 使用同一仿真时钟。
TF 由 robot_state_publisher 唯一发布；桥接不会重复发布 TF。

轨迹要求完整包含对应臂的七个关节，可任意排列名称。位置点使用线性插值，
带速度时使用三次插值，同时带速度和加速度时使用五次插值。
支持未来开始时间、action feedback、取消、路径/终点容差和终点等待时间；
同一臂繁忙时拒绝新目标，先取消再提交。左右臂和夹爪可以并发执行。
取消或执行失败后，控制目标切换为当前实测关节位置。

夹爪 `position` 是单根手指的位置，完整开口宽度为其两倍。`max_effort=0` 使用配置默认值，
正值受配置上限约束。`reached_goal` 来自实测位置和速度；未达到目标且持续停止运动时
返回 `stalled`，供现有 grasp 语义判断。这个标志本身不代表已经检测到线缆。

## 仿真模型与参数

配置位于 `dual_fr3_maniskill/config/simulation.yaml`，默认物理 500 Hz、控制 100 Hz、
状态发布 50 Hz。MoveIt 独立启动时可用 `maniskill_config:=...` 指向另一份参数文件。

启动时从 MoveIt 的同一份描述生成临时 URDF，解析 package URI，并转换 STL 为 GLB。
生成文件和 PhysX 缓存放在本次进程的临时目录，退出时清理，原始资产保持不变。
左右臂关节、安装位置、手指变换和 `0.1524 m` 的 TCP 偏移均取自项目描述。

STL 转换保留逐三角形法线，避免在板材和线槽的直角边缘进行平滑着色。否则即使
三角形坐标正确，安装板孔洞周围也会呈格纹，线槽会看起来鼓起。转换由
`dual_fr3_maniskill/assets.py` 完成，不要通过修改 URDF 尺寸或碰撞网格来补偿这种显示问题。
修改转换代码后，重启 ManiSkill 即可重新生成临时模型。灯光与 RViz 不同，亮度仍可能有差异。

双臂保留在同一个固定基座 articulation 中；工作台、底板和线槽几何作为独立静态 actor。
线槽使用静态三角网格碰撞，保留凹槽。SRDF 的所有碰撞排除通过精确的图团覆盖压缩到
PhysX 碰撞位掩码，不依赖 SAPIEN 只识别 `reason=Default` 的默认加载行为。
关节驱动使用 URDF effort 上限，每个物理子步计算并施加机器人被动力补偿。
ROS 桥接从 SAPIEN 2 的实际 `qpos/qvel` 和 link pose 读取反馈，不使用目标位置代替测量。

普通 `robot` 场景保留双臂、夹爪与固定场景执行闭环。可选 `usb_cable` 场景加入
USB 固定安装和 MPM/纤维/刚体接触混合线缆模型，尚未标定真实线材。
现有 MTC 入口仍使用普通场景，任务关键点不会自动变成软体材料约束；
线缆场景尚未接入 MTC 的夹爪流程、力控或视觉反馈。
也没有自动把运行时新增的 MoveIt CollisionObject 同步成 ManiSkill actor。
物理接触网格与 MoveIt 网格的加载方式不同，遇到接触阻挡时应检查碰撞形状和动力学，
不能用假反馈或无条件 action 成功来绕过。

## 验证

依赖一致性检查：

```bash
.venv/bin/python -m pip check
```

纯物理运行和 URDF 正运动学对照：

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/check_simulation.py
```

保存实际渲染图：

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/check_simulation.py \
  --render-output /tmp/dual_fr3_maniskill_scene.png
```

检查安装板孔洞与线槽硬边时，在此命令后加 `--fixture-closeup`。

自动集成验证脚本会固定使用本地域 87，启动自己的仿真并在退出时关闭它。
运行前请关闭该域内其他仿真：

```bash
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode actions
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode mtc
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode tests
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode physics
bash src/dual_fr3_maniskill/scripts/validate_integration.sh --mode mpm
```

`actions` 检查双臂并发执行、夹爪开合、取消、非法关节拒绝、容差错误返回和 TF/TCP 一致性。
`mtc` 使用原关键点文件执行完整任务。加 `--viewer` 可同时打开 RViz 与 ManiSkill。
日志分别写到 `/tmp/dual_fr3_maniskill_actions.log` 和 `/tmp/dual_fr3_maniskill_mtc.log`。

`mpm` 运行官方 `Hang-v0` 环境中的真实 CUDA MPM，检查粒子运动、形变有限性、
求解器错误状态及刚体耦合对象，并保存 `/tmp/dual_fr3_maniskill2_mpm.png`。
这验证的是独立软体参考场景，不表示双 FR3 已经完成线缆布线。
首次运行可能编译 CUDA 内核并生成机器人碰撞 SDF，需要等待。
也可以直接运行并显示：

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/check_mpm.py --viewer
```

后续在双 FR3 中使用 MPM 时，需要基于 `mani_skill2.envs.mpm.base_env.MPMBaseEnv`
建立环境，定义线缆粒子和材料参数，将双臂、夹爪及线槽加入耦合对象，并同步软体状态。
官方默认的网格精度与粘附参数需要根据实际电缆和线槽尺寸调整。

2026-09-08，迁移至 ManiSkill2 后在本机完成的验证：

- 三个包构建成功，177 项 Python/MTC/启动参数与网格回归测试通过，`pip check` 无依赖冲突。
- 无窗口物理测试中，关节目标最大稳态误差约 `3.6e-6`，TCP 与原 URDF FK 的位置误差小于 `2.5e-7 m`。
- 双臂并发 action、两夹爪开合、取消、非法关节拒绝和容差超限均通过真实仿真集成测试。
- RViz 与 ManiSkill2 窗口同时运行时完成执行检查；TF/TCP 位置差最大约 `1.4e-5 m`。
- 原始四个关键点的完整 MTC 任务成功执行，日志确认 `cached MTC solution execution finished`，包含两次准备夹爪操作。
- 定制 Warp 0.3.1 用 CUDA 11.8 编译；RTX 3050 Laptop GPU 上的 `Hang-v0` 推进
  3,636 个粒子、12 个耦合刚体、20 个控制步，状态有限且未触发求解器错误，粒子最大位移约 `0.0126 m`。
- 安装板与线槽离屏渲染完成，原始三角形和硬边法线保留。

迁移时的旧环境位于 `.venv.maniskill3-backup-20260908-163237`。当前依赖快照和验证日志
保存于 `.deps/maniskill2-installed.txt`、`.deps/maniskill2-*-validation.log`；
本次渲染图位于工作区 `temp/maniskill2_mpm.png` 和 `temp/maniskill2_scene.png`。
