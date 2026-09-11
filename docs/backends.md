# 后端、模型与排查

启动步骤见 [README](../README.md)。后端统一由 `simulation_backend` 选择，默认 `gazebo`。

## 控制器与时钟

| 后端 | 执行方式 | 控制器管理器 | 仿真时间 |
| --- | --- | --- | --- |
| `fake` | 虚拟硬件与夹爪动作服务 | `/left/controller_manager`、`/right/controller_manager` | 否 |
| `real` | 两套 Franka 硬件接口与官方夹爪节点 | 同上 | 否 |
| `gazebo` | Gazebo 内的 `gz_ros2_control` | `/controller_manager` | 是 |
| `maniskill` | SAPIEN 物理与 ROS 动作桥接 | 无 | 是 |

每个后端均由一个 MoveGroup 维护双臂规划场景。Gazebo 和 ManiSkill 提供 `/clock`，MoveIt 与 RViz 使用仿真时间；同一 ROS 域中只运行一套机器人状态源。

| 接口 | `fake` / `real` | `gazebo` / `maniskill` |
| --- | --- | --- |
| 左臂轨迹 | `/left/left_fr3_arm_controller/follow_joint_trajectory` | `/left_fr3_arm_controller/follow_joint_trajectory` |
| 右臂轨迹 | `/right/right_fr3_arm_controller/follow_joint_trajectory` | `/right_fr3_arm_controller/follow_joint_trajectory` |
| 左夹爪 | `/left_franka_gripper/gripper_action` | `/left_franka_gripper/gripper_cmd` |
| 右夹爪 | `/right_franka_gripper/gripper_action` | `/right_franka_gripper/gripper_cmd` |

对应 MoveIt 配置分别为 [moveit_controllers.yaml](../config/moveit_controllers.yaml) 和 [moveit_controllers_gazebo.yaml](../config/moveit_controllers_gazebo.yaml)。ManiSkill 复用后者的 action 名称，不启动 Gazebo。

`load_gripper` 控制夹爪模型。`start_gripper` 在 `fake` / `real` 分支单独控制驱动启动；Gazebo 夹爪控制器随 `load_gripper` 启动，ManiSkill 的夹爪动作由统一桥接提供，不是独立节点。

Gazebo 使用原生 `gz_ros2_control/GazeboSimSystem` 和 position 轨迹控制器。`gazebo_effort:=true` 额外暴露 effort 接口，不会把当前 MoveIt 轨迹控制器改为力矩控制器。`gz_args` 可传入世界文件等参数，例如 `gz_args:="empty.sdf -r"`。

## 模型与坐标

主要配置为 [dual_fr3.urdf.xacro](../config/dual_fr3.urdf.xacro)、[dual_fr3.gazebo.urdf.xacro](../config/dual_fr3.gazebo.urdf.xacro) 和 [dual_fr3.srdf.xacro](../config/dual_fr3.srdf.xacro)。

| 坐标 / 几何 | 当前配置 |
| --- | --- |
| `world` / `worktable` | 固定重合，台面上表面为 `z=0` |
| 工作台碰撞盒 | `0.75 × 1.5 × 0.04` m，中心 `(0.375, 0.75, -0.02)` |
| 左基座 | `world` 下 `(0.175, 0.35, 0)` m |
| 右基座 | `world` 下 `(0.175, 0.95, 0)` m |
| 基座朝向 | 两臂与 `world` 同向，基座间距 0.6 m |
| TCP | `left_fr3_hand_tcp` / `right_fr3_hand_tcp` |

Python 控制和 MTC 关键点默认使用 `left_fr3_link0`，因此右基座在该坐标系下为 `(0, 0.6, 0)`。传目标前需区分台面坐标与左基座坐标。

研究手指由 [research_franka_hand.xacro](../config/research_franka_hand.xacro) 加载，当前网格为 `finger1.STL`；配准和 TCP 说明见[手指网格文档](research_finger_mesh.md)。

视觉网格与碰撞几何用途不同。工作台、安装板使用简化碰撞盒；线槽和手指需保留任务相关的凹槽、孔洞，实际加载方式按后端检查。修改场景时同时核对普通和 Gazebo 描述，以及 ManiSkill 的资产转换，避免只改 RViz 外观。

## 添加障碍物

静态障碍可添加为 URDF 固定 link，优先用 box、cylinder 等简单碰撞形状；复杂网格放入 `meshes/` 并用 `package://dual_fr3_moveit_config/...` 引用。修改后重启环境，核对两个模型分支和碰撞矩阵。

运行时障碍通过 MoveIt PlanningScene / CollisionObject 接口添加，统一使用明确的 `world` 或其他已知坐标系。MoveIt 的动态碰撞物不会自动同步到 Gazebo 或 ManiSkill，若需要物理接触，必须另外创建对应仿真实体。

## 常用检查

加载 ROS 和工作区后，从工作区根目录执行：

```bash
xacro src/dual_fr3_moveit_config/config/dual_fr3.urdf.xacro > /tmp/dual_fr3.urdf
check_urdf /tmp/dual_fr3.urdf
xacro src/dual_fr3_moveit_config/config/dual_fr3.srdf.xacro > /tmp/dual_fr3.srdf
ros2 launch dual_fr3_moveit_config demo.launch.py --show-args
```

环境运行后：

```bash
ros2 action list -t
ros2 topic echo /joint_states --once
```

带夹爪时应包含 14 个机械臂关节和 4 个手指关节。按后端检查控制器：

```bash
# Gazebo
ros2 control list_controllers -c /controller_manager

# 虚拟硬件或真机
ros2 control list_controllers -c /left/controller_manager
ros2 control list_controllers -c /right/controller_manager
```

ManiSkill 使用[动作与状态接口](../../dual_fr3_maniskill/docs/interfaces.md)检查，没有控制器管理器。

| 问题 | 检查方向 |
| --- | --- |
| 规划成功但执行接口不存在 | 比对上表的后端命名空间，检查对应控制器是否激活 |
| RViz 状态跳动或时间异常 | 检查重复的 `/joint_states`、`/clock` 和另一套 launch |
| Gazebo 接受轨迹但运动很慢 | 检查仿真步进与碰撞几何复杂度，避免直接增加高面数接触网格 |
| 真机驱动启动失败 | 检查实际 IP、机器人连接状态及驱动日志 |
| ManiSkill 初始化失败 | 见[环境安装与验证](../../dual_fr3_maniskill/docs/setup.md) |
