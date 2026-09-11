# Python 控制接口

本包的脚本提供同步调用形式的 MoveIt 和夹爪接口。先按 [README](../README.md) 启动机器人环境，再在另一个已加载 ROS 和工作区的终端调用。

## 导入与最小示例

从工作区根目录把脚本目录加入导入路径：

```bash
export PYTHONPATH="$PWD/src/dual_fr3_moveit_config/scripts${PYTHONPATH:+:$PYTHONPATH}"
```

下例只规划左臂目标，不执行运动：

```python
import time
import rclpy
from fr3_controller import DualFR3Controller

rclpy.init()
controller = None
try:
    controller = DualFR3Controller()
    deadline = time.monotonic() + 10.0
    while controller.current_joint_state is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("未收到关节状态")
        rclpy.spin_once(controller, timeout_sec=0.1)
    ok = controller.move_to("left", 0.4, 0.0, 0.4, execute=False)
    print("规划成功" if ok else "规划失败，请查看 MoveIt 日志")
finally:
    if controller is not None:
        controller.destroy_node()
    rclpy.shutdown()
```

调用前要处理 ROS 回调并收到关节状态。控制方法内部会等待 action / service 返回，不应从阻塞它所需回调的执行上下文中调用。

直接运行下面两个脚本会执行其内置动作序列，包括机械臂运动和夹爪开合，适合在仿真中检查：

```bash
python3 src/dual_fr3_moveit_config/scripts/fr3_controller.py
python3 src/dual_fr3_moveit_config/scripts/fr3_controller_lin.py
```

## 坐标与单位

`arm` 取 `left` 或 `right`。默认 `default_frame="left_fr3_link0"`，`x/y/z` 为米，`roll/pitch/yaw` 为弧度，默认姿态为 `(π, 0, 0)`。可在构造时修改默认坐标系，或逐次传入 `frame_id="world"`、`frame_id="right_fr3_link0"` 等。

右基座在默认左基座坐标系下为 `(0, 0.6, 0)`；它与右臂自身坐标系的原点不同。双臂目标格式为 `[x, y, z, roll, pitch, yaw]`。

## 点到点与夹爪

类 `DualFR3Controller` 定义于 [fr3_controller.py](../scripts/fr3_controller.py)。运动方法默认 `execute=True`，预览需显式设为 `False`。

| 方法 | 用途 |
| --- | --- |
| `move_to(arm, x, y, z, ..., execute=True, frame_id=None)` | 单臂 TCP 目标；`move_left_to` / `move_right_to` 为快捷形式 |
| `move_dual_to(left_pose, right_pose, execute=True, frame_id=None)` | 使用 `dual_fr3_arms` 联合规划 |
| `reset_arm(arm)` / `reset_both_arms()` | 回到机械臂初始目标 |
| `reset(execute=True, reset_grippers=True)` | 复位机械臂，并按开关复位夹爪 |
| `rotate(arm, z, execute=True, relative=True)` | 第 7 轴相对或绝对旋转 |
| `rotate_to_home(arm)` | 第 7 轴回参考角 |
| `open_gripper(arm, width=0.08, speed=0.08)` | 打开到指定总宽度 |
| `close_gripper(arm, width=0.0, speed=0.08)` | 闭合到指定总宽度 |
| `set_gripper_action(arm, grip_action)` | 归一化开口，0 为闭合、1 为全开 |
| `grasp_object(arm, width=0.02, speed=0.05, force=50.0, ...)` | 抓取；真实工件需显式设置适合的参数 |
| `get_gripper_state(arm)` | 返回已接收的夹爪状态 |

`width` 为总开口，底层 `GripperCommand.position` 为单指位移。脚本优先使用真机 `/move`、`/grasp` 接口，仿真通过可用的 `gripper_action` / `gripper_cmd` 执行。MTC 使用独立的 profile 和限值配置，不与本脚本的默认抓取力混用。

本包配置的规划管线是 OMPL。通用脚本当前请求中的默认 `planner_id` 仍是 `PTP`，不要据方法名称认定正在使用 Pilz。需要严格复现 MTC 的 RRTConnect 或直线路径检查时，使用 MTC 或其逐段诊断工具。

## 笛卡尔接口

`DualFR3LinearController` 定义于 [fr3_controller_lin.py](../scripts/fr3_controller_lin.py)，继承上述控制类。

| 方法 | 行为 |
| --- | --- |
| `move_to(...)` | 默认尝试笛卡尔路径 |
| `move_to_cartesian(..., max_step=0.005, min_fraction=0.9)` | 请求笛卡尔路径，步长单位米 |
| `move_to_ompl(...)` | 调用基类的 MoveGroup 点到点接口 |
| `get_current_pose(arm)` / `get_current_pose_rpy(arm)` | TCP 在默认坐标系中的最新已缓存位姿 |
| `move_dual_to_cartesian(left_pose, right_pose, ...)` | 回退到双臂联合 MoveGroup 规划 |

笛卡尔服务不可用或返回比例低于 `min_fraction` 时，单臂方法也会回退到点到点规划。因此方法返回成功不保证执行的是完整直线，也不提供真正的双臂同步直线插补。位姿查询依赖 TF 回调和缓存更新。
