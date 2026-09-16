# 原始与简化线槽模型

## 当前 MTC 试验选择

ManiSkill 的 `trunking_cable` 场景读取线缆 YAML 的 `scene.trunking_mesh`，
选择碰撞网格；显示默认跟随它，也可通过 `scene.trunking_visual_mesh` 单独指定。
两个字段接受相同的模型名，并分别使用对应局部原点：

| 值 | 网格 | 局部原点 xyz / m |
| --- | --- | --- |
| `original` | `meshes/Trunking.STL` | `0 0 0` |
| `simplified` | `meshes/Trunking_simplify.stl` | `0.00014546 -0.00068397 0` |

当前默认 `trunking_cable_simplified_2mm.yaml` 使用新版简化碰撞、原始视觉网格和 2 mm 线径。`trunking_cable.yaml` 作为原始碰撞网格对照保留。
旧固定夹持版本曾完成完整任务的简化网格 / 2 mm 配置保存在 `trunking_cable_simplified_2mm.yaml`。
该配置当前的显示与碰撞选择：

```yaml
scene:
  trunking_mesh: simplified
  trunking_visual_mesh: original
```

MoveIt / MTC 使用简化碰撞几何，ManiSkill / RViz 当前显示原始 CAD。
需要显示实际碰撞外形时设置 `scene.trunking_visual_mesh: simplified`。
省略 `trunking_visual_mesh` 时保持原有的显示与碰撞一起切换行为。
详细验证见[3 mm 试验](../../dual_fr3_maniskill/docs/debugging_summary.md#original-3mm)。
此选择仅作用于启用线缆的 ManiSkill 场景；普通机器人及 Gazebo 的 Xacro 默认仍为简化网格。
两个 YAML 字段均未指定时保留输入 URDF；只指定显示字段时保留输入的碰撞几何。

## 简化网格来源与坐标对齐

2026-09-16 将用户提供的 `temp/trunking-simplified_1.stl` 逐字节覆盖到
`meshes/Trunking_simplify.stl`，旧简化文件不再保留在代码库中。保留资源文件名，
因此两处 Xacro、ManiSkill 和 MoveIt 的现有引用继续有效。单位为米。
原始 `meshes/Trunking.STL` 保留作为原始 CAD 对照。

| 网格 | 三角面数 | 封闭连通分量 | 最高点 / mm |
| --- | ---: | ---: | ---: |
| 原始 CAD | 53,712 | 12 | 58.4 |
| 上一版简化（已替换） | 10,938 | 13 | 88.8 |
| 当前简化 | 304 | 1 | 65.0 |

当前版比上一简化版减少约 97.2% 三角面。静态碰撞仍加载非凸三角网格，
保留内部通道，不使用封闭槽腔的整体凸包。

这次不仅减少三角面：外轮廓、部分槽壁和槽高也有改变，旧版独立零件已消失。
新文件局部 AABB 约为 `[-0.549544, -0.963441, 0]` 到
`[0.004034, -0.002741, 0.065]` m。这些差异不能通过单一平移消除。
保留现有场景局部变换 `xyz="0.00014546 -0.00068397 0"`，
不擅自移动 `plate_to_trunking` 或修改任务路径；该变换是沿用的布局约定，
不表示新旧槽壁精确重合。旧版按 58.4 mm 槽沿生成的任务路径需要重新检查。
当前配置显示原始 CAD；查看新模型实际槽壁时应切换为 simplified 显示，
以检查显示与碰撞之间的差异。

构建并重启场景后生效：

```bash
colcon build --symlink-install --packages-select dual_fr3_maniskill dual_fr3_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

历史测试条件与结果见
[简化线槽验证记录](../../dual_fr3_maniskill/docs/debugging_summary.md#resolution)。
