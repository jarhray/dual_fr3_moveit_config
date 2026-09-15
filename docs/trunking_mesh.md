# 原始与简化线槽模型

## 当前 MTC 试验选择

ManiSkill 的 `trunking_cable` 场景读取线缆 YAML 的 `scene.trunking_mesh`，
在最终共享 URDF 中一起选择显示、碰撞网格及对应局部原点：

| 值 | 网格 | 局部原点 xyz / m |
| --- | --- | --- |
| `original` | `meshes/Trunking.STL` | `0 0 0` |
| `simplified` | `meshes/Trunking_simplify.stl` | `0.00014546 -0.00068397 0` |

当前默认 `trunking_cable.yaml` 使用原始网格、3 mm 线径和校正后的初始穿线位置。
已完成完整任务的简化网格 / 2 mm 配置保存在 `trunking_cable_simplified_2mm.yaml`。
详细验证见[3 mm 试验](../../dual_fr3_maniskill/docs/debugging_summary.md#original-3mm)。
此选择仅作用于启用线缆的 ManiSkill 场景；普通机器人及 Gazebo 的 Xacro 默认仍为简化网格。
未指定该 YAML 字段时保留输入 URDF，历史配置并不自动固定线槽版本。

## 简化网格来源与坐标对齐

2026-09-15 起，`dual_fr3.urdf.xacro` 和 `dual_fr3.gazebo.urdf.xacro` 的
`trunking` 显示、碰撞几何统一使用 `meshes/Trunking_simplify.stl`。
这是用户提供的 `temp/Trunking_simplify.stl` 的逐字节副本，单位为米；
原始 `meshes/Trunking.STL` 保留，当前用于上述原始线槽试验。
旧的一次性网格切换脚本已随调试目录清理。现在可直接在配置副本中显式设置
`scene.trunking_mesh: original` 或 `simplified`，由正式入口选择对应网格与原点。
历史实验条件与结论保留在[调试总结](../../dual_fr3_maniskill/docs/debugging_summary.md#resolution)。

新网格有 10,938 个三角面，旧网格有 53,712 个，减少约 79.6%。
它填平齿缝，保留线槽内部通道；静态碰撞仍加载完整三角网格，未使用会封闭槽腔的凸包。

12 段线槽的外形尺寸与旧模型一致，但 CAD 导出原点存在共同偏移。
因此两处 URDF 的 visual 和 collision 都设置局部原点
`xyz="0.00014546 -0.00068397 0"`，将它们对齐原有场景布局。
`plate_to_trunking` 和 MTC 的任务坐标保持原有定义。

新文件另含一个独立零件，共有 13 个封闭网格分量。其最高点为 88.8 mm，
原有槽壁最高点仍为 58.4 mm；用户确认保留新尺寸，未删除或压缩此零件。
显示与碰撞都包含它，以便检查规划路径与实际接触。

构建并重启场景后生效：

```bash
colcon build --symlink-install --packages-select dual_fr3_moveit_config
source install/setup.bash
```

历史测试条件与结果见
[简化线槽验证记录](../../dual_fr3_maniskill/docs/debugging_summary.md#resolution)。
