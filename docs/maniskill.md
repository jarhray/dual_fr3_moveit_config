# ManiSkill 使用导航

完整的环境、物理接口和线缆文档集中在 `dual_fr3_maniskill`，避免两处维护不同的安装步骤和默认值。

- [安装与快速开始](../../dual_fr3_maniskill/README.md)
- [环境安装与验证](../../dual_fr3_maniskill/docs/setup.md)
- [ROS 接口与仿真模型](../../dual_fr3_maniskill/docs/interfaces.md)
- [独立 USB 线缆场景](../../dual_fr3_maniskill/docs/usb_cable.md)
- [MTC 准备阶段线缆](../../dual_fr3_maniskill/docs/mtc_cable.md)

加载 ROS、工作区和 ManiSkill 环境后，在工作区根目录选择一个入口：

```bash
# 普通机器人与 MoveIt
ros2 launch dual_fr3_moveit_config demo.launch.py simulation_backend:=maniskill

# 独立 USB 线缆演示
ros2 launch dual_fr3_moveit_config usb_cable.launch.py

# 包含准备夹持与线缆生成的 MTC 任务
ros2 launch dual_fr3_trunking_mtc mtc_prototype.launch.py \
  simulation_backend:=maniskill execute:=true
```

普通入口默认 `robot`，USB 入口选择 `usb_cable`；MTC 默认选择 `trunking_cable`，两次准备闭合后才生成线缆。MTC 使用 `maniskill_cable:=false` 可关闭线缆，`execute:=false` 只规划。
