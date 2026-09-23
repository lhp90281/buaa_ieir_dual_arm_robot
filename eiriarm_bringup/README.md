# EiriArm Bringup (W3 双臂)

完整且唯一的启动手册已整理到 [仓库根 README](../README.md)。
不要再使用旧 USB2CAN 或全身腰部仓库的启动命令。

- [安装与构建](../README.md#3-安装构建测试)
- [CAN 与 bridge](../README.md#4-can-fd-与-bridge)
- [完整方向/全手动标定/合并](../README.md#calibration)
- [真机与 MuJoCo UI](../README.md#6-真机控制与-ui)
- [主从遥操作](../README.md#teleoperation)
- [控制算法与已知安全边界](../docs/CONTROL_PIPELINE.md)

已完成本机标定时的日常入口（每个终端都从工作区根目录 source install/setup.bash）：

```bash
# 终端 A，只启动一次
ros2 launch eiriarm_bringup bridge.launch.py arms:=dual gripper:=false

# 终端 B，会使能真机，操作前支撑机械臂
ros2 launch eiriarm_bringup real_robot.launch.py \
  arms:=dual gripper:=false controller:=gravity use_gui:=true
```

默认标定路径为工作区下 src/ros2_ws_config/joint_offsets_dual.yaml，
摩擦文件为 src/ros2_ws_config/friction_model.yaml；从其他目录启动时用绝对路径覆盖。
只开右臂须同时修改 bridge 与控制端 arms:=right，控制端可指定单臂 offsets 文件。
没有装夹爪时，不要为了让 GUI 外观匹配而打开 gripper。
UI 的 H/E/S/T 等键会触发真机操作，mirror_real 不是禁用所有命令的只读模式。
