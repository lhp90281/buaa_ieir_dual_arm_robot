# IEIR Controllers

`ieir_controllers` 提供 IEIR 使用的 ros2_control 硬件接口、控制器插件和辅助脚本。

日常使用请优先看主手册：

```bash
src/ieir_bringup/README.md
```

本包包含：

- `DMHardwareInterface`：真机 DM 电机硬件接口
- `GravityCompensationController`：重力补偿和摩擦前馈
- `JointPositionController`：MIT 五参数关节位置控制，订阅 `/joint_position_command`
- `CartesianPositionControllerPlugin`：双臂笛卡尔目标逆解控制
- `gripper_controller_node`：独立夹爪控制节点，支持开/合/保持和遥操作 passive/track 模式
- `teleop_joint_bridge`：双机 UDP 关节遥操作桥
- `teleop_force_observer`：从臂外力估计观测/记录/绘图脚本
- `config/teleop_joint_gains.yaml`：实验性力反馈 master/slave 关节位置增益 profile；不建议日常真机使用
- `go_home`、`teach_replay`、`joint_zero_calibration`、`zero_at_current_pose` 等工具脚本

注意：本包不提供运动规划。笛卡尔控制器只对目标位姿做 IK，并把结果作为关节目标插值执行；它不做碰撞检测、避障或路径搜索。

本包不依赖 MuJoCo。仿真 topic 硬件接口、配置和 MJCF 已移至可选
[`ieir_simulation`](../simulation/README.md)。标定默认使用 Web UI；真机与仿真共用本包的控制算法。

常用入口：

```bash
# 真机，推荐通过 bringup 启动
ros2 launch ieir_bringup bridge.launch.py
ros2 launch ieir_bringup real_robot.launch.py

# 直接启动控制器侧，要求 bridge 已经在跑
ros2 launch ieir_controllers dual_arm.launch.py

# 回零、录制、回放也推荐通过 bringup 封装
ros2 launch ieir_bringup go_home.launch.py
ros2 launch ieir_bringup record.launch.py output:=recordings/demo.yaml
ros2 launch ieir_bringup replay.launch.py input:=recordings/demo.yaml

# 双机遥操作，具体 IP/端口和流程见 ieir_bringup/README.md
ros2 launch ieir_bringup teleop.launch.py role:=master peer_host:=192.168.10.20
```
