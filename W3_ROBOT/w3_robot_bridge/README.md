# W3 双臂 CAN Bridge

本分支从全身版驱动抽取双臂接入配置。详细启动、重新标定和遥操作流程见
[W3_MIGRATION.md](../../W3_MIGRATION.md)。

- 只支持 can0（左臂）和 can1（右臂），CAN FD 1M/5M。
- 当前 launch 默认 `arms:=right`，仅打开 can1，消息 channel 仍为 1；恢复双臂传 `arms:=dual`。
- 每臂默认 7 个电机，slot 0..6 = CAN ID 1..7。
- `gripper:=true` 时增加每臂 slot 7 / CAN ID 8。
- 不启动 can2；auto_enable_on_start 默认为 false。
- 默认每个电机 350 Hz 发送，ROS 状态 350 Hz 发布。
- 主配置在 `config/w3_robot_bridge.yaml`；逐电机型号和编解码范围在 `config/motor_config/`。
- 零偏与方向只在控制器硬件接口应用；本层 position_offset=0、signs 全部为 1。

```bash
ros2 launch w3_robot_bridge w3_robot_bridge.launch.py arms:=right
# 安装夹爪后：
ros2 launch w3_robot_bridge w3_robot_bridge.launch.py arms:=dual gripper:=true
```

| 接口 | 类型 | 用途 |
|---|---|---|
| ~/commands | MotorCommandArray | 批量电机命令 |
| ~/command | MotorCommand | 单电机命令 |
| ~/state | MotorStateArray | 聚合反馈，BestEffort |
| ~/enable | EnableMotor | 电机使能 |
| ~/disable | DisableMotor | 电机失能 |
| ~/zero | SetZero | 电机硬件清零（软件标定不用这个服务） |

默认节点名为 w3_robot_bridge_node。消息中的 channel 是 0/1，
motor_index 是零起始 slot，不能直接填 CAN ID。

MIT 控制命令使用 mode=3 和 position、velocity、kp、kd、torque。
反馈包含 online、enabled、error_flags 及原始电机位置/速度/力矩。
状态字段 error_flags 保留达妙反馈高四位：0=失能，1=使能，8..14=故障。
反馈支持共享主机 CAN ID `0x000`，通过数据第一个字节的低四位区分电机；
也兼容独立反馈 ID（例如 `0x11..0x17`）。看到电机使能指示灯但 ROS 全部
`online=false` 时，先检查反馈解码，不要跳过标定的状态确认。
