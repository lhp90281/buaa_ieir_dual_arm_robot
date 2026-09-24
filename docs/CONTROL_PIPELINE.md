# 控制算法与接口梳理

审查对象：本仓库 W3 固定基座双臂分支，不是腰部/底盘/Pico 全身仓库。
单位统一为 rad、rad/s、Nm、m、s。下文描述现有实现，不把拟议保护当作已实现功能。
启动命令见 [README](../README.md)。

## 1. 数据链路与所有权

```text
DM motor <-> CAN FD can0/can1 <-> w3_robot_bridge_node
  /state (MotorStateArray, raw units, best_effort)
    -> DMHardwareInterface.read -> calibrated ros2_control joint states
    -> joint_state_broadcaster -> /joint_states -> robot_state_publisher / local Web UI

gravity_compensation_controller -> effort (gravity + friction)
joint_position_controller OR cartesian_position_controller
    -> position / velocity / stiffness / damping
  -> DMHardwareInterface.write -> inverse sign/offset + nearest revolution
  -> /w3_robot_bridge_node/commands (MotorCommandArray) -> MIT motor inner loop
```

重力控制器常驻、独占 effort，两个位置控制器互斥、独占另外四个接口。
不是三个控制器轮流占用力矩，也不是在 bridge 再算一次重力。
默认控制器 manager 为 300 Hz，bridge 每通道调度及状态发布为 350 Hz；
这些是软件目标频率，不等同硬实时保证或每帧反馈一定到达。

## 2. 总线与电机标识

| 人工关节号 | slot / URDF 后缀 | CAN ID | 型号 | VM / TM 协议范围 |
|---|---|---|---|---|
| 1, 2 | 0, 1 | 1, 2 | DM8009 | +/-45 rad/s, +/-54 Nm |
| 3 | 2 | 3 | DM4340P | +/-20 rad/s, +/-28 Nm |
| 4 | 3 | 4 | DM4340 | +/-20 rad/s, +/-28 Nm |
| 5, 6, 7 | 4, 5, 6 | 5, 6, 7 | DM4310 | +/-50 rad/s, +/-10 Nm |
| 可选夹爪 | 7 | 8 | DM4310 | +/-50 rad/s, +/-10 Nm |

左臂 channel=0/can0，右臂 channel=1/can1。只开右臂时 channel 0 留空占位，绝不重编号。
编码 PMAX=12.5，KP 范围 0..500、KD 范围 0..5，须与固件配置一致。
protocol ranges 决定整数与物理量的比例，limits 才是发送前裁剪，二者不能混用。
feedback_type=1 对应当前达妙反馈；ERR=1 为使能、0 为失能，其他错误不能当作正常失能。
MotorState 还有 online 标志；颜色指示灯不是完整的软件状态证据。

配置：`W3_ROBOT/w3_robot_bridge/config/motor_config/{left_arm,right_arm}`。
全部位置 offset=0，position/velocity/torque signs=+1，bridge 暴露 raw 电机坐标。
不要再把 ros2_ws_config 的标定写进这些 YAML，否则会重复变换。

## 3. 标定和坐标

方向阶段用模型零位加实测增量，只决定 s=+1/-1；接近模型零位不是精确绝对标定。
限位参考为用户在模型中确认的 q_ref，手动稳定采样 raw_ref 后：

```text
offset = raw_ref - s * q_ref
q = s * (raw - offset)
v = s * raw_velocity
tau_measured = s * raw_torque

raw_q_target = s * q_target + offset
raw_v_target = s * v_target
raw_tau_ff = s * tau_ff
```

KP/KD 不乘符号，因为同一个符号已同时作用于命令和反馈误差。
硬件接口按 URDF 范围决定是否适合周期窗口归一化；允许时状态取相应 2pi 窗口，
位置命令逆变换选择离当前 raw 最近的一圈，避免切控制器时突然追到相差 2pi 的目标。
纯力矩模式 KP=KD=0 时，position 字段镜像当前 raw，velocity=0，真正驱动力来自 effort。

全手动入口总是 SeekMachine(manual=True)，发送层拒绝非零 KP/KD/v/torque；
先显示预期限位，采样即保存，随后实测预览，没有自动寻限位、回零或 re-seek。
历史自动求解类保留供离线测试，不被当前命令入口启用；旧 --auto-group 明确拒绝。
单臂每个关节原子保存，仅替换本关节；左右 14 关节由 merge_offsets.py 显式合并。
verified 方向优先于旧合并文件；如果变更旧方向必须有参考采样才能重新计算 offset。

## 4. URDF 与动力学

真机 `dual_arm.launch.py` 展开 `dual_arm_ros2_control.urdf.xacro`，包含
`description/dual_arm_support/urdf/dual_arm_robot_plug.urdf`。
gripper=false 时先删除夹爪全部子树（含惯性），再将同一 robot_description 交给
控制器及 robot_state_publisher。双臂几何和 attachment frame 不改。
单臂模式只暴露所选臂的 ros2_control 接口，仍保留固定基座全模型。

IeirDynamics 用 Pinocchio，模型根坐标为 base_footprint，重力 [0,0,-9.81]。
本分支没有动态腰部状态或移动基座姿态输入。若底座倾斜/改安装方式，重力方向必须重新建模。
日常 Web UI 使用同源 URDF 的视觉模型，gripper 与启动参数同步；显示不参与动力学计算。
历史 MuJoCo 面板已移至可选 `ieir_simulation`，不再由真机 launch 启动。
控制器、Web 模型和默认手动标定不依赖仿真包；真实模型仍来自 description 的 URDF。

## 5. 重力与摩擦前馈

`gravity_compensation_controller.cpp` 每个周期将实测关节状态按名字映射到全模型。
速度滤波和输出：

```text
v_f = alpha * previous_v_f + (1 - alpha) * measured_v
tau_g = Pinocchio.computeGeneralizedGravity(q)
tau_f = 0                                      if abs(v_f) <= deadband
tau_f = friction_gain * (C_pos + B*v_f)         if v_f > deadband
tau_f = friction_gain * (-C_neg + B*v_f)        if v_f < -deadband
tau_ff = clamp(gravity_gain * tau_g + tau_f, joint_effort_limit)
```

默认 alpha=0.98、deadband=0.05 rad/s；每关节倍率见 dual_arm_controllers.yaml。
摩擦文件按电机型号提供 coulomb_pos/coulomb_neg/viscous；static 字段不用于这个前馈。
此处使用 URDF 方向的实测滤波速度，非标定旧 seek 算法的目标速度前馈。
已有摩擦参数源自历史辨识，正负不对称项的坐标约定在换符号/传动时仍需实机确认。
加载摩擦参数失败时当前实现告警并关闭摩擦项，不能仅凭控制器 active 判断摩擦已生效。

重力控制器不提供位置恢复力或额外阻尼。推动后不会像位置伺服一样锁住，
但持续自加速/翻起是异常，须查标定、模型、倍率、协议量程和力矩方向。
运动摩擦补偿是正向注入力矩，过补偿可能自激；不能当作安全制动器。

## 6. 关节位置控制

`joint_position_controller.cpp` 接收 `/joint_position_command` 的 JointTrajectory。
激活时捕获当前实测位置保持；新轨迹以最后命令为衔接起点，按 joint_names 做索引。
目标点间按 time_from_start 分段线性插值，速度前馈为该段斜率；最后一点后保持。
它不占 effort，所以重力+摩擦前馈仍然存在。实际电机计算：

```text
tau_motor = KP*(raw_q_des - raw_q) + KD*(raw_v_des - raw_v) + raw_tau_ff
```

停用位置控制器会清零 KP/KD/速度并镜像当前位置，不应同时关闭 gravity controller。
线性插值不是 jerk/加速度受限轨迹，不是碰撞规划，不保证回零路径无干涉。
默认增益由 dual_arm_controllers.yaml 和 teleop_joint_gains.yaml 的 profile 生成运行配置，
后者会覆盖关节位置增益；笛卡尔增益是另一组参数，修改时不要只改其中一处。
当前 master/slave profile 数值相同，不能根据名字认为 master 已较软。

## 7. 笛卡尔 IK 与跟踪

输入 `/cartesian_position_controller/left_target_pose`、`right_target_pose`，PoseStamped。
当前代码直接取 pose 数值，不做 frame_id 的 TF 变换，发送者必须先转到 base_footprint。
末端为 left_attachment_point/right_attachment_point。输入四元数归一化，零范数按单位旋转。
激活时 q_desired/q_command/q_home 都从当前姿态捕获；每个新目标从上一期 q_desired 求解。

`IeirDynamics::solveIK` 迭代：

```text
T_error = inverse(T_current) * T_target
e = log6(T_error)                       # LOCAL/body frame
J = computeFrameJacobian(..., LOCAL)
dq = J^T * (J*J^T + lambda^2*I)^(-1) * e
q_next = integrate(q, ik_dt * dq)
q_next = clamp(q_next, model joint limits)
```

误差范数达阈值才接受，未收敛告警并保留旧目标，后续周期继续尝试待处理目标。
只拷贝该臂解出的关节，非双臂相对位姿约束求解。
随后每关节用 position_interpolation_speed*dt 限制 q_command 的步长，v_ff=step/dt。
这是**关节限速追踪 IK 解**，不保证末端直线、恒速或指定时间到达。
home 请求回到本次激活捕获的 q_home，不是网页“回零”的模型全零，二者不要混淆。
阻尼最小二乘不是完善奇异保护：没有本分支内的显式伸直检测、奇异裕量门控、jerk 或碰撞保护。

## 8. W3 输出、反馈与故障边界

HardwareInterface 发送 MotorCommandArray，每项使用 channel/motor_index，不使用电机 CAN ID 作为 slot。
桥拆分命令后按型号编码，clip 后发送；实机启停采用重复使能/失能与反馈检查。
启动读取缺失/重复/非有限 offsets 拒绝控制。错误在线反馈不会覆盖已缓存状态。

**待后续强化（本次未改变已验证运动算法）：** 硬件接口运行态 read 当前仍返回 OK，
没有独立逐关节反馈年龄联锁。bridge 虽会标记 offline，但上层可继续基于缓存状态发命令。
bridge watchdog 只看命令活动，不等价于反馈中断时可靠全臂停车；其他发布者也可能刷新它。
不得用这些机制替代硬件急停或认为单电机断线一定触发所有电机失能。

0.5 秒无命令后的阻尼动作是 KP=0、KD=2，不做姿态锁定；关停 bridge 不能保证静态托举。
力矩裁剪针对前馈字段，不等于总 MIT 弹簧/阻尼输出严格受相同软件阈值约束。

## 9. 双机遥操作

`teleop_joint_bridge.py` 是上层节点，不直接往 W3 motor commands 写 raw 位置。
读取本机 /joint_states，通过 UDP 交换关节名字/位置/角色/准备使能状态及可选夹爪比例，
按本地关节名字重排对端数据，再向本机 JointTrajectory 接口发目标。
两台本地 ROS 图独立，名字集合和标定后的 URDF 语义必须一致。

no_feedback 状态流：
1. prepare 等到本机及对端状态；master 暂切 joint_position，约 5 秒对齐 slave。
2. 对齐后 master 回 gravity；enable 检查准备状态和最大关节误差。
3. master 发送实测 q；slave 限制每周期目标变化并下发位置轨迹。
4. 断连/误差过大 disable 停止跟随，保持策略依赖当前本地控制模式；exit 尝试恢复 gravity。

默认交换 50 Hz，timeout 0.3 s，max_start_error 0.5 rad，max_runtime_error 1 rad，max_step 0.03 rad。
步长限制不是加速度/jerk 规划，不能直接理解为绝对安全速度。
force_feedback 只是双方位置耦合实验；没有实测外力闭环或无源性保证。
UDP 无身份认证、加密、完整重放防护；只用于受信网络。
gripper=false 在回调和发布路径均禁用夹爪控制；true 时使用各机行程的归一化比例。

## 10. 显示、仿真与测试划分

真机 Web UI：`ui.launch.py` 统一入口，打开页面不使能；确认后启停自有控制端、标定、遥操作、回放。
bridge 独立手动启动；控制端和标定互斥，进程入口固定白名单，外部进程不被终止。
`web_console.launch.py` 默认保留只连接模式；显式确认后切模式或发布轨迹。
回零将当前反馈到零位的五次曲线按 50 Hz 采样成 JointTrajectory，理论峰值不超过 20°/s；
仍由原控制器分段线性插值，保留 gravity effort，不保证碰撞安全或实际到达。
网页显示以 30 Hz SSE 接收姿态并单独插值绘制，不修改控制频率，不外推丢失反馈。
纯仿真：从 /ctrl/command + /ctrl/gains 合成 MIT 力矩推进 MuJoCo，并发布 joint_states。
标定显示器：只有 FK，无物理，采样前是理论参考角，采样后是新标定角；其余关节显示零位。
三种窗口不能相互替代。

离线测试覆盖 W3 型号/slot/方向配置、夹爪质量剔除不改变臂几何、标定公式/保存/手动入口、
主从无夹爪发布，以及分发依赖/模型资源。假反馈 GUI 测试不接 CAN，不能证明真实力矩或时延安全。
真实安装差异、固件量程、电机摩擦、负载和急停均必须在目标机器人逐项验证。
