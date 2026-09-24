# W3 双臂版调试说明

从克隆到启动的发布版流程以 [根 README](README.md) 为准；本文件保留迁移和标定细节。

基线：旧双臂仓库 `070fea0`。W3 通信来自全身仓库 `36c28ee`。
工作分支：`feat/w3-dual-arm-can`。
本地独立工作区：`/home/lhp/ros2_ws/w3_dual_arm_ws`。
原全身工作区未切换分支。不要混用两个工作区的 install 环境。

## 范围

- 14 个双臂关节，保留重力补偿、关节控制、笛卡尔控制、双机 UDP 遥操作与录制回放。
- 默认无夹爪，不启动夹爪节点，不发送夹爪 CAN 命令，真机模型不含夹爪质量。
- 不引入底盘、腰部模型/控制器或 WebUI。W3 驱动内保留的其他协议代码不被双臂配置启用。
- 旧 USB2CAN 源码留作参考，通过 COLCON_IGNORE 排除构建；运行与标定工具均已切到 W3。
- MuJoCo 仿真/兼容面板已移至可选 `simulation/ieir_simulation`，默认不构建，真机和默认标定不依赖它。其静态 MJCF 仍带夹爪；真机重力补偿只使用可选夹爪 URDF，不使用 MJCF。

## 构建

新终端中执行：

```bash
cd /home/lhp/ros2_ws/w3_dual_arm_ws
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src --packages-up-to ieir_controllers ieir_bringup
source install/setup.bash
```

离线测试，不连接 CAN：

```bash
colcon test --base-paths src --packages-select w3_robot_bridge ieir_controllers
colcon test-result --verbose
```

## 通信与电机

### 单臂排查模式

发布版 bridge launch 默认 `arms:=dual`；单臂排查请显式指定 `arms:=right`，只打开 can1，保留右臂 channel=1；
can0 不打开、不轮询、不发送 watchdog 或使能/失能帧。夹爪仍默认关闭。
先停止旧 bridge，再启动，禁止同时运行两个 bridge：

```bash
ros2 launch ieir_bringup bridge.launch.py arms:=right gripper:=false
```

单独测试右臂时，控制端也必须选择右臂并指定标定文件：
`ros2 launch ieir_bringup real_robot.launch.py arms:=right gripper:=false offsets_yaml:=src/ros2_ws_config/joint_offsets_right.yaml`。
控制端默认仍为 dual；单臂模式支持重力补偿/关节控制，当前笛卡尔控制器要求 dual。
双臂时建议 bridge 显式传 `arms:=dual`。以下双臂流程已显式指定该参数。

左臂 channel 0 / can0，右臂 channel 1 / can1。slot 0..6 对应 CAN ID 1..7，
可选夹爪 slot 7 对应 CAN ID 8。两条总线都是 CAN FD，仲裁 1 Mbit/s，数据 5 Mbit/s。
默认每电机发送与 ROS 状态发布均为 350 Hz，控制器原频率保留。

| slot | 型号 | PMAX | VMAX | TMAX |
|---|---|---:|---:|---:|
| 0、1 | DM8009 | 12.5 | 45 | 54 |
| 2 | DM4340P | 12.5 | 20 | 28 |
| 3 | DM4340 | 12.5 | 20 | 28 |
| 4、5、6、可选 7 | DM4310 | 12.5 | 50 | 10 |

这些编解码范围沿用旧双臂配置。若电机寄存器被改过，应同步更新对应 YAML。
W3 的 `position_offset=0`、三个 `signs=1`，只传原始电机角度。
硬件接口统一执行 `q_urdf = axis_sign * (q_raw - zero_offset)`，
回写使用与当前读数最近的等价圈数。W3 原始位置限幅是 ±12.5 rad，
不能把 URDF 的 ±pi 当作原始编码器范围。

CAN 配置命令见根 README 第 4 节；先停止控制并支撑机器人，不在运动中重新配置总线。

## 重新标定

### 推荐：先方向，再限位

保持 `bridge.launch.py arms:=dual gripper:=false` 运行，停止所有控制器、
遥操作及手动重复使能命令。一次标定一条臂；支撑好双臂，标定臂使用
零阻抗，没有重力补偿。另一条臂不会由该标定脚本使能。

第一步，右臂方向确认：

```bash
ros2 run ieir_controllers joint_zero_calibration \
  --mode direction \
  --calibration-yaml src/ros2_ws_config/joint_calibration_dual_right.yaml \
  --output src/ros2_ws_config/joint_directions_right.yaml
```

默认使用 Web UI 的标定页，仅做 FK 显示，不运行物理、不发布机器人位置目标。
先启动 `ros2 launch ieir_bringup ui.launch.py workspace:="$PWD"` 并打开标定页；
推荐直接在网页选择方向确认阶段，不需要执行上面的终端命令。
模型起始为全零；每次只有当前高亮关节显示 `axis_sign * (raw_now - raw_start)`。
不要求编码器读数为零，也不会把近似零姿态作为最终零偏。

1. 真实手臂摆在接近模型零位，保持网页标定页可见。
2. 按 Enter 记录当前关节增量起点，模型仍为零。
3. 手动小幅运动当前关节，至少约 5 度；比较真实与模型运动方向。
4. 相反按空格翻转显示；一致按 Enter 确认，进入下一关节并清零模型。
5. R 清除本关节起点，恢复近零后再按 Enter；Backspace 返回上一关节。
6. 网页停止、心跳丢失或终端 Ctrl+C 中止，脚本尝试失能并确认反馈。网页提供对应确认/翻转操作按钮。

增量超过 1 rad、相邻显示采样跳变超过 0.35 rad 或反馈丢失，会阻止确认；
恢复反馈、把手臂放回近零后按 R 重做。鼠标左拖旋转视角，右拖平移，滚轮缩放。
所有方向确认且退出失能确认成功才写文件；中止不会覆盖已有方向结果。

第二步，先检查限位参考角，再带方向文件进行右臂限位标定。

可以独立打开只读预览，不需要 bridge，也不会使能或控制电机：

```bash
source install/setup.bash
ros2 run ieir_controllers joint_zero_calibration \
  --mode limit-preview \
  --calibration-yaml src/ros2_ws_config/joint_calibration_dual_right.yaml \
  --output src/ros2_ws_config/joint_calibration_dual_right_reviewed.yaml
```

窗口按人工关节编号 `7-6-5-3-4-2-1` 显示参考姿态，当前关节高亮。
同时显示配置中的参考角、限位方向和 URDF 软件范围；其余关节显示为零，
不读取真机位置，不运行物理仿真，也不能据此认定真机不会碰撞。

- `E` 输入角度（单位度，允许负数），模型实时预览；`Enter` 结束输入，再按一次确认。
- 左右方向键微调 1 度，按住 `Shift` 微调 0.1 度。
- `R` 恢复输入 YAML 数值，`Backspace` 返回上一关节，`Esc` 取消。
- 无效数值不能确认。所有关节确认后才另存 reviewed 文件，取消不覆盖文件。

`hard-stop` 也会在使能前强制进入同一个确认界面；确认值用于零偏计算，
并另存到输出零偏文件旁的 `*_references.yaml`，不会修改原始配置或 URDF 限位。
机械硬限位参考角和软件限位不一定相同，超出 URDF 范围会警告，需核实机械结构。
`hard-stop` 仍是人工推到限位采样；推荐使用下一节的七关节全手动可视化入口。

带方向文件进行限位标定：

```bash
ros2 run ieir_controllers joint_zero_calibration \
  --mode hard-stop \
  --calibration-yaml src/ros2_ws_config/joint_calibration_dual_right.yaml \
  --directions-yaml src/ros2_ws_config/joint_directions_right.yaml \
  --output src/ros2_ws_config/joint_offsets_right.yaml
```

左臂执行同样两条命令，将三个文件名中的 `right` 改为 `left`。
限位正负指 URDF 方向，不是电机编码器方向；参考角仍须与实际机械限位对应。
计算 `zero_offset = raw_at_limit - axis_sign * urdf_pos_at_limit`，因此负方向关节
也能还原正确限位角。完成双臂后运行 `python3 src/ros2_ws_config/merge_offsets.py`。
新方向阶段确认的符号优先于旧合并文件，不会被历史符号覆盖。

### 七关节全手动标定

所有关节统一手动采样，不再使用自动寻限位、回零或二次寻限位。
默认顺序为 **7、6、5、3、4、2、1**；已有结果也会显示，由操作人按 S 跳过，
不会自动略过。使用 `--joint 4` 可单独选择任意一个关节，不限制先后顺序。

推荐入口为 `joint_manual_calibration`。旧名称 `joint_auto_calibration` 兼容同一个
全手动程序；旧的 `--auto-group` 会在使能前明确报错，不可能启动自动运动。
`--manual`、`--manual-group` 作为兼容参数可省略；后者现在也包含所有七个关节，
不再只代表 4、2、1。旧自动力矩、速度、摩擦前馈等命令行参数已移除。

先停控制器、遥操作、其他标定进程和命令发布者，只保留 bridge。
支撑所有关节，其他电机保持失能；当前关节也没有重力托举。
第 4 关节采样前，先手动摆好并支撑第 3 关节，确认不会撞支架。
已确认的方向与参考角文件继续复用，不修改已有零偏。

右臂启动示例：

```bash
source install/setup.bash
ros2 run ieir_controllers joint_manual_calibration \
  --calibration-yaml src/ros2_ws_config/joint_calibration_dual_right_reviewed.yaml \
  --directions-yaml src/ros2_ws_config/joint_directions_right.yaml \
  --output src/ros2_ws_config/joint_offsets_right.yaml
```

左臂使用对应的 left 文件；必须先有该臂已确认的方向、限位参考角，不能混用右臂文件。
单独重做某关节时在上述命令后加 `--joint 4` 等。

窗口操作：

1. 窗口显示当前关节的预期限位姿态。第一次 Enter 只使能这个电机取得零阻抗反馈。
2. 手动推至对应机械限位，停稳约 1 秒，Enter 采样并立即原子保存。
3. 模型开始显示按新零偏换算的实测运动。可手动移动观察，无需再确认、无需移动满 5 度。
4. N 失能当前关节并打开下一项，下一项仍需单独按 Enter 才会使能。
5. 采样前 S 跳过且保持原记录；保存后 N/S 都只切换下一项。
   Esc、关窗或 Ctrl-C 会退出；已经保存的记录不撤销。

全过程发送 `kp=kd=v_ff=tau_ff=0`，没有主动位置/力矩驱动。
发送层会拒绝非零增益、速度或力矩前馈。参数中不会读取或应用控制器增益、摩擦前馈。
零阻抗命令循环默认 `--rate 100` Hz，实际反馈频率由 bridge 决定；图形显示只有 FK，没有物理仿真。
采样前模型是预期参考姿态，采样后才是实际标定角度；其他模型关节保持零位，不用于碰撞验证。

零偏计算仍为 `zero_offset = raw_at_limit - axis_sign * urdf_pos_at_limit`。
保存项标记 `method: manual`、`reference_confirmed: true`、`visually_verified: false`，
不冒称经过二次视觉确认。只替换当前确认的关节，其他已有结果保留；七项齐全才标记 complete。
错误方向/文件、反馈失效、命令冲突等仍会拒绝或中止，失能未确认时会告警。
标定过程中不修改已经完成的右臂结果，除非操作人明确重新采样保存该关节。
双臂都完成后再运行 `python3 src/ros2_ws_config/merge_offsets.py`。

离线测试使用独立 ROS 域、假反馈，不连接 CAN：

```bash
python3 src/ieir_bringup/test/check_web_workflow.py
# 兼容 MuJoCo 窗口的测试已移至 simulation/ieir_simulation/test，需额外构建仿真包。
```

### 原单阶段流程（兼容保留）

摩擦参数已原样复用到 `src/ros2_ws_config/friction_model.yaml`。
不复制历史零偏；控制器在标定文件缺失或缺少当前关节时拒绝启动。
标定前只启动 bridge，不同时启动控制器或摩擦测试：

```bash
ros2 launch ieir_bringup bridge.launch.py arms:=dual
```

另一个终端 source 本工作区后标定：

```bash
ros2 run ieir_controllers joint_zero_calibration \
  --mode hard-stop \
  --calibration-yaml src/ros2_ws_config/joint_calibration_dual_left.yaml \
  --output src/ros2_ws_config/joint_offsets_left.yaml

ros2 run ieir_controllers joint_zero_calibration \
  --mode hard-stop \
  --calibration-yaml src/ros2_ws_config/joint_calibration_dual_right.yaml \
  --output src/ros2_ws_config/joint_offsets_right.yaml

python3 src/ros2_ws_config/merge_offsets.py
```

有零位夹具时可改成 `--mode zero-pose`。hard-stop 参考角度由输入文件初始化，
经预览修改和确认后使用，需与本机实际机械硬限位一致。标定会进入零阻抗，机械臂需要支撑。
合并脚本固定左 0 / 右 1。没有方向文件时仍默认 +1，并提示未验证；
复用历史符号时会按参考采样重新计算零偏，缺少参考数据则拒绝合并。
不要只修改 `axis_sign` 而不重算限位零偏，推荐上面的两阶段流程。

## 真机与遥操作

完成本机标定后，保持 bridge 运行，在第二个终端启动：

```bash
ros2 launch ieir_bringup real_robot.launch.py
# 或
ros2 launch ieir_bringup real_robot.launch.py controller:=cartesian_position
```

默认 `gripper:=false`。两端 UDP 遥操作原流程保留，例如：

```bash
# 主臂
ros2 launch ieir_bringup real_robot.launch.py \
  teleop:=true teleop_node_role:=master \
  teleop_peer_host:=192.168.10.20 teleop_local_port:=15000 teleop_peer_port:=15001
# 从臂
ros2 launch ieir_bringup real_robot.launch.py \
  teleop:=true teleop_node_role:=slave \
  teleop_peer_host:=192.168.10.10 teleop_local_port:=15001 teleop_peer_port:=15000
```

两端分别启动本机 bridge。主臂调用 `/teleop/prepare` 完成对齐，再用
`/teleop/toggle` 开始或暂停，`/teleop/disable` 停止。
没安装夹爪时遥操作不会发夹爪命令，也不要求夹爪反馈。

安装夹爪后，bridge 和控制器两个终端都传 `gripper:=true`：

```bash
ros2 launch ieir_bringup bridge.launch.py arms:=dual gripper:=true
ros2 launch ieir_bringup real_robot.launch.py gripper:=true
```

独立启动 `teleop.launch.py` 时也传 `gripper:=true`；
直接运行 `teleop_joint_bridge` 时使用 `--gripper true`。
这个开关同时决定真机模型是否装载左右夹爪子树，影响全部基于
`robot_description` 的 FK/重力计算，末端 attachment frame 保留。

## 验证边界

离线验证覆盖电机参数/协议编解码、左右通道选择、离线/非有限反馈过滤、
标定输出映射、夹爪开关及移除夹爪前后的 FK/质量/重力变化。
真实夹爪节点另用隔离 ROS 话题完成了模拟反馈测试：初始离线、使能、
收到有效反馈、进入标定、退出失能；此测试不连接 CAN。
尚需连接真实 CAN 验证接线、寄存器范围、重新标定、使能/失能和空载低速遥操作。
