# IEIR W3 双臂控制工作区

将原 USB2CAN 双臂控制迁移到 **W3 SocketCAN FD**：can0 左臂、can1 右臂，每臂 7 个达妙电机，**夹爪默认未装配/不启动**。
保留重力补偿、关节/笛卡尔控制、方向与全手动标定、示教回放和双机主从遥操作。
真机日常面板统一使用本地 Web UI，可管理控制端、全手动标定、遥操作和回放。
**真机控制、网页模型、标定、遥操作均不依赖 MuJoCo/GLFW。** 仿真已拆成可选 `ieir_simulation` 包，默认不安装、不构建。
不启动底盘、腰部、can2 或全身版 Web UI；本分支提供独立的本地双臂网页。

发布分支为 `feat/w3-dual-arm-can`，不覆盖原仓库 main。
不要混用 `ieir_dualarm_for_waic` 全身仓库的 install 或 launch。
此分支遥操作是两套机械臂之间的 UDP 关节跟随，**没有移植全身版本的 Pico、torso-aware 或舒适区协调层**。

- [完整算法、坐标和接口](docs/CONTROL_PIPELINE.md)
- [配置文件与标定产物](ros2_ws_config/README.md)
- [发布验证与已知限制](docs/RELEASE_CHECKLIST.md)
- [统一网页控制台](docs/WEB_CONSOLE.md)：手动启动 bridge 后，其余控制与标定在网页交互完成。
- [可选仿真包](simulation/README.md)：单独安装、构建和启动，不连接 CAN。
- [IEIR 更名迁移](docs/IEIR_MIGRATION.md)：旧工作区升级与远端仓库名称。
- [迁移和标定细节](W3_MIGRATION.md)

## 1. 安全边界

本项目是研究/调试软件，不是安全认证系统。没有碰撞检测、避障、箱体刚性约束或安全 PLC。
软件失能/watchdog 不替代物理急停、机械支撑和限位。

1. 初次启动前支撑双臂、保持空载、人员退出扫掠范围，核实急停有效。
2. 仓库不分发实测零偏、方向或已确认限位；这些文件只保存在本机并由 Git 忽略。每套机械臂必须自行标定、合并后再启动控制器，不能用全零文件代替标定。
3. 标定时停控制器、遥操作、回放和重复使能命令，只保留一个 bridge。
4. real_robot 会使能所选关节。重力模式不是位置锁定，不能保证松手不动。
5. gravity_gains、friction_gains、KP/KD 是样机调试值，不是任意装配的安全默认值。
6. 异常时先支撑并停止驱动，不要通过继续提高力矩、摩擦补偿或增益试错。

## 2. 环境与克隆

已验证基线：**Ubuntu 22.04 x86_64、ROS 2 Humble、系统 Python 3.10、GCC 11**。
仅可选仿真包附带 MuJoCo C SDK **3.3.0**，不需 pip 安装 mujoco，真机不加载该库。
Pinocchio 使用 ROS apt 包，当前验证版本 **3.9.0**；apt 候选版本会更新，
安装脚本不强制降级系统，其他大版本需重新构建和测试，不能假定已验证。
ARM64/Ubuntu 24.04/Jazzy 未验证，不要把附带 x86_64 库用于 ARM 工控机。

先按 [ROS 官方 Humble apt 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
安装 ROS（`ros-base` 即可，Web UI 不需要桌面、RViz 或 DISPLAY）。本仓库不自动改系统 apt 源或内核/CAN 驱动。
使用全新独立工作区，以下示例不要覆盖旧的 ~/ros2_ws：

```bash
mkdir -p ~/w3_dual_arm_ws
cd ~/w3_dual_arm_ws
git clone --branch feat/w3-dual-arm-can \
  https://github.com/lhp90281/buaa_ieir_dual_arm_robot.git src
```

仓库是源码根目录，克隆到 src，不含 build/install/log。
W3 源码已随仓库收录，不是 submodule，无需再克隆另一份 W3。

| 目录 | 内容 |
|---|---|
| W3_ROBOT/w3_robot_bridge | W3 消息、服务、协议、电机参数、双 CAN FD bridge |
| ieir_controllers | ros2_control、Pinocchio、硬件接口、标定、遥操作 |
| ieir_bringup | 真机、Web UI、示教和遥操作 launch |
| simulation/ieir_simulation | 可选独立仿真包，含 SDK、MJCF、仿真 launch/config/topic 硬件插件、兼容显示器 |
| description | 真机与网页共用的双臂/夹爪 URDF、原始 STL，不含仿真硬件插件 |
| ros2_ws_config | 运行时从源码读取的本机标定、摩擦参数和合并脚本 |
| USB2CAN | 历史参考，COLCON_IGNORE 排除，不参与新构建 |

## 3. 安装、构建、测试

使用新终端，退出 Conda/venv，不 source 旧工作区。依赖统一 apt/rosdep，
不要 sudo pip install，不要用 pip 的 pin/cmeel 覆盖 ROS Pinocchio。
以下安装需要网络和 sudo，**不会配置 CAN 或使能机器人**：

```bash
cd ~/w3_dual_arm_ws
bash src/scripts/install_dependencies.sh --install
bash src/scripts/build_workspace.sh "$PWD"
source install/setup.bash
```

安装脚本要求已有 /opt/ros/humble/setup.bash，安装编译工具、rosdep、ros2_control、
Pinocchio、CAN 工具，再按 package.xml 补齐依赖；默认不安装仿真图形依赖。
只检查不安装：`bash src/scripts/install_dependencies.sh --check`。
网络、apt 源、rosdep 失败会停止，不静默跳过依赖。

构建脚本只用 Humble underlay，清除旧 overlay 路径影响；默认顺序编译、每包 1 个编译任务，
减少 Pinocchio 内存峰值。内存足够可用 `JOBS=2 bash src/scripts/build_workspace.sh "$PWD"`。
手工入口为 `colcon build --base-paths src --executor sequential`。
嵌套工作区若父目录有 COLCON_IGNORE，仍须显式给出 base-paths。
`src/simulation/COLCON_IGNORE` 只阻止默认递归发现仿真；显式 `--with-simulation` 才加入该包。
旧版本升级必须清理旧构建缓存，见 [更名迁移](docs/IEIR_MIGRATION.md)，不要继续叠加旧包的 install。

```bash
# 以下测试不连接 CAN
colcon test --base-paths src --packages-select w3_robot_bridge ieir_controllers ieir_bringup --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg prefix w3_robot_bridge
ros2 pkg prefix ieir_controllers
```

prefix 必须指向本工作区。每个新运行终端执行 `cd ~/w3_dual_arm_ws; source install/setup.bash`。
改 C++、launch、包内 YAML 后重新构建；改 src/ros2_ws_config 不需编译，
但需重新合并零偏（如适用）并重启控制器。不要让全身版与本分支叠加到同一终端。

## 4. CAN FD 与 Bridge

当前使用 W3 支持的 KAS-ROBOT-CANFD 适配器，必须由驱动呈现 SocketCAN 网络接口。
can-utils 不会替代适配器驱动/固件。先确认 can0/can1 存在，换 USB 口后重新核对物理通道。
每条总线 ID=1..7，对应 slot=0..6；bitrate/dbitrate、终端电阻、供电和模式必须匹配。
**只在控制已停止、关节已支撑时配置总线**：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 txqueuelen 1000
sudo ip link set can1 up
ip -details -statistics link show can0
ip -details -statistics link show can1
```

本配置为 CAN FD，不是旧 USB2CAN 的经典 CAN。
终端 A 保留运行：

```bash
ros2 launch ieir_bringup bridge.launch.py arms:=dual gripper:=false
```

默认 arms 已统一为 dual，排查单臂显式用 left/right；
right 始终是 channel=1/can1，不会重编号。bridge 不自动使能。
无命令 0.5 秒后 watchdog 切阻尼，**阻尼不是失能或姿态保持**。

```bash
ros2 topic info /w3_robot_bridge_node/state --verbose
ros2 topic echo /w3_robot_bridge_node/state --qos-reliability best_effort --once
```

使能前可能没有有效反馈；结合 online/enabled/error_flags 判断，不能只看灯色。
不要同时运行旧 USB2CAN 或多个 W3 bridge。

<a id="calibration"></a>
## 5. 完整标定

顺序为方向 -> 理论参考角 -> 七关节手动限位 -> 双臂合并。
只运行 bridge。左右臂分别进行，已完成的同一台机器不必重做。

```bash
cd ~/w3_dual_arm_ws
source install/setup.bash
C="$PWD/src/ros2_ws_config"
cp -a "$C" "$HOME/w3-calibration-backup-$(date +%Y%m%d-%H%M%S)"
SIDE=left  # 完成后改 right，核对对应物理臂
```

### 5.1 方向（已有正确 directions 文件可跳过）

```bash
ros2 run ieir_controllers joint_zero_calibration \
  --mode direction \
  --calibration-yaml "$C/joint_calibration_dual_${SIDE}.yaml" \
  --output "$C/joint_directions_${SIDE}.yaml"
```

此阶段使能所选臂全部电机，零阻抗、没有重力托举，**支撑整臂**。
实物放近模型零位，窗口 Enter 记录增量起点，手动转动约 5 度；
相反按空格，一致 Enter 确认下一关节。R 重做起点，Backspace 返回，Esc 中止。
全部确认且失能后才写方向文件。近似零位不作为最终零偏。

### 5.2 限位参考角（只显示，无电机命令）

```bash
ros2 run ieir_controllers joint_zero_calibration \
  --mode limit-preview \
  --calibration-yaml "$C/joint_calibration_dual_${SIDE}.yaml" \
  --output "$C/joint_calibration_dual_${SIDE}_reviewed.yaml"
```

按 7、6、5、3、4、2、1 查看预期限位角。
E 输入角度，Enter 结束编辑，再 Enter 确认；方向键微调 1 度、Shift 微调 0.1 度。
全部确认生成 reviewed 文件。机械硬限位可能不同于 URDF 软件限位；
视觉姿态不能证明实际无碰撞。

### 5.3 全手动零偏采样

```bash
ros2 run ieir_controllers joint_manual_calibration \
  --calibration-yaml "$C/joint_calibration_dual_${SIDE}_reviewed.yaml" \
  --directions-yaml "$C/joint_directions_${SIDE}.yaml" \
  --output "$C/joint_offsets_${SIDE}.yaml"
```

默认 **7 -> 6 -> 5 -> 3 -> 4 -> 2 -> 1**；单独重做加 `--joint 4`。
其他关节失能并支撑，第 4 关节先手动调整并支撑第 3 关节，避免撞支架。

| 操作 | 行为 |
|---|---|
| 首次 Enter | 仅使能当前关节、零阻抗反馈；模型显示预期限位角 |
| 手推到限位、停稳约 1 秒，再 Enter | 立即保存零偏；无自动回零、无二次寻限位 |
| 保存后手动移动 | 模型显示按新零偏换算的实测角度 |
| N | 失能当前关节，进入下一项；下一项仍需 Enter 使能 |
| S | 显式跳过、保留旧记录，不自动跳过已完成项 |
| Esc/关窗/Ctrl-C | 退出并尝试失能；已保存的记录保留 |

全过程 kp=kd=v_ff=tau_ff=0，无摩擦前馈、无主动运动。
旧 joint_auto_calibration 是同一手动入口；--auto-group 已禁用。
每次 Enter 自动更新单臂文件，七项齐全标记 complete，不需要单臂合并。

### 5.4 双臂合并、生效

```bash
python3 "$C/merge_offsets.py"
```

读取左右两个七关节文件，生成 joint_offsets_dual.yaml，成功打印 Merged 14 W3 joints。
缺文件、缺关节、错误名字/slot/通道会拒绝合并。不要伪造未标定臂数据填满。
重标后**重新合并、重启控制器，无需编译**；单臂可直接用本臂 offsets。
W3 motor YAML 保持 position_offset=0、signs=+1，仅硬件接口做一次符号和零偏变换。

## 6. 真机控制与 UI

推荐：终端 A 手动启动 bridge，终端 B 只启动总 UI（本步骤不使能电机）：

```bash
cd ~/w3_dual_arm_ws  # 本机已有目录为 ~/ros2_ws/w3_dual_arm_ws 时使用该路径
source install/setup.bash
# 终端 A
ros2 launch ieir_bringup bridge.launch.py arms:=dual gripper:=false
# 终端 B，同样 cd/source 后
ros2 launch ieir_bringup ui.launch.py workspace:="$PWD" gripper:=false
```

打开 http://127.0.0.1:8766 ，解锁后在“运行总览”选择机械臂并确认“启动控制端”，此时才使能电机。
页面可切模式、回零、启停遥操作/回放；停止控制端后可直接在标定页运行方向确认、限位编辑、手动采样及合并。
所有关节先支撑，控制和标定互斥。无夹爪默认不加载夹爪质量。详见[网页完整流程](docs/WEB_CONSOLE.md)。
网页启动控制端时内部使用 `use_web:=false`，不递归启动第二个网页。外部控制器必须由原终端停止，网页不会强杀外部进程。

兼容方式：不用网页管理进程时，退出标定、保留 bridge，在终端 B 直接启动控制器
（**以下命令会立即使能真机，不是只读窗口**）：

```bash
cd ~/w3_dual_arm_ws
source install/setup.bash
ros2 launch ieir_bringup real_robot.launch.py \
  arms:=dual controller:=gravity gripper:=false use_web:=true web_allow_control:=true \
  offsets_yaml:="$PWD/src/ros2_ws_config/joint_offsets_dual.yaml" \
  friction_model_yaml:="$PWD/src/ros2_ws_config/friction_model.yaml"
```

默认激活 joint_state_broadcaster 和 gravity_compensation_controller，
关节/笛卡尔控制器加载但 inactive。重力控制器在位置模式下继续提供前馈。
首次只验证标定姿态时，可在上述命令加 `gravity_compensation:=false`，同时保持
`controller:=gravity gripper:=false teleop:=false`：仍使能反馈，但不激活力矩/位置控制器。
此时必须支撑全部关节，并设置 `web_allow_control:=false`，仅查看反馈，不要从其他节点切换模式。
- gripper=false 删除控制 URDF 的夹爪子树与质量，不只是停止节点。
- Web UI 使用 URDF/STL，`gripper` 参数同步传给网页显示。
- 单臂用 arms:=right 和对应 joint_offsets_right.yaml，bridge 也用 right。
- 笛卡尔模式仅支持 dual；勿重复启动 real_robot，勿与仿真共用 ROS 域。

打开 http://127.0.0.1:8766 。页面默认锁定，开启“允许控制”后逐次确认操作。
`use_web` 默认开启，但 `web_allow_control` 默认 false（只读）；无网页可用 `use_web:=false`。
旧 `use_gui:=true/false` 仅作为 `use_web` 默认值的兼容参数，不再启动 MuJoCo 真机面板。
不要同时启动两个同端口网页服务，端口占用用 `web_port:=8767`。

已有控制端，只补 UI（不会启动控制器、使能电机）：

```bash
ros2 launch ieir_bringup web_console.launch.py \
  workspace:="$PWD" port:=8766 allow_control:=true gripper:=false
```

网页只订阅实测，不运行物理或发布 joint_states。先核对模型与实物是否一致。
姿态使用 30 Hz 独立流和浏览器显示插值；日志/表格保持约 4 Hz，插值不改变控制命令。

| 页面入口 | 影响 |
|---|---|
| 总览 → 重力补偿 | 切换重力模式，不等于失能 |
| 总览 → 关节位置 / 笛卡尔 | 切换位置接口，保留重力前馈 |
| 总览 → 左臂 / 右臂 / 双臂回零 | 自动切关节位置，默认 8 秒回模型零位，大行程自动延长，需确认路径 |
| 关节控制 | 读取当前、编辑草稿、确认后发送目标和插值时间 |
| 主从遥操作 | 准备对齐 / 开始 / 暂停 / 退出 |

回零不修改电机零偏，不做碰撞规划。关闭或锁定页面不停止已有轨迹和遥操作。

用 ros2 control list_controllers 查看活跃状态。
正常停机：支撑，停止遥操作/回放，停止控制端并确认失能，最后停 bridge。
杀进程/Ctrl-C 不等同物理急停，掉电也可能重力下落。

## 7. 关节、笛卡尔与示教

启动选择 controller:=joint_position 或 cartesian_position；已有网页在总览切换，
不要再启动第二套控制器。关节/笛卡尔互斥，均与重力 effort 前馈共存。

/joint_position_command 接收 JointTrajectory，关节名 left_joint_0..6/right_joint_0..6，
角度 rad，time_from_start 秒/纳秒，内部时间分段线性插值、末点保持。
不是碰撞规划器，没有通用 jerk 限制；小范围验证优先用网页关节控制草稿，确认后下发。

笛卡尔输入 /cartesian_position_controller/{left,right}_target_pose 为 PoseStamped，
**数值须已在 base_footprint 下**，不能只改 frame_id 期待 TF 换算。
末端 frame 为 {left,right}_attachment_point。交互入口：
`ros2 run ieir_controllers cartesian_keyboard_control`。
当前是 IK 后的关节限速插值，不是全身版本的“末端增量+时间”接口，也不是双臂刚性约束。

示教/回放需已有 bridge 和控制端。拖动录制时用重力模式：

```bash
ros2 launch ieir_bringup record.launch.py output:=recordings/demo.yaml
# Ctrl-C 保存，确认扫掠空间后回放：
ros2 launch ieir_bringup replay.launch.py input:=recordings/demo.yaml time_scale:=0.5 ramp_in:=5.0
# 仅模型零位无干涉时使用：
ros2 launch ieir_bringup go_home.launch.py duration:=8.0
```

回放默认切关节控制、保留重力，结束恢复重力。录制文件不随仓库提交。

<a id="teleoperation"></a>
## 8. 双机主从遥操作

两台机器各自完成本机标定，只经 UDP 交换 URDF 关节角，**不是 raw 编码器角**，
因此 offsets 不能互用。两边都运行本分支。ROS 图无需跨机互通：
主机 ROS_DOMAIN_ID=41，从机=42；每台机器所有本地终端都用自己的同一域。

每台终端 A source 后 export 对应域，再启动 bridge：
`ros2 launch ieir_bringup bridge.launch.py arms:=dual gripper:=false`。
示例 IP 按实际网卡修改，放行对应 UDP 端口：

```bash
# 主机 192.168.10.10，终端 B 已 source 本工作区
export ROS_DOMAIN_ID=41
ros2 launch ieir_bringup real_robot.launch.py \
  arms:=dual gripper:=false use_web:=true web_allow_control:=true teleop:=true \
  teleop_node_role:=master teleop_mode:=no_feedback \
  teleop_peer_host:=192.168.10.20 teleop_local_port:=15000 teleop_peer_port:=15001
```

```bash
# 从机 192.168.10.20，终端 B 已 source 本工作区
export ROS_DOMAIN_ID=42
ros2 launch ieir_bringup real_robot.launch.py \
  arms:=dual gripper:=false use_web:=true web_allow_control:=true teleop:=true \
  teleop_node_role:=slave teleop_mode:=no_feedback \
  teleop_peer_host:=192.168.10.10 teleop_local_port:=15001 teleop_peer_port:=15000
```

主机网页进入“主从遥操作”，选择“准备对齐”，**主臂自动约 5 秒对齐从臂**；
确认后“开始跟随”，可“暂停跟随”或“退出遥操作”。
准备对齐也会驱动主臂。无网页时主机 prepare 后 toggle；任意端可 disable/exit：

```bash
ros2 service call /teleop/prepare std_srvs/srv/Trigger {}
ros2 service call /teleop/toggle std_srvs/srv/Trigger {}
ros2 service call /teleop/disable std_srvs/srv/Trigger {}
ros2 service call /teleop/exit std_srvs/srv/Trigger {}
```

no_feedback：主臂对齐后回重力，发送实测关节角，从臂位置跟随。
默认 50 Hz、超时 0.3 s、启动误差 0.5 rad、运行误差 1 rad、每周期目标步长 0.03 rad。
这些不保证急停/制动距离。UDP 无认证/加密，只在受信局域网或受保护隧道内使用，不暴露公网。
force_feedback 是位置耦合实验，不是真实力传感器闭环，不推荐日常使用。
teleop_role 是增益配置，teleop_node_role 才是网络角色；当前 master/slave 增益数值相同，
不要凭配置名假设主臂更软。gripper=false 时不发夹爪控制。

## 9. 纯仿真与可选夹爪

仿真不需要 bridge、CAN 或 offsets。仿真代码可以不部署到工控机。
需要仿真时显式安装和构建，再用独立 ROS 域，避免影响真机：

```bash
bash src/scripts/install_dependencies.sh --install --with-simulation
bash src/scripts/build_workspace.sh "$PWD" --with-simulation
source install/setup.bash
export ROS_DOMAIN_ID=60
ros2 launch ieir_simulation system.launch.py controller_type:=joint_position
# 或 controller_type:=gravity_compensation / cartesian_position
```

/ctrl/command 和 /ctrl/gains 计算 MIT 力矩，只验证接口/轨迹，不代表准确真机动力学。
纯仿真不要启动面向 W3 的夹爪节点。仿真 MJCF 自带夹爪惯性，
没有与真机 gripper=false 完全一致的惯性自动切换，不能据此标定真实前馈。
仿真入口已移除启动真实夹爪节点的选项。文件位置、独立显示及兼容标定入口见 [仿真手册](simulation/README.md)。

装上夹爪后，bridge 和 real_robot 同时指定 gripper:=true，
加入每臂 slot7/ID8、夹爪模型质量和独立控制器。
启动包含开合行程标定，清空夹持区域。未装夹爪不得开启。
参数见 ieir_controllers/config/gripper_controller.yaml；
交互入口 `ros2 run ieir_controllers gripper_keyboard_control`。

## 10. 排障

| 现象 | 优先检查 |
|---|---|
| 找不到命令/参数不同 | ros2 pkg prefix，是否 source 全身版或旧 install |
| rosdep glfw3 报错 | 本分支已改为 libglfw3-dev，检查是否用了旧 package.xml |
| Pinocchio/eigenpy/NumPy ABI | 退出 Conda，不混用 pip/cmeel，系统 Python + ROS apt 重建 |
| libmujoco 缺失 | 只影响可选仿真；按仿真手册重建 ieir_simulation，勿替换为不同 SDK ABI |
| CAN ENOBUFS/bus-off | 电源、接线、电阻、位速率、ACK；不能只增加队列掩盖 |
| no-data | can0/1 物理对应、FD模式、ID、使能反馈、固件量程 |
| QoS RELIABILITY 不兼容 | state 用 best_effort，echo 显式指定 |
| 角度/力矩尺度错误 | PMAX/VMAX/TMAX 与电机固件一致，ranges 不是任意安全限幅 |
| 方向对、姿态偏 | reference、offset、合并文件是否最新，是否本机数据 |
| 重力下上翻/自运动 | 支撑并停止，查姿态、夹爪质量、重力/摩擦倍率及力矩映射 |
| 标定不能使能 | 其他电机未失能、多命令发布者、控制器/遥操作仍运行 |
| 标定后仍读旧数据 | merge，检查启动输出的绝对 offsets 路径，重启控制端 |
| 遥操作 enable 失败 | prepare 对齐、本机 ROS 域、UDP 地址/端口、关节名字集合 |
| SSH 无 GUI | 总 UI 与网页标定不需要 DISPLAY；仅原 MuJoCo 标定显示需图形环境。远程浏览仅用受信 SSH 本地端口转发 |

`ros2 launch ... --show-args` 可查看安装版本参数，不启动节点。
更多真实边界、审查结果和测试记录见 [发布检查](docs/RELEASE_CHECKLIST.md)。
