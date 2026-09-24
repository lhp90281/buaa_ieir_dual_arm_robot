# IEIR 可选仿真

`ieir_simulation` 是同仓库中的独立 ROS 包。它依赖共享 `ieir_controllers` 算法和 description 模型；
真机包不反向依赖它。`simulation/COLCON_IGNORE` 使默认构建、依赖安装和测试跳过仿真。
工控机无需安装 MuJoCo、GLFW、OpenGL 或 Python mujoco，也无需运行此包。

## 安装与构建

在工作区根目录、无旧 overlay 的新终端执行：

```bash
bash src/scripts/install_dependencies.sh --install --with-simulation
bash src/scripts/build_workspace.sh "$PWD" --with-simulation
source install/setup.bash
```

脚本显式加入 `src/simulation/ieir_simulation`，不需要删除 COLCON_IGNORE。
仍使用内置 Linux x86_64 MuJoCo C SDK 3.3.0，不能随意替换动态库版本；ARM64 未验证。
不使用 pip MuJoCo。模型显示需要可用的 OpenGL/桌面环境。

## 启动

无需 bridge、CAN 配置、使能或本机标定文件。必须与真机使用不同的 ROS_DOMAIN_ID：

```bash
export ROS_DOMAIN_ID=60
source install/setup.bash
ros2 launch ieir_simulation system.launch.py controller_type:=joint_position
# controller_type:=gravity_compensation 或 cartesian_position
```

模拟器默认暂停，模型加载后在 MuJoCo 窗口中解除暂停。
只启动模拟器：`ros2 launch ieir_simulation mujoco_sim.launch.py`。
连接已有模拟器：`ros2 launch ieir_simulation controllers.launch.py controller_type:=joint_position`。
仿真 launch 不会启动真实 W3 电机/夹爪驱动。

## 文件和控制链

- `ieir_simulation/mjcf/dual_arm_robot.xml`：仿真源模型；安装后位于 `share/ieir_simulation/mjcf/`。
- `ieir_simulation/config/simulate.yaml`：模型 URI、反馈和命令 topic。
- `ieir_simulation/config/dual_arm_sim_ros2_control.urdf.xacro`：仿真专用硬件配置。
- `ieir_simulation/config/dual_arm_sim_controllers.yaml`：共享控制器的仿真参数。
- `ieir_simulation/TopicBasedHardwareInterface`：MIT 命令和增益经 `/ctrl/command`、`/ctrl/gains` 送模拟器。
- 模拟器执行 `tau = torque_ff + kp*(q_des-q) + kd*(v_des-v)`，发布 `/joint_states`。
- `meshes` 在源码中链接共享 description/STL；安装时复制到本包，避免安装模型依赖源码绝对路径。

仿真只验证接口和轨迹，不是精确动力学模型。当前模型带夹爪惯性，不能用来直接标定未装夹爪真机的前馈。
Web 真机模型仍按 `gripper:=false` 移除夹爪质量，与仿真包是否安装无关。

## 兼容显示器

日常真机只用 `ieir_bringup ui.launch.py`。历史 MuJoCo 面板和标定窗口仅作为显式选择保留：

```bash
ros2 launch ieir_simulation mujoco_panel.launch.py
# 标定命令额外指定 --preview-backend mujoco
```

兼容面板不是纯只读：编辑/发送快捷键会向当前 ROS 域中的控制器发送目标，不要误接真机域。
默认手动标定使用网页，不会调用以上显示器。

历史 RViz 模型展示也归入可选包，例如
`ros2 launch ieir_simulation dual_arm_support_display_robot.launch.py`。
真机的 description 包只保留模型资源，不再强制安装 RViz 或关节状态 GUI。

## 测试

```bash
colcon test --base-paths src src/simulation/ieir_simulation \
  --packages-select ieir_simulation --event-handlers console_direct+
colcon test-result --verbose
```

包内测试只加载模型、解析 launch/plugin，不连接 CAN、不发送电机命令。
历史 GUI 标定假电机测试位于 `ieir_simulation/test/check_*`，需要图形环境和隔离的 ROS 域。
无图形环境也可用合成反馈测试真实插件加载：
`python3 src/simulation/ieir_simulation/test/check_controllers.py`，该脚本固定使用回环 ROS 域 221。
