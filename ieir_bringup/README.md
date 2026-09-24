# IEIR Bringup (W3 双臂)

本包只负责真机和 Web UI，不依赖 MuJoCo。仿真请见
[`ieir_simulation`](../simulation/README.md)，旧版本升级请先看 [更名迁移](../docs/IEIR_MIGRATION.md)。

完整且唯一的启动手册已整理到 [仓库根 README](../README.md)。
不要再使用旧 USB2CAN 或全身腰部仓库的启动命令。

- [安装与构建](../README.md#3-安装构建测试)
- [CAN 与 bridge](../README.md#4-can-fd-与-bridge)
- [完整方向/全手动标定/合并](../README.md#calibration)
- [真机与 Web UI](../README.md#6-真机控制与-ui)
- [统一网页控制台](../docs/WEB_CONSOLE.md)（管理控制器、标定、遥操作、回放；不管理 bridge）
- [主从遥操作](../README.md#teleoperation)
- [控制算法与已知安全边界](../docs/CONTROL_PIPELINE.md)

日常入口（每个终端都从 W3 工作区根目录 source install/setup.bash）：

```bash
# 终端 A，只启动一次
ros2 launch ieir_bringup bridge.launch.py arms:=dual gripper:=false

# 终端 B，只启动总 UI；打开页面不使能电机
ros2 launch ieir_bringup ui.launch.py workspace:="$PWD" gripper:=false
```

解锁网页后，“运行总览”选择双臂/单臂并确认“启动控制端”，这一步才使能并启动重力补偿。
默认读取工作区 src/ros2_ws_config/joint_offsets_dual.yaml；单臂选择读取相应 joint_offsets_left/right.yaml。
摩擦文件为 src/ros2_ws_config/friction_model.yaml。只开右臂须同时修改 bridge 的 arms:=right 与网页手臂选择。
没有装夹爪时，不要为了让 GUI 外观匹配而打开 gripper。
浏览器打开 http://127.0.0.1:8766 ，开启页面控制权限后逐次确认操作。
总览包含左臂/右臂/双臂回零。`use_gui` 兼容参数现在也只启动 Web UI，不再启动 MuJoCo 真机面板。
网页还可启动/停止遥操作和回放。标定前支撑全部关节，先停止遥操作/回放，再停止控制端。
进入标定页选手臂与阶段即可交互执行，参考角也在网页编辑；不用另开终端或 MuJoCo 窗口。
两臂完成后显式执行“双臂合并”，再回总览启动控制端，无需重新编译。
关闭浏览器不停止已发轨迹；网页模式标定例外，失去可见页面心跳会退出并尝试失能。
停止 UI 服务会停止其自有控制端，须先支撑全臂。外部节点、bridge 不会被网页终止。

只增加网页，不启动任何控制节点：

```bash
ros2 launch ieir_bringup web_console.launch.py workspace:="$PWD" port:=8766
```

访问 http://127.0.0.1:8766 。已有启动附带的网页时不要重复运行。明确需要网页控制时加 `allow_control:=true`，
再在页面开启控制并确认每次操作。纯界面预览使用 `preview:=true`，不连接 ROS。
