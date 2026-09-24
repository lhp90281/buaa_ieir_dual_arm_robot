# W3 双臂发布检查

目标：原 `lhp90281/buaa_ieir_dual_arm_robot` 的 `feat/w3-dual-arm-can` 分支。
不覆盖 main，不推送 `ieir_dualarm_for_waic`，不 force push。

## 打包范围

- W3 bridge 源码/消息/电机配置随仓库收录，不依赖另一个未克隆的嵌套仓库。
- description 包含共用 STL/URDF；可选 simulation/ieir_simulation 包含 MJCF 和 MuJoCo 3.3.0 SDK。真机默认不构建/安装仿真依赖。
- 旧 USB2CAN 仅供追溯，以 COLCON_IGNORE 排除；不启动 can2、腰部、底盘或全身版 Web UI。当前双臂版使用独立 Web UI。
- 不提交 build/install/log、缓存、录制轨迹、虚拟环境或凭据。
- 本机实测方向、零偏、合并零偏和已确认限位只保留在本地，由 Git 忽略，不进入发布提交历史。首次克隆必须完成本机标定。

## 分发修复

- ROS 包、插件、源码、命令与 UI 统一为 IEIR；真机控制/网页/默认标定不依赖 MuJoCo。
- 仿真和旧 RViz 显示集中在可选 `simulation/ieir_simulation`，默认安装/构建跳过。

- 根 README 成为完整启动手册，bringup README 指向它，避免多个入口说明相互冲突。
- 默认 bridge arms 从临时 right 改回 dual，与 real_robot 一致；gripper 保持 false。
- 修复无效 rosdep 键 glfw3 -> libglfw3-dev，去掉与内置 SDK 冲突的外部 mujoco 依赖。
- 补齐 controller_manager、joint_state_broadcaster、launch、模型和 Python 测试依赖。
- ROSIDL member_of_group 放到 package.xml 正确层级，去掉跳过校验。
- MuJoCo 3.3.0 库安装到包的 lib，执行文件用相对 RPATH，不再只依赖源目录中的库。
- 安装脚本使用 apt/rosdep，不混装 pip 的 ROS/Pinocchio/MuJoCo；构建脚本隔离旧 overlay。

## 已知边界与审查结果

1. 未建立完整运行态逐关节反馈超时停车联锁；命令 watchdog 不代表反馈健康，详见算法文档。
2. 笛卡尔输入必须已在 base_footprint；当前代码不按 header.frame_id 做 TF 转换。
3. 关节轨迹是线性插值，笛卡尔是 IK 后关节限速，没有碰撞/jerk/严格奇异裕量保护。
4. 纯仿真 MJCF 的夹爪质量与真机 gripper=false 模型不自动同步；UI 外观不能用于判断控制模型质量。
5. 当前 master/slave 增益相同；force_feedback 是位置耦合实验，非真实外力闭环。
6. 夹爪质量剔除、方向/offset 一次变换有离线回归；真实电机协议量程、装配和负载仍需现场验证。
7. UDP 遥操作只用于受信网络；无认证/加密，禁止直接暴露公网。
8. 已验证平台仅 Ubuntu 22.04 x86_64 + Humble。当前 Pinocchio=3.9.0；apt 新大版本未宣称兼容。
9. 部分历史包 license 仍为 TODO，不能把整个仓库当作已统一授予某一种开源许可证。

## 发布验证

2026-09-24，IEIR 更名及仿真拆分后，在独立 `/tmp/ieir-validation` 的 build/install 验证，未覆盖工作区原安装：

- 默认 7 个真机 ROS 包全新编译、安装成功，未安装 ieir_simulation；显式添加仿真后共 8 包构建成功。
- colcon 汇总 197 tests，0 errors、0 failures、0 skipped（含 Python 用例和 CTest 包装项）。
- 真机 6 个控制/硬件动态库可加载，ldd 不含 MuJoCo。
- W3 协议测试覆盖全部 16 个电机配置；迁移、标定和分发回归通过。
- Web 标定在隔离 ROS 域 218、临时文件、假反馈下完成方向、限位编辑、全手动采样、失联退出及双臂合并。
- 网页管理按钮的确认/取消、互斥、标定阶段切换及桌面/手机布局通过 Playwright 检查；原始 STL 保持 19 个模型部件、537034 个三角面。
- 默认 rosdep check 通过；源码中全部 MJCF 和安装后的完整模型均能加载。
- 无 GUI 的仿真插件测试在域 221 中验证 topic 硬件接口及重力/关节控制器 active，输出有限的 MIT 命令且无 CAN 节点。
- 安装后的 MuJoCo 程序通过相对 RUNPATH 加载安装目录中的动态库。
- 新名称 ui.launch.py 的 --show-args 检查、launch 语法与 git diff --check 通过。
- MuJoCo 完整图形窗口未在本轮运行；未进行真机运动验证。

这不是全新操作系统安装测试：独立构建复用了本机系统依赖。确切版本见
[已验证依赖版本](TESTED_VERSIONS.md)，没有宣称未经测试的新 Pinocchio 大版本兼容。
上述结果不能替代真机功能与安全验收。

测试输入在临时目录合成，不改动或打包本机实测标定。
编译仍有上游/既有 unused-parameter、MuJoCo UI initializer 等警告；
ROS Web 流程退出时偶见 rclpy Destroyable 清理告警，退出码与断连失能测试通过，未作为本次改名附带重构。

本次不运行真实 CAN、使能、运动或遥操作；实机“现在正常”为操作者反馈，非本次自动测试结论。

## 第三方来源

- W3 bridge 源自 https://github.com/PeterZFY/W3_ROBOT ，本分支修改为双臂配置及 raw 坐标适配。
- MuJoCo SDK 来源 https://github.com/google-deepmind/mujoco ，构建固定用 3.3.0；
  third_party 下保留 Apache-2.0 LICENSE 与第三方说明，其他存留版本库不参与本次链接。
- lodepng 保留其 LICENSE；tabulate 文件保留原文件版权声明。
- ROS、Pinocchio 等从系统 ROS apt 安装，具体许可证随各上游包；不重新授予第三方许可。
