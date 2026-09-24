# IEIR 名称与仿真拆分

## 新名称

ROS 包统一为 `ieir_controllers`、`ieir_bringup`、可选 `ieir_simulation`。
C++ 命名空间、头文件目录、插件导出、launch/配置引用、Web UI 标题和命令同步更名。
关节名、ROS 话题/服务名、W3 消息和 CAN 通道不变，已有本机标定结果无需重标。
`ros2_ws_config` 内被 Git 忽略的实测文件不参与改名或上传。

旧包名不再提供别名；不要混用旧 `eiriarm_*` 安装和新 `ieir_*` 源码。
原全身工作区及 `ieir_dualarm_for_waic` 仓库不在本次迁移范围。

## 现有工作区升级

源码改名不会更新已经安装的脚本和插件。先按原停机流程停止控制、标定、遥操作、UI 和 bridge，
支撑机械臂并确认失能。以下操作**不要在真机进程运行期间执行**。
打开新终端，不 source 旧工作区；在 `w3_dual_arm_ws` 根目录执行：

```bash
backup="log/pre_ieir_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup"
for dir in build install; do
  if [ -d "$dir" ]; then mv "$dir" "$backup/"; fi
done
bash src/scripts/install_dependencies.sh --install
bash src/scripts/build_workspace.sh "$PWD"
source install/setup.bash
ros2 pkg prefix ieir_controllers
ros2 pkg prefix ieir_bringup
```

备份可回退，不删除本机 `src/ros2_ws_config` 数据。`ieir_simulation` 默认不在新安装中，
如需使用，按 [仿真手册](../simulation/README.md) 显式构建。

升级后的真机入口仍是两个终端：

```bash
# 终端 1，先按主 README 配好 CAN
source install/setup.bash
ros2 launch ieir_bringup bridge.launch.py arms:=dual gripper:=false

# 终端 2
source install/setup.bash
ros2 launch ieir_bringup ui.launch.py workspace:="$PWD" gripper:=false
```

打开 http://127.0.0.1:8766，在网页中显式确认启动控制器或标定；打开页面不会自动使能。
Web UI 使用原始 STL 和浏览器 Three.js，不依赖 MuJoCo。

## GitHub

目标仓库名为 `lhp90281/buaa_ieir_dual_arm_robot`，继续使用 `feat/w3-dual-arm-can` 分支。
GitHub Settings 中完成仓库更名后，在源码根目录更新原双臂远端：

```bash
git remote set-url old-origin git@github.com:lhp90281/buaa_ieir_dual_arm_robot.git
git remote -v
```

不要将指向全身仓库的 `origin` 当作本分支发布目标。更名本身不提交、不推送本机未提交的源码。
