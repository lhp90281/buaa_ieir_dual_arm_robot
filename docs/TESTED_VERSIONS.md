# 本次验证环境快照

2026-09-24，Ubuntu 22.04 x86_64。这是可追溯的测试环境记录，
不是承诺这些精确 deb 版本永远可从 apt 镜像安装，也不是要求对现有系统强制降级。
新机器使用 ROS Humble apt 包后，先运行完整编译/测试，再进行低风险实机验证。

| 包 | 本机 deb 版本 |
|---|---|
| cmake | 3.22.1-1ubuntu1.22.04.2 |
| gcc / g++ 元包 | 4:11.2.0-1ubuntu1 |
| libglfw3-dev | 3.3.6-1 |
| libyaml-cpp-dev | 0.7.0+dfsg-8build1 |
| python3-numpy | 1:1.21.5-1ubuntu22.04.1 |
| python3-yaml | 5.4.1-1ubuntu1 |
| ros-humble-eigenpy | 3.12.0-1jammy.20260226.010915 |
| ros-humble-pinocchio | 3.9.0-1jammy.20260304.203533 |
| ros-humble-ros2-control | 2.54.0-1jammy.20260505.183920 |
| ros-humble-ros2-controllers | 2.53.1-1jammy.20260505.184646 |
| 可选 ieir_simulation 的 MuJoCo C SDK | 3.3.0，mj_version()=330；真机无需安装 |

特别注意 apt 当前候选 Pinocchio 可能不是 3.9.0；本次没有修改主机 apt、
没有在全新 OS 虚拟机上联网安装验证所有候选版本。独立源码副本构建仍复用了本机系统依赖。
若要严格复现，部署时记录自己的 dpkg-query 输出并保留合法获得的系统镜像/软件包缓存。
