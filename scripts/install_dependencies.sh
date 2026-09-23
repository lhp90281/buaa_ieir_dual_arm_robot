#!/usr/bin/env bash
# System packages only: no pip, motor commands, CAN setup, or ROS nodes.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:---check}
if [[ $# -gt 1 || ( "$mode" != --check && "$mode" != --install ) ]]; then
  echo "Usage: bash scripts/install_dependencies.sh [--check|--install]" >&2
  exit 2
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || $(uname -m) != x86_64 ]]; then
  echo "Supported baseline: Ubuntu 22.04 x86_64, ROS 2 Humble. See README.md." >&2
  exit 2
fi
if [[ -n ${CONDA_PREFIX:-} || -n ${VIRTUAL_ENV:-} ]]; then
  echo "Deactivate Conda/venv first; ROS uses /usr/bin/python3." >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
  exit 2
fi
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [[ "$mode" == --install ]]; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake git python3-colcon-common-extensions \
    python3-rosdep python3-pytest can-utils iproute2 libglfw3-dev libgl1-mesa-dev \
    ros-humble-pinocchio ros-humble-ros2-control ros-humble-ros2-controllers
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update --rosdistro humble
  # Explicit roots keep historical USB2CAN out of dependency discovery too.
  rosdep install --from-paths "$root/eiriarm_controllers" "$root/eiriarm_mujoco" \
    "$root/eiriarm_bringup" "$root/description" "$root/W3_ROBOT" \
    --ignore-src --rosdistro humble -y
fi
rosdep check --from-paths "$root/eiriarm_controllers" "$root/eiriarm_mujoco" \
  "$root/eiriarm_bringup" "$root/description" "$root/W3_ROBOT" \
  --ignore-src --rosdistro humble
/usr/bin/python3 -c 'import rclpy, yaml, numpy, pinocchio; print("Python dependencies OK; Pinocchio", pinocchio.__version__)'
libraries=$(ldd "$root/eiriarm_mujoco/third_party/mujoco/lib/libmujoco.so.3.3.0")
printf '%s\n' "$libraries"
if [[ "$libraries" == *"not found"* ]]; then
  echo "Missing MuJoCo runtime library dependency." >&2
  exit 1
fi
