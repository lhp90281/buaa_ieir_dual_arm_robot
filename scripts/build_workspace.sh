#!/usr/bin/env bash
# Run in a child shell so an old workspace cannot enter the new build underlay.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then
  echo "Usage: bash scripts/build_workspace.sh WORKSPACE [colcon build options...]"
  echo "Example: bash src/scripts/build_workspace.sh \"\$PWD\"; JOBS=1 by default."
  exit 0
fi
if [[ $# -lt 1 || ! -d "$1" ]]; then
  echo "Pass the existing workspace directory, not the repository directory." >&2
  exit 2
fi
workspace=$(cd "$1" && pwd)
shift
if [[ "$workspace" == "$root" ]]; then
  echo "Keep build/install/log outside the source repository." >&2
  exit 2
fi
if [[ -n ${CONDA_PREFIX:-} || -n ${VIRTUAL_ENV:-} ]]; then
  echo "Deactivate Conda/venv before building." >&2
  exit 2
fi
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset COLCON_CURRENT_PREFIX PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=${JOBS:-1}
export MAKEFLAGS="-j${JOBS:-1}"
cd "$workspace"
colcon build --base-paths "$root" --executor sequential "$@"
