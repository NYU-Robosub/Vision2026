#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
if [[ -f "${REPO_ROOT}/ros_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ros_ws/install/setup.bash"
fi

ros2 launch vision_bringup slam_live.launch.py "$@"
