#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Keep all relative paths in the ROS parameters rooted at this repository.
cd "${REPO_ROOT}"

# ROS's generated setup files reference optional environment variables.  Source
# them with nounset disabled, then restore strict mode for the launcher itself.
set +u
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
if [[ -f "${REPO_ROOT}/ros_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ros_ws/install/setup.bash"
fi
set -u

ros2 launch vision_bringup zed_rtabmap.launch.py "$@"
