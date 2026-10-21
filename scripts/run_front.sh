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

# One folder per run. Override the location with VISION_LOG_ROOT.
LOG_ROOT="${VISION_LOG_ROOT:-${HOME}/.ros/robosub/logs}"
RUN_DIR="${LOG_ROOT}/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "${RUN_DIR}/ros"
export ROS_LOG_DIR="${RUN_DIR}/ros"
# Without this, piping into tee can make the terminal output arrive in bursts.
export PYTHONUNBUFFERED=1

# Everything inside the braces is shown on screen and also saved to console.log.
{
  echo "[run_front] repo: ${REPO_ROOT}"
  echo "[run_front] launch args: ${*:-<defaults>}"
  echo "[run_front] logs: ${RUN_DIR}"
  ros2 launch vision_bringup zed_rtabmap.launch.py "$@"
} 2>&1 | tee "${RUN_DIR}/console.log"
