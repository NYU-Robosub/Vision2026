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
RUN_DIR="${LOG_ROOT}/$(date -u +%Y-%m-%d_%H-%M-%S)"
mkdir -p "${RUN_DIR}/ros"
export ROS_LOG_DIR="${RUN_DIR}/ros"
export VISION_RUN_DIR="${RUN_DIR}"
# Without this, piping into tee can make the terminal output arrive in bursts.
export PYTHONUNBUFFERED=1

START_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
GIT_REV="$(git -C "${REPO_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain 2>/dev/null)" ]]; then
  GIT_DIRTY=true
else
  GIT_DIRTY=false
fi

# First argument: exit status, or empty while the run is in progress.
# The remaining arguments are the launch arguments.
write_metadata() {
  RUN_DIR="${RUN_DIR}" START_UTC="${START_UTC}" GIT_REV="${GIT_REV}" \
  GIT_BRANCH="${GIT_BRANCH}" GIT_DIRTY="${GIT_DIRTY}" EXIT_STATUS="${1:-}" \
  python3 - "${@:2}" <<'PY'
import datetime
import json
import os
import sys

meta = {
    'run_dir': os.environ['RUN_DIR'],
    'started_utc': os.environ['START_UTC'],
    'git_revision': os.environ['GIT_REV'],
    'git_branch': os.environ['GIT_BRANCH'],
    'git_uncommitted_changes': os.environ['GIT_DIRTY'] == 'true',
    'launch_args': sys.argv[1:],
    'host': os.uname().nodename,
    'exit_status': None,
}
status = os.environ.get('EXIT_STATUS', '')
if status != '':
    meta['exit_status'] = int(status)
    meta['finished_utc'] = datetime.datetime.now(
        datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
with open(os.path.join(os.environ['RUN_DIR'], 'metadata.json'), 'w') as handle:
    json.dump(meta, handle, indent=2)
    handle.write('\n')
PY
}

write_metadata "" "$@"
trap 'write_metadata "$?" "$@"' EXIT

# Everything inside the braces is shown on screen and also saved to console.log.
{
  echo "[run_front] repo: ${REPO_ROOT}"
  echo "[run_front] launch args: ${*:-<defaults>}"
  echo "[run_front] logs: ${RUN_DIR}"
  python3 "${SCRIPT_DIR}/vision_logs.py" prune --protect "${RUN_DIR}" \
    || echo "[run_front] WARNING: could not prune old logs; continuing."
  MONITOR_PID=""
  if [[ "${VISION_STATUS_MONITOR:-1}" != "0" ]] && ros2 pkg prefix vision_status >/dev/null 2>&1; then
    echo "[run_front] starting runtime status monitor (VISION_STATUS_MONITOR=0 turns it off)"
    setsid ros2 run vision_status vision_status --ros-args \
      -p rgb_topic:=/zed_front/zed_node/rgb/color/rect/image \
      -p depth_topic:=/zed_front/zed_node/depth/depth_registered \
      -p camera_info_topic:=/zed_front/zed_node/rgb/color/rect/camera_info \
      -p odometry_topic:=/zed_front/zed_node/odom &
    MONITOR_PID=$!
  else
    echo "[run_front] runtime status monitor not started (vision_status not installed, or disabled)"
  fi
  # A failing launch must still stop the monitor and report its own exit status.
  set +e
  ros2 launch vision_bringup zed_rtabmap.launch.py "$@"
  LAUNCH_STATUS=$?
  set -e
  if [[ -n "${MONITOR_PID}" ]]; then
    # setsid gave the monitor its own process group; stop the whole group.
    kill -TERM -- "-${MONITOR_PID}" 2>/dev/null || kill -TERM "${MONITOR_PID}" 2>/dev/null || true
    wait "${MONITOR_PID}" 2>/dev/null || true
  fi
  exit "${LAUNCH_STATUS}"
} 2>&1 | tee "${RUN_DIR}/console.log"
