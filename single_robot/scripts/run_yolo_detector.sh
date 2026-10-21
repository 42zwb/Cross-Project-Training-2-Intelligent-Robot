#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$workspace_root/.venv-yolo/bin/python" ]]; then
  echo "Create .venv-yolo and install src/turtlebot4_tutorial/requirements-yolo.txt first." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$workspace_root/.venv-yolo/bin/activate"
source "$workspace_root/install/setup.bash"
export PYTHONNOUSERSITE=1
exec "$workspace_root/.venv-yolo/bin/python" -m turtlebot4_tutorial.yolo_detector "$@"
