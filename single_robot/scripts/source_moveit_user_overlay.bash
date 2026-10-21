#!/usr/bin/env bash

_moveit_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_moveit_workspace_root="$(cd -- "$_moveit_script_dir/.." && pwd)"
_moveit_prefix="$_moveit_workspace_root/.moveit-overlay/opt/ros/humble"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed under /opt/ros/humble." >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -f "$_moveit_prefix/share/moveit_task_constructor_demo/package.xml" ]]; then
  echo "Run scripts/install_moveit_user_overlay.sh first." >&2
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$_moveit_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$_moveit_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$_moveit_prefix/lib:$_moveit_prefix/lib/x86_64-linux-gnu:$_moveit_workspace_root/.moveit-overlay/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$_moveit_prefix/lib/python3.10/site-packages:$_moveit_prefix/local/lib/python3.10/dist-packages:$_moveit_workspace_root/.moveit-overlay/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$_moveit_prefix/bin:$PATH"
unset _moveit_script_dir _moveit_workspace_root _moveit_prefix
