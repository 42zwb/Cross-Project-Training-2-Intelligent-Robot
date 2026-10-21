#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cache_root="$workspace_root/.moveit-cache"
overlay_root="$workspace_root/.moveit-overlay"
temporary_root="$workspace_root/.moveit-overlay.tmp.$$"

if [[ -d "$overlay_root/opt/ros/humble/share/moveit_task_constructor_demo" \
    && -d "$overlay_root/opt/ros/humble/share/moveit_task_constructor_visualization" ]]; then
  echo "MoveIt user overlay is already available at $overlay_root"
  exit 0
fi

mkdir -p "$cache_root/archives/partial"
apt-get -y --download-only --no-install-recommends \
  -o "Dir::Cache::archives=$cache_root/archives" \
  -o Debug::NoLocking=1 \
  install ros-humble-moveit \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-moveit-task-constructor-demo \
    ros-humble-moveit-task-constructor-visualization

rm -rf "$temporary_root"
mkdir -p "$temporary_root"
found_deb=false
for package_file in "$cache_root"/archives/*.deb; do
  [[ -f "$package_file" ]] || continue
  found_deb=true
  dpkg-deb -x "$package_file" "$temporary_root"
done

if [[ "$found_deb" != true || ! -f "$temporary_root/opt/ros/humble/share/moveit_task_constructor_demo/package.xml" ]]; then
  rm -rf "$temporary_root"
  echo "MoveIt packages were not extracted; check the apt download output." >&2
  exit 1
fi

mv "$temporary_root" "$overlay_root"
echo "Extracted MoveIt and Panda/MTC packages into $overlay_root without changing dpkg state."
echo "In each ROS terminal, run: source $workspace_root/scripts/source_moveit_user_overlay.bash"
