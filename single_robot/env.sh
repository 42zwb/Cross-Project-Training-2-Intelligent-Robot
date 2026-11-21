#!/usr/bin/env bash
# 在每个单机终端加载同一 ROS 域和 Gazebo 分区。
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '%s\n' '请在 Bash 终端运行 source ./env.sh' >&2
  exit 1
fi

tb4_single_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1
if [[ ! -f "$tb4_single_root/install/local_setup.bash" ]]; then
  printf '%s\n' '请先在 single_robot 目录用 colcon build 构建工作区。' >&2
  unset tb4_single_root
  return 1
fi

source /opt/ros/humble/setup.bash || return 1
source "$tb4_single_root/install/local_setup.bash" || return 1
unset tb4_single_root
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${TB4_DEMO_DOMAIN_ID:-74}"
export IGN_PARTITION="${TB4_DEMO_PARTITION:-tb4_single_demo}"
export GZ_PARTITION="$IGN_PARTITION"
