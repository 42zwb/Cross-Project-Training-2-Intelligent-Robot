#!/usr/bin/env bash
# Isolated environment: no dependency on the single-robot tutorial workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
FLEET_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$FLEET_ROOT/install/setup.bash" ]] && source "$FLEET_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${FLEET_DOMAIN_ID:-91}"
export RMW_IMPLEMENTATION="${FLEET_RMW:-rmw_cyclonedds_cpp}"
# This project runs three peers and one simulator on the same host. Bind both
# transports to loopback so a Wi-Fi address change cannot split a live run.
export ROS_LOCALHOST_ONLY=1
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]; then
 # Humble's RMW injects another lo entry when ROS_LOCALHOST_ONLY=1; use
 # one explicit interface in XML instead of selecting lo twice.
 export ROS_LOCALHOST_ONLY=0
 export CYCLONEDDS_URI="file://$FLEET_ROOT/src/fleet_bringup/config/cyclonedds-local.xml"
fi
export IGN_IP=127.0.0.1
export GZ_IP="$IGN_IP"
export IGN_PARTITION="${FLEET_IGN_PARTITION:-tb4_course_fleet}"
export GZ_PARTITION="$IGN_PARTITION"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export FLEET_ROOT

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

# Project-local ABI-compatible tf2 lock-order fix; system ROS is untouched.
if [[ -f "$FLEET_ROOT/vendor/runtime/lib/libtf2.so" ]]; then
 export LD_LIBRARY_PATH="$FLEET_ROOT/vendor/runtime/lib:$LD_LIBRARY_PATH"
fi
