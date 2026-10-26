#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")/.."
source ./env.sh
export PYTHONPATH="$FLEET_ROOT/src/fleet_control:$FLEET_ROOT/src/fleet_mapping:$PYTHONPATH"
python3 -m pytest -q tests
timeout 20 ./install/fleet_bridge/lib/fleet_bridge/bridge_lifecycle_probe
timeout 3 ./build/tf2_compat/tf2_lock_regression
