#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
source ./env.sh
cmake -S vendor/tf2_compat -B build/tf2_compat -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$FLEET_ROOT/vendor/runtime"
cmake --build build/tf2_compat --parallel 2
cmake --install build/tf2_compat
colcon build --base-paths src --symlink-install --parallel-workers 2 --event-handlers desktop_notification- "$@"
