#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
source ./env.sh
if [[ ! -f install/fleet_bringup/share/fleet_bringup/package.xml || ! -f vendor/runtime/lib/libtf2.so || ! -f install/fleet_control/share/fleet_control/web/app.js || ! -f install/fleet_control/share/fleet_control/web/style.css ]]; then
  ./build.sh
  source ./env.sh
fi
exec python3 scripts/supervise.py "$@"
