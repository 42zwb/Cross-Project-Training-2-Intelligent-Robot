#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
source ./env.sh
if [[ ! -f install/fleet_bringup/share/fleet_bringup/package.xml || ! -f vendor/runtime/lib/libtf2.so ]]; then
  ./build.sh
  source ./env.sh
fi
exec python3 scripts/supervise.py "$@"
