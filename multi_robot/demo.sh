#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
source ./env.sh
exec python3 scripts/one_click_demo.py "$@"
