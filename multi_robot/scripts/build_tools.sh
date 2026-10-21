#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")/.."
mkdir -p build/tools
g++ -std=c++17 -O2 tools/truth_probe.cpp -o build/tools/truth_probe $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)
