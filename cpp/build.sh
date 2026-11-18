#!/bin/bash
# Builds cpp/mapper_core.cpp into scripts/mapper_core*.so (pybind11; python3-pybind11 / pybind11-dev).
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out="$here/../scripts/mapper_core$(python3-config --extension-suffix)"
c++ -O3 -Wall -shared -std=c++17 -fPIC $(python3 -m pybind11 --includes) "$here/mapper_core.cpp" -o "$out"
echo "built $out"
