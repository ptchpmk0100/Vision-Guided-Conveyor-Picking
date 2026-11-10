#!/bin/bash
# Clean-shell entry: source this checkout only; Python owns and cleans marked PIDs.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_VERSION=harmonic
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib | sort | paste -sd:)"
exec python3 tools/run_parcel_cell_check.py "$@"
