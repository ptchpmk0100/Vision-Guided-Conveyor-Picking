#!/bin/bash
# Run the C++ condition node's gtests (built with colcon) and fail if any failed or was skipped.
# Needs ROS sourced, the package built in this workspace, and CONDITION_MODEL set (else the classifier tests skip).
set -eo pipefail
cd "$(dirname "$0")/.."
colcon test --base-paths src --packages-select twin_condition_cpp --event-handlers console_direct+
colcon test-result --verbose
# the classifier tests skip without a model; here a skip means the model is missing, so fail loudly
if grep -rl 'result="skipped"' build/twin_condition_cpp/test_results; then echo "C++ TESTS: SKIPPED"; exit 1; fi
echo "C++ TESTS: ALL PASS"
