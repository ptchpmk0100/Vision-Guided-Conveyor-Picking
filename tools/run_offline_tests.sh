#!/bin/bash
# B5: offline test suite — everything that runs WITHOUT Gazebo/a sim (so it is CI-able).
# Covers the three component self-tests + a py_compile of every package/tool module.
# The full ground-truth sort checks need a running simulator and are NOT part of this suite
# (see tools/run_sort_line_check.sh). No `set -u` (ROS setup.bash).
set -o pipefail
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
PKG="$MIRROR/src/twin_moveit_scripts/twin_moveit_scripts"
source /opt/ros/jazzy/setup.bash 2>/dev/null
[ -f "$MIRROR/install/setup.bash" ] && source "$MIRROR/install/setup.bash" 2>/dev/null
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-/tmp/offline_pyc}"
export PYTHONPATH="$MIRROR/src/twin_moveit_scripts:$TOOLS:$PYTHONPATH"

fail=0
run () {  # $1 = label, rest = command
  local label="$1"; shift
  if "$@" >/tmp/offline_$$.log 2>&1; then
    echo "PASS  $label"
  else
    echo "FAIL  $label"; tail -3 /tmp/offline_$$.log | sed 's/^/        /'; fail=1
  fi
}

echo "===== OFFLINE TEST SUITE (no simulator) ====="
# 1. component self-tests
run "behaviour-tree self-test"  python3 "$PKG/behaviour_tree.py" --self-test
run "validator self-test"       python3 "$PKG/sorting_validator.py" --self-test
run "extrinsic-solver self-test" python3 "$TOOLS/fit_extrinsic_calibration.py" --self-test
run "parcel geometry and depth perception" python3 "$TOOLS/test_parcel_cell.py"
run "parcel grasp confirmation and loss" python3 "$TOOLS/test_parcel_grasp.py"
run "parcel appearance variants keep identical physics" python3 "$TOOLS/test_parcel_variants.py"
run "condition classifier crop geometry" python3 "$TOOLS/test_parcel_condition.py"
run "condition routing: votes, router, bin world" python3 "$TOOLS/test_parcel_reject.py"
run "ten-parcel capacity perception and contact IDs" python3 "$TOOLS/test_ten_parcel.py"
run "motion experiment preflight and planner options" python3 "$TOOLS/test_motion_audit.py"
run "motion validation prevents dispatch" python3 "$TOOLS/test_motion_dispatch.py"
run "early launch, slot confirmation and release stroke" python3 "$TOOLS/test_pick_gate.py"
run "shadow arrival predicate" python3 "$TOOLS/test_arrival_gate.py"
run "saved controller spline reconstruction" python3 "$TOOLS/test_toppra_path.py"
run "Pilz sequence contact and blend contracts" python3 "$TOOLS/test_pilz_sequence.py"

# 2. py_compile every package module + tool script
mods=()
for f in "$PKG"/*.py "$TOOLS"/*.py; do mods+=("$f"); done
run "py_compile all modules"    python3 -m py_compile "${mods[@]}"

echo "=============================================="
if [ "$fail" = 0 ]; then echo "OFFLINE SUITE: ALL PASS"; else echo "OFFLINE SUITE: FAILURES ABOVE"; fi
exit "$fail"
