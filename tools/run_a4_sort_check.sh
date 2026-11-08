#!/bin/bash
# A4 (and the real A3 validation): run the sort task in the A2 cell with bins published
# as MoveIt collision objects, then score by GROUND TRUTH.
#   (a) ground truth : tools/check_sort.py -- is every object in its correct bin?
#   (b) log health   : scan for errors / planning failures / tracebacks
#   (c) consecutive  : the sort moves several objects in one run; repeat the whole run
#                      with $1 to prove stability.
# No `set -u` (ROS setup.bash).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
RUNS="${1:-1}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export TWIN_CELL_COLLISION=1          # A3: bins ARE collision objects for the sort
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

LAUNCH="$MIRROR/install/twin_action_demo/share/twin_action_demo/launch/industrial_cell_a2_bringup.launch.py"

for run in $(seq 1 "$RUNS"); do
  echo "############ SORT RUN $run/$RUNS ############"
  LPGID=""
  cleanup(){ [ -n "$LPGID" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "$LPGID" ] && kill -KILL -"$LPGID" 2>/dev/null; }
  trap cleanup EXIT

  setsid ros2 launch "$LAUNCH" headless:=true > "$LOGDIR/a4_bringup_$run.log" 2>&1 &
  LPGID=$!

  ok=0
  for i in $(seq 1 60); do
    n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
    if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
    sleep 2
  done
  [ "$ok" = 1 ] || { echo "!! controllers never came active"; cleanup; exit 2; }

  for i in $(seq 1 12); do
    timeout 6 ros2 topic echo /detected_objects/array --once 2>/dev/null | grep -q 'detections' && break
  done
  sleep 3

  echo "--- ground truth BEFORE sort ---"
  python3 "$TOOLS/check_sort.py" 2>&1 | tail -6

  echo "--- running sort_task ---"
  timeout 900 ros2 run twin_moveit_scripts sort_task > "$LOGDIR/a4_sort_$run.log" 2>&1
  echo "sort_task rc=$? (245 = benign teardown segv)"
  grep -E '^SORT |placed \(ground truth' "$LOGDIR/a4_sort_$run.log" | tail -8

  echo "--- GROUND TRUTH AFTER sort (this is the verdict) ---"
  python3 "$TOOLS/check_sort.py" 2>&1 | tail -8

  echo "--- log health ---"
  for pair in "ERRORs(non-benign):\[ERROR\].*(move_to|sort_task)" \
              "planning failures:Planning FAILED|returned no trajectory" \
              "tracebacks:Traceback \(most recent" \
              "gripper open retries:OPEN did not take" \
              "arrival misses:did not arrive"; do
    lbl="${pair%%:*}"; pat="${pair#*:}"
    printf '%-24s %s\n' "$lbl" "$(grep -cE "$pat" "$LOGDIR/a4_sort_$run.log" 2>/dev/null)"
  done

  cleanup; trap - EXIT; sleep 3
done
echo "##### A4 SORT CHECK DONE. Logs: $LOGDIR"
