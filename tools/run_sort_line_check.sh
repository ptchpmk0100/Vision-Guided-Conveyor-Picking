#!/bin/bash
# Stage C/D sort runner: bring up the raised sorting line with the detector, run the
# sorting-line sort task (which adds the tray collision scene itself), and score PLACEMENT
# by Gazebo ground truth. Three legs: (a) ground truth --placed, (b) tightened log health,
# (c) $1 consecutive runs. No `set -u` (ROS setup.bash).
#   usage: run_sort_line_check.sh [runs] [sort_args]
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
RUNS="${1:-2}"
SORTARGS="${2:-}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
DB="${TWIN_DB:-$LOGDIR/sline_benchmark.db}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export TWIN_DB="$DB"
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

[ -f "$DB" ] && mv -f "$DB" "$DB.bak"; rm -f "$DB-wal" "$DB-shm"

log_health () {
  local f="$1" PLANNER='RRTConnect|planner_manager|model_based_planning_context|Planner .* failed|Planning FAILED|Unable to (sample|solve)|Insufficient states|returned no trajectory'
  local sys plan trc fat
  sys=$(grep -E '\[ERROR\]' "$f" 2>/dev/null | grep -vE 'sort_line: (pick|place) failed' | grep -vE "$PLANNER" | grep -vcE 'No 3D sensor plugin')
  plan=$(grep -cE '\[ERROR\].*Planning FAILED' "$f" 2>/dev/null)
  trc=$(grep -cE 'Traceback \(most recent' "$f" 2>/dev/null)
  fat=$(grep -cE '\[FATAL\]|terminate called' "$f" 2>/dev/null)
  printf '  system ERRORs(non-task): %s\n  planning failures: %s\n  tracebacks: %s\n  fatal: %s\n' "$sys" "$plan" "$trc" "$fat"
}

for run in $(seq 1 "$RUNS"); do
  RID="sline_$(date +%H%M%S)_$run"
  export TWIN_RUN_ID="$RID" TWIN_SCENARIO="sline" TWIN_LAUNCH="industrial_sorting_line"
  echo "############ SORT LINE RUN $run/$RUNS ($RID) ############"
  LPGID=""
  cleanup(){ [ -n "$LPGID" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "$LPGID" ] && kill -KILL -"$LPGID" 2>/dev/null; }
  trap cleanup EXIT
  setsid ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py \
    headless:=true detector:=true ${TWIN_EXTRINSIC:+extrinsic_file:="$TWIN_EXTRINSIC"} \
    > "$LOGDIR/sline_bringup_$run.log" 2>&1 &
  LPGID=$!

  ok=0
  for i in $(seq 1 60); do
    n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
    if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
    sleep 2
  done
  [ "$ok" = 1 ] || { echo "!! controllers never came active"; cleanup; exit 2; }
  for i in $(seq 1 15); do
    timeout 6 ros2 topic echo /detected_objects/array --once 2>/dev/null | grep -q 'detections' && break
  done
  sleep 3

  echo "--- ground truth BEFORE (staging) ---"
  python3 "$TOOLS/check_sorting_line.py" 2>&1 | tail -6
  SORTEXE="${SORTEXE:-sort_task_sorting_line}"
  echo "--- running $SORTEXE $SORTARGS ---"
  timeout 900 ros2 run twin_moveit_scripts "$SORTEXE" $SORTARGS > "$LOGDIR/sline_sort_$run.log" 2>&1
  echo "sort rc=$? (245 = benign teardown segv)"
  grep -E '^SORTLINE |placed \(ground' "$LOGDIR/sline_sort_$run.log" | tail -6
  echo "--- GROUND TRUTH AFTER (placed; the verdict) ---"
  python3 "$TOOLS/check_sorting_line.py" --placed 2>&1 | tail -8
  echo "--- log health ---"
  log_health "$LOGDIR/sline_sort_$run.log"
  cleanup; trap - EXIT; sleep 3
done
echo "##### SORT LINE CHECK DONE. DB: $DB  Logs: $LOGDIR"
