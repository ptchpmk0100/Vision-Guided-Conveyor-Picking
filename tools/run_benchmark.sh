#!/bin/bash
# A5 SORTING BENCHMARK HARNESS.
# Runs the two scenarios the roadmap asks for, each repeated N times (consecutive-run
# validation), recording every trial and every ground-truth verdict to SQLite, then
# prints the metrics report.
#   S1  single sort   : sort ONE object            (sort_task --max 1)
#   S2  colour sort    : sort the whole 3-colour set (sort_task, no cap)
#
# Three-legged validation, every run:
#   (a) GROUND TRUTH  -- benchmark.py reads Gazebo poses before & after each run.
#   (b) LOG HEALTH    -- tightened scan below (the task's OWN honest 'pick/place failed'
#                        reports are NOT system errors and are excluded -- that was the
#                        over-count wart in run_a4_sort_check.sh).
#   (c) CONSECUTIVE   -- N>=2 runs per scenario; the report aggregates them.
#
# Usage:  run_benchmark.sh [runs_per_scenario]      (default 2)
# No `set -u` (ROS setup.bash trips on unbound vars).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
N="${1:-2}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
DB="${TWIN_DB:-$LOGDIR/benchmark.db}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export TWIN_CELL_COLLISION=1          # A3: bins ARE collision objects for the sort
export TWIN_DB="$DB"
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

LAUNCH="$MIRROR/install/twin_action_demo/share/twin_action_demo/launch/industrial_cell_a2_bringup.launch.py"

# Fresh dataset for this benchmark invocation (keep the previous one as .bak).
[ -f "$DB" ] && mv -f "$DB" "$DB.bak"
rm -f "$DB-wal" "$DB-shm"

log_health () {   # $1 = sort log file
  # Counts DISTINCT health signals. Three classes of line are deliberately NOT counted
  # as "system errors" because they are not independent failures:
  #   - the task's OWN honest reports ('sort_task: pick/place failed') -- those are the
  #     outcome, already in the metrics;
  #   - MoveIt/OMPL planner-internal chatter (RRTConnect / planner_manager /
  #     model_based_planning_context / "Planner '..' failed" / "Planning FAILED") -- one
  #     logical plan failure emits ~8 of these; they are counted ONCE under "planning
  #     failures" instead;
  #   - the benign one-shot octomap "No 3D sensor plugin" notice at bringup.
  local f="$1"
  local sys trc fat plan grip arr
  local PLANNER='RRTConnect|planner_manager|model_based_planning_context|Planner .* failed|Planning FAILED|Unable to (sample|solve)|Insufficient states|returned no trajectory'
  local BENIGN='No 3D sensor plugin'
  sys=$(grep -E '\[ERROR\]' "$f" 2>/dev/null \
        | grep -vE 'sort_task: (pick|place) failed' \
        | grep -vE "$PLANNER" | grep -vcE "$BENIGN")
  trc=$(grep -cE 'Traceback \(most recent' "$f" 2>/dev/null)
  fat=$(grep -cE '\[FATAL\]|terminate called' "$f" 2>/dev/null)
  plan=$(grep -cE '\[ERROR\].*Planning FAILED' "$f" 2>/dev/null)
  grip=$(grep -cE 'OPEN did not take' "$f" 2>/dev/null)
  arr=$(grep -cE 'did not arrive' "$f" 2>/dev/null)
  printf '  %-26s %s\n' "system ERRORs(unexpected):" "$sys"
  printf '  %-26s %s\n' "tracebacks:" "$trc"
  printf '  %-26s %s\n' "fatal/terminate:" "$fat"
  printf '  %-26s %s\n' "planning failures:" "$plan"
  printf '  %-26s %s\n' "gripper open retries:" "$grip"
  printf '  %-26s %s\n' "arrival misses:" "$arr"
}

run_scenario () {   # $1 = scenario tag, $2 = extra sort args
  local scen="$1" extra="$2"
  export TWIN_SCENARIO="$scen"
  export TWIN_LAUNCH="industrial_cell_a2_bringup"
  for run in $(seq 1 "$N"); do
    local RID="${scen}_$(date +%H%M%S)_$run"
    export TWIN_RUN_ID="$RID"
    echo "############ $scen RUN $run/$N  ($RID) ############"
    LPGID=""
    cleanup(){ [ -n "$LPGID" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "$LPGID" ] && kill -KILL -"$LPGID" 2>/dev/null; }
    trap cleanup EXIT

    setsid ros2 launch "$LAUNCH" headless:=true > "$LOGDIR/bench_bringup_${RID}.log" 2>&1 &
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

    echo "--- ground truth BEFORE ---"
    python3 "$TOOLS/benchmark.py" truth --run "$RID" --phase before --db "$DB"

    echo "--- running sort_task $extra ---"
    timeout 900 ros2 run twin_moveit_scripts sort_task $extra > "$LOGDIR/bench_sort_${RID}.log" 2>&1
    echo "sort_task rc=$? (245 = benign teardown segv)"
    grep -E '^SORT |placed \(ground truth' "$LOGDIR/bench_sort_${RID}.log" | tail -8

    echo "--- ground truth AFTER (this is the verdict) ---"
    python3 "$TOOLS/benchmark.py" truth --run "$RID" --phase after --db "$DB"

    echo "--- log health ---"
    log_health "$LOGDIR/bench_sort_${RID}.log"

    cleanup; trap - EXIT; sleep 3
  done
}

run_scenario S1 "--max 1"
run_scenario S2 ""

echo
echo "##### BENCHMARK RUNS DONE -- metrics report #####"
python3 "$TOOLS/benchmark.py" report --db "$DB"
echo "DB: $DB   (logs in $LOGDIR/bench_*.log)"
