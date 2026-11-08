#!/bin/bash
# Durable ground-truth harness for the v2 Sorting Cell work.
# Replaces the throwaway session-scratchpad scripts so ANY future session can verify.
#
#   tools/run_groundtruth.sh <launch-file> [cycles] [tag]
#
#   <launch-file>  a headless launch (see tools/verify_*_headless.launch.py), or a
#                  package launch name is NOT supported -- pass a path.
#   [cycles]       pick_and_place cycles to run (default 2)
#   [tag]          log-name prefix (default: basename of the launch)
#
# Judges success by GAZEBO GROUND TRUTH, never by log lines: every status message in
# this project has lied at least once (HELD on an empty hand, place=True on an
# unmoved cube). Ignore the rc=245 teardown segfault -- it fires after "Done.".
#
# NOTE: no `set -u` -- ROS setup.bash references unset vars and would abort.

TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
LAUNCH="${1:?usage: run_groundtruth.sh <launch-file> [cycles] [tag]}"
NRUNS="${2:-2}"
TAG="${3:-$(basename "$LAUNCH" .launch.py)}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

# ground-truth pose of a named entity (default "b"): first 3 numbers = position x y z
truth(){ local n="${1:-b}"; timeout 8 gz topic -e -n 1 -t /world/empty/dynamic_pose/info 2>/dev/null \
  | grep -A5 "name: \"$n\"" | grep -E " [xyz]:" | awk '{print $2}' | head -3 | tr '\n' ' '; }

cleanup(){ echo ">>> cleanup"; [ -n "${LPGID:-}" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "${LPGID:-}" ] && kill -KILL -"$LPGID" 2>/dev/null; }
trap cleanup EXIT

echo ">>> [$TAG] launching headless: $LAUNCH"
setsid ros2 launch "$LAUNCH" > "$LOGDIR/${TAG}_bringup.log" 2>&1 &
LPGID=$!

echo ">>> waiting for controllers active (<=120s)"
ok=0
for i in $(seq 1 60); do
  n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
  if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
  sleep 2
done
[ "$ok" = 1 ] || { echo "!! controllers never came active"; exit 2; }

echo ">>> waiting for a detection to publish (<=40s)"
det=0
for i in $(seq 1 8); do
  if timeout 8 ros2 topic echo /detected_objects/box --once 2>/dev/null | grep -q 'position'; then det=1; echo "detector publishing after ~$((i*8))s"; break; fi
done
[ "$det" = 1 ] || echo "!! WARNING: nothing on /detected_objects/box (ok if this launch uses the multi-object topic)"

sleep 3
echo "=== SPAWN ground truth: [$(truth)] ==="
for r in $(seq 1 "$NRUNS"); do
  PRE="$(truth)"; L="$LOGDIR/${TAG}_cyc$r.log"
  echo ">>> RUN $r : pick_and_place box (no respawn)"
  timeout 240 ros2 run twin_moveit_scripts move_to pick_and_place box > "$L" 2>&1; rc=$?
  sleep 4; POST="$(truth)"
  DET="$(grep -oE "live 'box' @ \([-0-9. ,]+\)" "$L" | tail -1)"
  CHOSE="$(grep -oE "drop slot [0-9]+ @ \([-0-9. ,]+\)" "$L" | tail -1)"
  CYC="$(grep -E 'Cycle result' "$L" | tail -1 | sed 's/.*Cycle result: //')"
  PARK="$(grep -c 'returning to safe' "$L")"
  echo "RUN $r | rc=$rc | pre=[$PRE] detected=[$DET] chose=[$CHOSE] -> post=[$POST] | cycle: $CYC | parked=$PARK"
done

# ---- leg (b): LOG VALIDATION -------------------------------------------------
# Ground truth decides SUCCESS; logs are how we detect SYSTEM HEALTH problems.
# rc=245 / "Segmentation fault" is the known-benign MoveItCpp teardown (fires after
# "Done.") and is reported separately so it never masks a real fault.
echo "===== LOG HEALTH ($TAG) ====="
ALL=$(ls "$LOGDIR/${TAG}_bringup.log" "$LOGDIR/${TAG}"_cyc*.log 2>/dev/null)
scan(){ printf '%-26s %s\n' "$1" "$(grep -hcE "$2" $ALL 2>/dev/null | paste -sd+ | bc 2>/dev/null || echo 0)"; }
scan "ERROR lines"            '\[ERROR\]'
scan "python tracebacks"      'Traceback \(most recent'
scan "planning failures"      'Planning FAILED|planner returned no trajectory'
scan "arrival misses"         'did not arrive'
scan "exec aborted/timeouts"  'TIMED_OUT|ABORTED|execution failed'
scan "implausible detections" 'rejected implausible'
scan "benign teardown segv"   'Segmentation fault'
echo "(ERRORs/tracebacks/planning failures should be 0; benign teardown segv is expected)"
echo "##### DONE ($TAG). Logs: $LOGDIR"
