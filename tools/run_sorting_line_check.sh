#!/bin/bash
# Sorting-line stage runner (manager-operated). Brings up the raised cell headless and
# scores by ground truth. Stage A = geometry only (detector off).
#   usage: run_sorting_line_check.sh [runs] [detector:true|false]
# No `set -u` (ROS setup.bash).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
RUNS="${1:-2}"
DETECTOR="${2:-false}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

for run in $(seq 1 "$RUNS"); do
  echo "############ SORTING-LINE RUN $run/$RUNS (detector=$DETECTOR) ############"
  LPGID=""
  cleanup(){ [ -n "$LPGID" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "$LPGID" ] && kill -KILL -"$LPGID" 2>/dev/null; }
  trap cleanup EXIT
  setsid ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py \
    headless:=true detector:="$DETECTOR" ${TWIN_EXTRINSIC:+extrinsic_file:="$TWIN_EXTRINSIC"} \
    > "$LOGDIR/sline_bringup_$run.log" 2>&1 &
  LPGID=$!

  ok=0
  for i in $(seq 1 60); do
    n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
    if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
    sleep 2
  done
  [ "$ok" = 1 ] || { echo "!! controllers never came active"; cleanup; exit 2; }
  sleep 4   # let the boxes drop and settle on the deck

  echo "--- ground truth (geometry) ---"
  python3 "$TOOLS/check_sorting_line.py"

  if [ "$DETECTOR" = "true" ]; then
    echo "--- waiting for detector to publish ---"
    for i in $(seq 1 15); do
      timeout 6 ros2 topic echo /detected_objects/array --once 2>/dev/null | grep -q 'detections' && { echo "detector publishing"; break; }
    done
    sleep 2
    echo "--- detector ground truth (Stage B) ---"
    python3 "$TOOLS/check_detections_sorting_line.py"
  fi

  cleanup; trap - EXIT; sleep 3
done
echo "##### SORTING-LINE CHECK DONE. Logs: $LOGDIR"
