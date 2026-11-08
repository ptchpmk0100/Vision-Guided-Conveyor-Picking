#!/bin/bash
# A2c/A2d validation of the multi-object detector -- all THREE legs in one run:
#   (a) ground truth : check_detections.py scores detections against Gazebo poses
#   (b) log health   : scan bringup/detector logs for errors, tracebacks, exceptions
#   (c) consecutive  : repeat the scoring N times; a single pass cannot prove stability
# No `set -u` (ROS setup.bash references unset vars).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
REPEATS="${1:-3}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

cleanup(){ echo ">>> cleanup"; [ -n "${LPGID:-}" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "${LPGID:-}" ] && kill -KILL -"$LPGID" 2>/dev/null; }
trap cleanup EXIT

LAUNCH="$MIRROR/install/twin_action_demo/share/twin_action_demo/launch/industrial_cell_a2_bringup.launch.py"
echo ">>> launching A2 cell headless (3 mixed objects + multi detector)"
setsid ros2 launch "$LAUNCH" headless:=true > "$LOGDIR/a2_bringup.log" 2>&1 &
LPGID=$!

echo ">>> waiting for controllers (<=120s)"
ok=0
for i in $(seq 1 60); do
  n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
  if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
  sleep 2
done
[ "$ok" = 1 ] || { echo "!! controllers never came active"; exit 2; }

echo ">>> waiting for the detector array (<=60s)"
det=0
for i in $(seq 1 12); do
  if timeout 6 ros2 topic echo /detected_objects/array --once 2>/dev/null | grep -q 'detections'; then
    det=1; echo "array publishing after ~$((i*6))s"; break; fi
done
[ "$det" = 1 ] || { echo "!! nothing on /detected_objects/array"; exit 3; }
sleep 3

# ---- legs (a) + (c): score against ground truth, REPEATS times -------------
PASS=0
for r in $(seq 1 "$REPEATS"); do
  echo "----- detection score, pass $r/$REPEATS -----"
  if python3 "$TOOLS/check_detections.py" 2>&1 | tee "$LOGDIR/a2_score_$r.log" | grep -E 'OK |MISS|CLASS|POS|PHANTOM|-----'; then
    PASS=$((PASS+1))
  fi
  sleep 2
done
echo "===== CONSECUTIVE RESULT: $PASS/$REPEATS passes clean ====="

# ---- leg (b): log health ---------------------------------------------------
echo "===== LOG HEALTH ====="
ALL="$LOGDIR/a2_bringup.log"
for pair in "ERROR lines:\[ERROR\]" "tracebacks:Traceback \(most recent" \
            "exceptions:Exception|raised" "cv2 errors:cv2\.error" \
            "detector warnings:\[WARN\].*detector"; do
  lbl="${pair%%:*}"; pat="${pair#*:}"
  printf '%-22s %s\n' "$lbl" "$(grep -cE "$pat" $ALL 2>/dev/null)"
done
echo "--- last detector report ---"
grep -oE 'detections\([0-9]+\):.*' "$ALL" | tail -2
echo "##### A2 DETECTOR CHECK DONE. Logs: $LOGDIR"
