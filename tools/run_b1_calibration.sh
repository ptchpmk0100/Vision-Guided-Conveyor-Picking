#!/bin/bash
# B1 calibration runner (manager-operated): for each of 2 fresh headless launches, collect a
# correspondence dataset; then fit the extrinsic (Kabsch, gated) from both. The go/no-go is
# whether the fit recovers the known nominal extrinsic within the solver's gates.
# No `set -u` (ROS setup.bash).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
CALDIR="$LOGDIR/calibration"
mkdir -p "$CALDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

DS=()
for run in 1 2; do
  echo "############ CALIBRATION CAPTURE $run/2 ############"
  LPGID=""
  cleanup(){ [ -n "$LPGID" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "$LPGID" ] && kill -KILL -"$LPGID" 2>/dev/null; }
  trap cleanup EXIT
  setsid ros2 launch twin_action_demo sorting_line_extrinsic_calibration.launch.py \
    headless:=true > "$LOGDIR/b1cal_bringup_$run.log" 2>&1 &
  LPGID=$!
  ok=0
  for i in $(seq 1 60); do
    n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
    if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
    sleep 2
  done
  [ "$ok" = 1 ] || { echo "!! controllers never came active"; cleanup; exit 2; }
  sleep 6    # let all 12 boxes settle on the platform/pedestals

  OUT="$CALDIR/dataset_run${run}.json"
  echo "--- collecting correspondences -> $OUT ---"
  python3 "$TOOLS/collect_extrinsic_calibration.py" --run-id "run${run}" --output "$OUT" \
    --target-half-height 0.002 2>&1 | tail -20
  [ -f "$OUT" ] && DS+=("$OUT")
  cleanup; trap - EXIT; sleep 3
done

echo
echo "############ FITTING EXTRINSIC (both datasets) ############"
if [ "${#DS[@]}" -lt 2 ]; then
  echo "!! only ${#DS[@]} dataset(s) collected; need 2 for repeatability. Aborting fit."
  exit 3
fi
python3 "$TOOLS/fit_extrinsic_calibration.py" "${DS[@]}" 2>&1
echo "fit rc=$?"
echo "##### B1 CALIBRATION DONE. datasets+candidate in $CALDIR #####"
