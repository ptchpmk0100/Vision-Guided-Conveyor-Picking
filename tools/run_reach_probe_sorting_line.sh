#!/bin/bash
# Stage 0 runner (manager-operated): bring up the headless verification cell, CONFIRM the
# EE link is actually published on the pose topic, then run the raised reach probe.
# No `set -u` (ROS setup.bash).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
PASSES="${1:-2}"
LOGDIR="${LOGDIR:-$MIRROR/logs/v2}"
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
source "$MIRROR/install/setup.bash"
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz 2>/dev/null | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib 2>/dev/null | sort | paste -sd:)"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"

LAUNCH="$TOOLS/verify_industrial_headless.launch.py"
LPGID=""
cleanup(){ [ -n "$LPGID" ] && kill -TERM -"$LPGID" 2>/dev/null; sleep 3; [ -n "$LPGID" ] && kill -KILL -"$LPGID" 2>/dev/null; }
trap cleanup EXIT

echo "### bringing up $LAUNCH (headless) ###"
setsid ros2 launch "$LAUNCH" > "$LOGDIR/reachline_bringup.log" 2>&1 &
LPGID=$!

ok=0
for i in $(seq 1 60); do
  n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
  if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
  sleep 2
done
[ "$ok" = 1 ] || { echo "!! controllers never came active"; exit 2; }
sleep 3

echo "### PRECHECK: are the probe's GT frames (fingers + link7) published? ###"
SAMPLE=$(timeout 15 gz topic -e -n 1 -t /world/empty/pose/info 2>/dev/null)
HIT=$(echo "$SAMPLE" | grep -cE 'panda_leftfinger|panda_rightfinger|panda_link7')
echo "finger/link7 name hits on pose/info: ${HIT:-0} (expect >=3)"
if [ "${HIT:-0}" -lt 3 ]; then
  echo "!! probe GT frames missing on pose/info -- ground-truth read would be invalid."
  echo "   panda frames present:"
  echo "$SAMPLE" | grep -oE 'name: "[^"]+"' | sort -u | grep -iE 'panda|finger|link'
  echo "ABORTING probe run (feed this back to Codex)."
  exit 3
fi

echo "### running reach probe --passes $PASSES ###"
timeout 900 python3 "$TOOLS/reach_probe_sorting_line.py" --passes "$PASSES" \
  2>&1 | tee "$LOGDIR/reachline_probe.log"
echo "probe rc=${PIPESTATUS[0]}"
echo "### done ###"
