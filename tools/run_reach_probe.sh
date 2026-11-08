#!/bin/bash
# A2a: bring the cell up headless, then run the reach probe. No `set -u` (ROS setup.bash).
TOOLS="$(cd "$(dirname "$0")" && pwd)"
MIRROR="$(dirname "$TOOLS")"
LAUNCH="${1:-$TOOLS/verify_industrial_headless.launch.py}"
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

echo ">>> launching headless for reach probe: $LAUNCH"
setsid ros2 launch "$LAUNCH" > "$LOGDIR/reach_bringup.log" 2>&1 &
LPGID=$!

echo ">>> waiting for controllers active (<=120s)"
ok=0
for i in $(seq 1 60); do
  n=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c 'active')
  if [ "${n:-0}" -ge 3 ]; then ok=1; echo "controllers active after ~$((i*2))s"; break; fi
  sleep 2
done
[ "$ok" = 1 ] || { echo "!! controllers never came active"; exit 2; }
sleep 3

echo ">>> running reach probe"
timeout 600 python3 "$TOOLS/reach_probe.py" 2>&1 | tee "$LOGDIR/reach_probe.log" | grep -E 'REACH|SUMMARY|=====|reachable'
echo "##### REACH PROBE DONE. Full log: $LOGDIR/reach_probe.log"
