#!/bin/bash
# Run the full 10-box arm task on a MIXED intact/damaged batch and score it with the independent checker.
#   tools/condition_arm_check.sh <seed> <damaged_csv> [model.onnx]
# With a model the challenger node runs beside the arm (log only) and its output is saved. Logs: logs/condition/arm_<seed>/
# ROUTING=1 (needs a model): condition routing is ON - reject bin in the world, damaged -> bin, intact -> tote, unknown -> skipped,
# judged by tools/check_condition_routing.py (true condition vs Gazebo pose). Without ROUTING the arm uses the depth detector only.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
SEED="${1:?seed}"; DAMAGED="${2:?damaged csv}"; MODEL="${3:-}"
DOMAIN="${DOMAIN:-86}"; SPEED="${SPEED:-0.045}"; ROUTING="${ROUTING:-}"
if [ -z "$ARM_CLEAN" ]; then
  exec env -i ARM_CLEAN=1 HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin DOMAIN="$DOMAIN" SPEED="$SPEED" ROUTING="$ROUTING" \
    DISPLAY="${DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" bash --noprofile --norc "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_VERSION=harmonic ROS_DOMAIN_ID="$DOMAIN" GZ_PARTITION="parcel_demo_$DOMAIN"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib | sort | paste -sd:)"
LOG="$ROOT/logs/condition/arm_$SEED"; rm -rf "$LOG"; mkdir -p "$LOG"
before=$(pgrep -f "gz sim" | sort | tr '\n' ' ' || true)
setsid ros2 launch twin_action_demo industrial_parcel_cell_bringup.launch.py continuous:=true parcel_count:=10 \
  grasp_sensors:=true headless:=true variant_seed:="$SEED" variant_damaged:="$DAMAGED" \
  variant_manifest:="$LOG/manifest.json" reject_bin:=$([ -n "$ROUTING" ] && echo true || echo false) \
  > "$LOG/sim.log" 2>&1 &
simpid=$!
nodepid=""
cleanup () {
  [ -n "$nodepid" ] && kill -INT "$nodepid" 2>/dev/null || true
  kill -INT -- -"$simpid" 2>/dev/null || true
  for _ in $(seq 1 25); do kill -0 "$simpid" 2>/dev/null || break; sleep 1; done
  kill -TERM -- -"$simpid" 2>/dev/null || true
  for g in $(pgrep -f "gz sim" || true); do case " $before " in *" $g "*) ;; *) kill "$g" 2>/dev/null || true ;; esac; done
  sleep 3
  for g in $(pgrep -f "gz sim" || true); do case " $before " in *" $g "*) ;; *) kill -9 "$g" 2>/dev/null || true ;; esac; done
}
trap cleanup EXIT
echo "[arm_check] waiting for controllers"
for _ in $(seq 1 150); do
  [ "$(ros2 control list_controllers 2>/dev/null | grep -o active | wc -l || true)" -ge 3 ] && break; sleep 1
done
if [ -n "$MODEL" ]; then
  COND_NODE_CLEAN= DOMAIN="$DOMAIN" "$ROOT/tools/run_condition_node.sh" "$MODEL" > "$LOG/condition_node.log" 2>&1 &
  nodepid=$!
  python3 "$ROOT/tools/condition_live_check.py" --manifest "$LOG/manifest.json" --out "$LOG/live_check.json" \
    > "$LOG/live_check.log" 2>&1 &
  livepid=$!
fi
sleep 3
export TWIN_TRAJECTORY_CHECK=sampled
echo "[arm_check] task start $(date +%T)"
rc=0
ros2 run twin_moveit_scripts parcel_task --continuous --grasp-mode confirmed --parcel-count 10 \
  --belt-speed "$SPEED" ${ROUTING:+--condition-routing} --output "$LOG/task_result.json" > "$LOG/task.log" 2>&1 || rc=$?
echo "[arm_check] task exit=$rc $(date +%T)"
if [ -n "${livepid:-}" ]; then kill -INT "$livepid" 2>/dev/null || true; wait "$livepid" 2>/dev/null || true; cat "$LOG/live_check.log"; fi
grep -E "PARCEL_TASK_COMPLETE|FULL|Traceback" "$LOG/task.log" | tail -5 || true
if [ -n "$ROUTING" ]; then
  python3 tools/check_condition_routing.py --manifest "$LOG/manifest.json" > "$LOG/checker.txt" 2>&1 || true
  python3 -c "import json;d=json.load(open('$LOG/checker.txt'));print({k:v for k,v in d.items() if k!='parcels'})" || tail -25 "$LOG/checker.txt"
else
  python3 tools/check_parcel_cell.py --placed --parcel-count 10 > "$LOG/checker.txt" 2>&1 || true
  tail -25 "$LOG/checker.txt"
fi
echo "tracebacks in task log: $(grep -c Traceback "$LOG/task.log" || true)"
