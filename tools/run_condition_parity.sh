#!/bin/bash
# B2: replay one recorded bag through the Python node and/or the C++ node and record their outputs.
#   tools/run_condition_parity.sh <bag_dir> <out_dir> [both|python|cpp]
# Needs: install/ built (twin_moveit_scripts, twin_condition_cpp), CONDITION_MODEL, ONNXRUNTIME_ROOT not needed at
# run time (RPATH). No Gazebo. Uses its own ROS domain (default 78). Logs: <out_dir>/python.log, cpp.log.
set -eo pipefail
set -m   # job control: every background job gets its own process group, so we can stop its whole tree
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG="$(readlink -f "${1:?usage: run_condition_parity.sh <bag_dir> <out_dir> [both|python|cpp]}")"
OUT="$(mkdir -p "${2:?out_dir}" && readlink -f "$2")"
MODE="${3:-both}"
MODEL="$(readlink -f "${CONDITION_MODEL:-$ROOT/logs/condition/runs/r1/model.onnx}")"
export DOMAIN="${DOMAIN:-78}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd "$ROOT"
rm -rf "$OUT/out" "$OUT"/*.log
pids=()
stop_group() { kill -TERM -- "-$1" 2>/dev/null || kill -TERM "$1" 2>/dev/null || true; }  # SIGINT is ignored by & jobs
cleanup() { for p in "${pids[@]}"; do stop_group "$p"; done; wait 2>/dev/null || true; }
trap cleanup EXIT

clean_ros() {  # run a command in a clean ROS shell with this checkout's overlay
  env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin ROS_DOMAIN_ID="$DOMAIN" \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST bash --noprofile --norc -c \
    "source /opt/ros/jazzy/setup.bash; source '$ROOT/install/setup.bash'; $*"
}
wait_ready() { for _ in $(seq 60); do grep -q "condition classifier ready" "$1" 2>/dev/null && return 0; sleep 0.5; done
  echo "node not ready: $1" >&2; tail -5 "$1" >&2; return 1; }

if [ "$MODE" != cpp ]; then
  COND_NODE_CLEAN= DOMAIN="$DOMAIN" tools/run_condition_node.sh "$MODEL" > "$OUT/python.log" 2>&1 & pids+=($!)
fi
if [ "$MODE" != python ]; then
  COND_NODE_CLEAN= DOMAIN="$DOMAIN" IMPL=cpp OUTPUT_TOPIC=/parcel_cell/condition_cpp \
    tools/run_condition_node.sh "$MODEL" > "$OUT/cpp.log" 2>&1 & pids+=($!)
fi
[ "$MODE" != cpp ] && wait_ready "$OUT/python.log"
[ "$MODE" != python ] && wait_ready "$OUT/cpp.log"

clean_ros "exec ros2 bag record -s mcap -o '$OUT/out' --use-sim-time /parcel_cell/condition /parcel_cell/condition_cpp" \
  > "$OUT/record.log" 2>&1 & rec=$!
sleep 3
clean_ros "exec ros2 bag play '$BAG' --clock" > "$OUT/play.log" 2>&1
sleep 4
stop_group "$rec"; wait "$rec" 2>/dev/null || true
cleanup; pids=()
echo "python: $(grep 'latency_ms' "$OUT/python.log" 2>/dev/null | tail -1 | sed 's/.*\] //')"
echo "cpp   : $(grep 'latency_ms' "$OUT/cpp.log" 2>/dev/null | tail -1 | sed 's/.*\] //')"
echo "output bag: $OUT/out"
