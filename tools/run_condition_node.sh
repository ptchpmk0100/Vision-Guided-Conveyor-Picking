#!/bin/bash
# Run the parcel-condition challenger node (log only; the arm never reads its topic).
#   tools/run_condition_node.sh <model.onnx> [min_confidence]
# Uses the training venv (onnxruntime) with this checkout's ROS overlay. Same DOMAIN as the sim (default 86).
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
MODEL="$(readlink -f "${1:?usage: run_condition_node.sh <model.onnx> [min_confidence]}")"
CONF="${2:-0.75}"
VENV="${CONDITION_VENV:-$HOME/venvs/parcel_cls}"
DOMAIN="${DOMAIN:-86}"
IMPL="${IMPL:-python}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-}"   # C++ node only; default /parcel_cell/condition
if [ -z "$COND_NODE_CLEAN" ]; then
  exec env -i COND_NODE_CLEAN=1 HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin DOMAIN="$DOMAIN" IMPL="$IMPL" OUTPUT_TOPIC="$OUTPUT_TOPIC" \
    bash --noprofile --norc "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="$DOMAIN" GZ_PARTITION="parcel_demo_$DOMAIN"
if [ "$IMPL" = cpp ]; then
  EXTRA=()
  [ -n "$OUTPUT_TOPIC" ] && EXTRA=(-p output_topic:="$OUTPUT_TOPIC")
  exec ros2 run twin_condition_cpp condition_node --ros-args \
    -p model:="$MODEL" -p min_confidence:="$CONF" -p use_sim_time:=true \
    -p extrinsic_file:="$ROOT/src/twin_action_demo/config/overhead_camera_extrinsic.yaml" "${EXTRA[@]}"
fi
exec "$VENV/bin/python" -m twin_moveit_scripts.parcel_condition_node --ros-args \
  -p model:="$MODEL" -p min_confidence:="$CONF" -p use_sim_time:=true \
  -p extrinsic_file:="$ROOT/src/twin_action_demo/config/overhead_camera_extrinsic.yaml"
