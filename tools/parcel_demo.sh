#!/bin/bash
# Run the parcel cell by hand in two terminals so you can WATCH the arm.
#
#   Terminal 1 (simulator + camera detector + contact sensors):
#       tools/parcel_demo.sh sim              # Gazebo window
#       tools/parcel_demo.sh sim headless     # no window
#   Terminal 2 (camera-guided arm task; waits for the controllers, then starts):
#       tools/parcel_demo.sh task
#
# Optional environment (set it the same in BOTH terminals):
#   COUNT=3|7|8|10   boxes in the batch (default 10)
#   SPEED=<m/s>      belt speed (default 0.045 for 7/8/10, 0.06 for 3)
#   DOMAIN=<n>       ROS domain / Gazebo partition (default 86)
#   CONDITION_MODEL=<model.onnx>   OPT-IN condition routing (COUNT=7|8|10; set it the same in BOTH terminals):
#                    some parcels spawn damaged (VARIANT_SEED, VARIANT_DAMAGED e.g. 3,6,9), a reject bin is added,
#                    the task starts the RGB classifier node and sends damaged parcels to the bin, intact ones to the
#                    tote and unknown ones nowhere (skipped). Unset = depth detector only, exactly as before.
#
# Stop with Ctrl+C in terminal 1. Relaunch the simulator for another batch (the tote
# must be empty). This is the same configuration the scored harness
# (tools/run_parcel_cell_check.sh) validates, but WITHOUT its independent checkers.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

COUNT="${COUNT:-10}"
DOMAIN="${DOMAIN:-86}"
if [ "$COUNT" = 3 ]; then SPEED="${SPEED:-0.06}"; else SPEED="${SPEED:-0.045}"; fi

# Re-run in a clean environment: never inherit another workspace from ~/.bashrc.
if [ -z "$PARCEL_DEMO_CLEAN" ]; then
  exec env -i PARCEL_DEMO_CLEAN=1 HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    COUNT="$COUNT" DOMAIN="$DOMAIN" SPEED="$SPEED" CONDITION_MODEL="${CONDITION_MODEL:-}" \
    VARIANT_SEED="${VARIANT_SEED:-}" VARIANT_DAMAGED="${VARIANT_DAMAGED:-}" bash --noprofile --norc "$0" "$@"
fi

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_VERSION=harmonic
export ROS_DOMAIN_ID="$DOMAIN"
export GZ_PARTITION="parcel_demo_$DOMAIN"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib | sort | paste -sd:)"

case "${1:-}" in
  sim)
    headless=false
    [ "${2:-}" = headless ] && headless=true
    echo "[parcel_demo] simulator: $COUNT boxes, ROS domain $DOMAIN, headless=$headless"
    extra=()
    if [ -n "$CONDITION_MODEL" ]; then
      extra=(reject_bin:=true variant_seed:="${VARIANT_SEED:-demo}" variant_damaged:="${VARIANT_DAMAGED:-3,6,9}"
             variant_manifest:="$ROOT/logs/parcel_demo_manifest.json")
      echo "[parcel_demo] condition routing ON: damaged parcels ${VARIANT_DAMAGED:-3,6,9}, reject bin added (truth: logs/parcel_demo_manifest.json)"
    fi
    exec ros2 launch twin_action_demo industrial_parcel_cell_bringup.launch.py \
      continuous:=true parcel_count:="$COUNT" grasp_sensors:=true headless:="$headless" "${extra[@]}"
    ;;
  task)
    echo "[parcel_demo] waiting for the arm and gripper controllers (domain $DOMAIN)..."
    ready=0
    for _ in $(seq 1 120); do
      active=$(ros2 control list_controllers 2>/dev/null | grep -o active | wc -l || true)
      if [ "${active:-0}" -ge 3 ]; then ready=1; break; fi
      sleep 1
    done
    if [ "$ready" != 1 ]; then
      echo "[parcel_demo] controllers never became active - is 'sim' running with the same DOMAIN?" >&2
      exit 1
    fi
    sleep 3
    echo "[parcel_demo] starting the arm task: $COUNT boxes, belt $SPEED m/s"
    export TWIN_TRAJECTORY_CHECK=sampled
    if [ -n "$CONDITION_MODEL" ]; then
      echo "[parcel_demo] starting the RGB condition classifier ($CONDITION_MODEL)"
      DOMAIN="$DOMAIN" "$ROOT/tools/run_condition_node.sh" "$CONDITION_MODEL" > "$ROOT/logs/parcel_demo_condition_node.log" 2>&1 &
      node=$!
      trap 'kill -INT $node 2>/dev/null' EXIT
      sleep 4
      ros2 run twin_moveit_scripts parcel_task --continuous --grasp-mode confirmed \
        --parcel-count "$COUNT" --belt-speed "$SPEED" --condition-routing
      exit $?
    fi
    exec ros2 run twin_moveit_scripts parcel_task --continuous --grasp-mode confirmed \
      --parcel-count "$COUNT" --belt-speed "$SPEED"
    ;;
  *)
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
