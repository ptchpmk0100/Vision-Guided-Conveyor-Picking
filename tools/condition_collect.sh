#!/bin/bash
# Record N labelled parcel-condition datasets, one fresh headless sim per recording.
#
#   tools/condition_collect.sh plan 20            # write logs/condition/plan.json (seeds, damaged sets, speeds, splits)
#   tools/condition_collect.sh run                # record every planned recording not yet done
#   tools/condition_collect.sh one r01            # record a single planned recording
#
# Output: logs/condition/rec/<name>/{bag,manifest.json,run_summary.json}. logs/ is git-ignored.
# Gazebo/MoveIt must NOT run while you train; this script owns the sim for each recording and stops it after.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
OUT="$ROOT/logs/condition"
PLAN="$OUT/plan.json"
DOMAIN="${DOMAIN:-86}"

if [ -z "$COND_CLEAN" ]; then
  exec env -i COND_CLEAN=1 HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin \
    DISPLAY="${DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" DOMAIN="$DOMAIN" \
    bash --noprofile --norc "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_VERSION=harmonic ROS_DOMAIN_ID="$DOMAIN" GZ_PARTITION="parcel_demo_$DOMAIN"
export PATH="$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin"
export GZ_CONFIG_PATH="$(ls -d /opt/ros/jazzy/opt/*_vendor/share/gz | sort | paste -sd:)"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$(ls -d /opt/ros/jazzy/opt/*_vendor/lib | sort | paste -sd:)"

make_plan () {
  python3 - "$1" "$PLAN" <<'EOF'
import json, random, sys
n = int(sys.argv[1]); rng = random.Random(20261111)
names = [f'r{i:02d}' for i in range(1, n + 1)]
order = names[:]; rng.shuffle(order)
n_test, n_val = max(2, round(n * .2)), max(2, round(n * .15))
split = {nm: 'test' for nm in order[:n_test]} | {nm: 'val' for nm in order[n_test:n_test + n_val]}
recs = []
for nm in names:
    k = rng.randint(3, 7)
    recs.append({'name': nm, 'seed': f'cond-{nm}', 'damaged': ','.join(map(str, sorted(rng.sample(range(1, 11), k)))),
                 'speed': rng.choice([0.07, 0.08]), 'split': split.get(nm, 'train')})
json.dump({'recordings': recs, 'splits': {r['name']: r['split'] for r in recs}}, open(sys.argv[2], 'w'), indent=1)
print(f"planned {n}: " + ', '.join(f"{s}={sum(r['split']==s for r in recs)}" for s in ('train', 'val', 'test')))
EOF
}

field () { python3 -c "import json,sys;print(next(r for r in json.load(open('$PLAN'))['recordings'] if r['name']=='$1')['$2'])"; }

record_one () {
  local name="$1" dir="$OUT/rec/$1"
  if [ -f "$dir/run_summary.json" ] && grep -q '"ok": true' "$dir/run_summary.json"; then echo "[$name] done already"; return; fi
  rm -rf "$dir"; mkdir -p "$dir"
  local seed damaged speed
  seed=$(field "$name" seed); damaged=$(field "$name" damaged); speed=$(field "$name" speed)
  echo "[$name] seed=$seed damaged=$damaged speed=$speed"
  local before; before=$(pgrep -f "gz sim" | sort | tr '\n' ' ' || true)
  setsid ros2 launch twin_action_demo industrial_parcel_cell_bringup.launch.py continuous:=true parcel_count:=10 \
      headless:=true variant_seed:="$seed" variant_damaged:="$damaged" variant_manifest:="$dir/manifest.json" \
      variant_yaw_deg:=10 > "$dir/sim.log" 2>&1 &
  local pid=$!
  local rc=0
  python3 tools/condition_run.py --out "$dir" --speed "$speed" || rc=$?
  kill -INT -- -"$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  kill -TERM -- -"$pid" 2>/dev/null || true
  # the gz sim child can outlive the launch: stop only sims that did not exist before this recording
  for g in $(pgrep -f "gz sim" || true); do
    case " $before " in *" $g "*) ;; *) kill "$g" 2>/dev/null || true ;; esac
  done
  sleep 3
  for g in $(pgrep -f "gz sim" || true); do
    case " $before " in *" $g "*) ;; *) kill -9 "$g" 2>/dev/null || true ;; esac
  done
  echo "[$name] exit=$rc"
  return 0
}

case "${1:-}" in
  plan) mkdir -p "$OUT"; make_plan "${2:-20}" ;;
  one)  record_one "$2" ;;
  run)  for name in $(python3 -c "import json;print(' '.join(r['name'] for r in json.load(open('$PLAN'))['recordings']))"); do
          record_one "$name"
        done ;;
  *) sed -n 2,9p "$0"; exit 2 ;;
esac
