# Factory Sorting Cell — ROS 2 / Gazebo digital twin

A simulated industrial **pick-and-sort cell**: a Franka Panda arm detects coloured boxes
staged on a belt-height deck, identifies each by colour, and sorts it into the matching
colour-labelled tray — collision-aware, orchestrated by a behaviour tree, monitored by a
runtime validator, and measured against Gazebo ground truth.

**Stack:** ROS 2 Jazzy · Gazebo Harmonic (`gz sim`) · MoveIt 2 (`moveit_py`) · OpenCV (HSV +
connected components) · a dependency-free behaviour tree · SQLite benchmarking.

> Every result below is judged by **Gazebo ground truth** (actual entity poses), never by a
> node's own log lines — a hard rule of this project after status lines were caught lying
> (e.g. `place=True` on an object that never moved). See `V2_DECISIONS.md` for the full
> decision trail and `V2_RESUME.md` for the engineering ledger.

## What it does (pipeline)

```
overhead RGBD camera
  └─ object_detector_sorting_line   HSV saturation gate → connected components
     │                               → per-blob median depth → pinhole deprojection
     │                               → CALIBRATED extrinsic (config/overhead_camera_extrinsic.yaml)
     ▼
  /detected_objects/array  (vision_msgs/Detection3DArray: colour, pose)
     ├─ sort_tree_sorting_line       behaviour tree: Sequence[Detect → Retry(Pick,2) → Place → Home]
     │   └─ move_to (MoveIt 2)        collision-aware top-down grasp / place into trays
     └─ sorting_validator            runtime monitor → /validation/report (workspace, class, wrong-tray)
     ▼
  SQLite benchmark (trial_db) + ground-truth checkers (tools/check_*.py)
```

## Results (all by Gazebo ground truth)

| Capability | Result |
|---|---|
| Reach at belt height (go/no-go) | **42/42** probe legs |
| Scene geometry (boxes settle @ z=0.320) | **3/3** × 2 runs |
| Perception (class-correct, 0 phantoms) | **3/3**, XY err ≤ 4.5 mm × 2 |
| Hand-eye calibration (recovers true extrinsic) | **RMS 0.37 mm**, 0.06° / 0.74 mm; repeatable |
| Full colour sort (into correct tray) | **3/3** × 2, pick **6/6 first-attempt**, 0 errors |
| Behaviour-tree sort | **3/3** × 2 with reactive recovery |
| Runtime validator | flags all injected violations; OK on valid scene |
| Legacy ground-cell pipeline (regression) | **3/3** (unchanged) |

## Quick start

```bash
# 1. build (clean env recommended; the repo is a ROS 2 workspace)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select twin_action_demo twin_moveit_scripts
source install/setup.bash
export GZ_VERSION=harmonic

# 2. bring up the sorting cell (GUI). Add headless:=true for full-speed no-GUI.
#    detector:=true starts perception; extrinsic_file:=... uses the calibrated transform.
ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py detector:=true

# 3. run the sort (behaviour-tree orchestrator) in a second terminal
ros2 run twin_moveit_scripts sort_tree_sorting_line
#   (or the imperative equivalent: sort_task_sorting_line)

# 4. independent ground-truth verdict (third terminal)
python3 tools/check_sorting_line.py --placed     # every box in its correct tray?
```

### Validate without a GUI (headless, scored)
```bash
tools/run_sort_line_check.sh 2            # 2 consecutive sorts, ground truth + log health
tools/run_offline_tests.sh                # component self-tests, no simulator (CI backbone)
```

### Recalibrate the camera (hand-eye)
```bash
tools/run_b1_calibration.sh               # 2 captures of 12 flat targets → gated Kabsch fit
#   writes logs/v2/calibration/overhead_camera_extrinsic.candidate.yaml; promote to
#   src/twin_action_demo/config/overhead_camera_extrinsic.yaml to put it in service.
```

## Industrial parcel-cell variant

The additive parcel scene keeps the same Panda and overhead RGBD calibration. It adds a
long conveyor, cardboard parcels, an open blue tote, industrial pedestal, cabinet, fencing,
and saved presentation cameras. The original colour-sorting launch remains available.

After sourcing the ROS base and this checkout's overlay in each terminal:

```bash
ros2 launch twin_action_demo industrial_parcel_cell_bringup.launch.py continuous:=true
ros2 run twin_moveit_scripts parcel_task --continuous
```

Omit both continuous flags for the original stationary batch. For a queued stop-and-pick
batch, launch with `indexed:=true` and run `parcel_task --indexed`. The belt
moves cartons by Gazebo surface-contact physics, stops for observation/picking, and stops
on lost command messages. Three cartons fill three tote slots; start with an empty tote
and reset/relaunch for the next batch. The right feeder is a stationary supported section.
Use distinct `ROS_DOMAIN_ID` and `GZ_PARTITION` values for concurrent simulations.

For moving-target handling, pair `continuous:=true` in the launch with
`parcel_task --continuous` in the other terminal. This selects an extended upstream
belt with 1.2 m parcel spacing (20 simulated seconds between arrivals). The belt runs
at 0.06 m/s throughout normal picking,
placement and verification. The robot selects the closest reachable camera track,
matches conveyor motion above it, descends while following, closes and lifts it,
then places it in the next tote slot. The next parcel keeps travelling during placement.
Perception/command loss or a failed intercept stops the feed. This demo finishes when
the three-slot tote is full; relaunch with an empty tote for another batch.
The original stationary and indexed modes remain available.

An opt-in **7, 8 or 10-parcel** batch uses the SAME conveyor as the three-parcel
profile (a 5.0 m deck, y = -3.6 to +1.4 m; it is never lengthened) and a wider,
single-layer tote. The extra boxes are fitted by shrinking the pitch so the last box
stays 0.2 m inside the upstream end: **10 boxes at 0.344 m, 8 at 0.442 m, 7 at 0.516 m**
(the three-box profile keeps 1.2 m). The smallest allowed pitch is 0.14 m, which leaves
at least 35 mm between an open finger and the next box. With a fixed belt length the
belt speed must match the arm: the arrival interval is pitch / belt speed and must not
be shorter than the pick-place-return cycle. Ten boxes run at **0.045 m/s** (7.6
simulated seconds between arrivals). The three-box profile still runs at 0.06 m/s.

These profiles also use a faster cycle than the three-box path (the three-box path
is unchanged): the gripper opens in 0.5 s instead of 2.5 s, there is no separate retract
move after release, the tote slot is confirmed from depth while the arm is already
heading back, one camera tracker stays live for the whole run, and the arm launches as
soon as a whole, tracked box is reachable (predicted y >= -0.25 m instead of -0.16 m)
with a shorter, still collision-checked interception (0.3 s planning lead, grasp at
1.35 s, 0.9 s lift). If the arm is not yet able to follow an early box, nothing is
sent and it retries as the box comes closer. After placing, the arm waits for the next
whole box; it does no extra moves.

With the ROS base and this mirror's overlay sourced in both terminals, run:

```bash
# Terminal 1: simulator, camera detector and finger contact sensors
ros2 launch twin_action_demo industrial_parcel_cell_bringup.launch.py \
  continuous:=true parcel_count:=10

# Terminal 2: camera-guided interception with confirmed two-pad grasps
ros2 run twin_moveit_scripts parcel_task --continuous --grasp-mode confirmed \
  --parcel-count 10 --belt-speed 0.045
```

The launch and task profiles must match (the pitch is derived from the count; do not
pass one that does not fit). The profile also widens the temporary RGBD sensor field of
view; it retains the calibrated camera transform and reads the actual camera
intrinsics. A fault stops the feed. Relaunch with an empty tote for another batch.
See [TEN_PARCEL_PLAN.md](TEN_PARCEL_PLAN.md) for the capacity maths, risks and evidence.

To simply WATCH the arm in the Gazebo window, use the helper. It starts from a clean
environment, uses the validated settings and waits for the controllers, so each terminal
is one command (start the second right after the first):

```bash
tools/parcel_demo.sh sim     # terminal 1: simulator, detector, sensors (add "headless" for no window)
tools/parcel_demo.sh task    # terminal 2: the camera-guided arm task
```

Set `COUNT=3|7|8|10`, `SPEED=<m/s>` or `DOMAIN=<n>` (the same in both terminals) to change
the batch. Stop with Ctrl+C in terminal 1. It has none of the scored harness's independent
checkers; use `tools/run_parcel_cell_check.sh` (below) for pass/fail evidence.

### Parcel condition: damaged vs intact (RGB classifier, opt-in)

A small ResNet18 classifies the RGB crop at each depth-detected parcel as `intact` or `damaged`. It is OFF by default:
the depth detector alone drives the arm. Results, commands and caveats (simulation only, large synthetic marks):
[PARCEL_CONDITION_CLASSIFIER.md](PARCEL_CONDITION_CLASSIFIER.md). To watch it route parcels (7/8/10 boxes, needs the
trained model from `tools/condition_train.py`, which lives outside the ROS python in `~/venvs/parcel_cls`):

```bash
COUNT=10 CONDITION_MODEL=logs/condition/runs/r1/model.onnx VARIANT_DAMAGED=3,6,9 tools/parcel_demo.sh sim   # terminal 1
COUNT=10 CONDITION_MODEL=logs/condition/runs/r1/model.onnx VARIANT_DAMAGED=3,6,9 tools/parcel_demo.sh task  # terminal 2
```

Damaged parcels go to the red reject bin, intact ones to the tote, and a parcel whose class stays unknown is left on the
belt and logged (never guessed). Afterwards `python3 tools/check_condition_routing.py --manifest logs/parcel_demo_manifest.json`
judges the result against Gazebo. Scored version with the same checker:
`ROUTING=1 tools/condition_arm_check.sh <seed> <damaged csv> <model.onnx>` (checker `tools/check_condition_routing.py`).
Unset `CONDITION_MODEL` (or omit `--condition-routing` / `reject_bin:=true`) for the original depth-only behaviour.

For a scored ten-box GUI test from this repository root in a desktop terminal:

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin \
  DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash --noprofile --norc tools/run_parcel_cell_check.sh \
  --gui --mode continuous --grasp-mode confirmed --parcel-count 10 --belt-speed 0.045 \
  --motion-metrics --trajectory-check sampled --runs 1 --domain 85 \
  --logdir "logs/orig_belt/manual_$(date +%Y%m%d_%H%M%S)"
```

This performs independent geometry, perception, transport, grasp and placement
checks and then closes its own simulation. Omit `--gui` and the display variables
for headless operation. GUI playback can take substantially longer than simulated
time on this machine; a 7.6-second simulated arrival interval is not 7.6 wall seconds.
A heavily loaded host (for example a busy editor language server) lowers the
real-time factor and can make the unchanged three-box path time out on a gripper
result; see TEN_PARCEL_PLAN.md.

Run independent scored checks from a clean shell, choosing a fresh evidence directory:

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin bash --noprofile --norc \
  tools/run_parcel_cell_check.sh --mode stationary --runs 2 \
  --logdir /tmp/panda-parcel-stationary-check
# Use --mode continuous for moving grasps, indexed for stop-and-pick transport,
# or geometry/transport/faults for focused checks.
```

Actual verification results, changed components, and limitations are recorded in
[PARCEL_CELL_IMPLEMENTATION.md](PARCEL_CELL_IMPLEMENTATION.md). Design rationale and
acceptance gates are in [PANDA_ENVIRONMENT_REDESIGN_PLAN.md](PANDA_ENVIRONMENT_REDESIGN_PLAN.md).

If the arm pauses at `ready`, check the task's last state. `FEED` means it is waiting for
the conveyor to deliver a carton; the first delivery takes about 20–30 wall seconds in
the GUI on this machine. `FEED_WAIT` now reports progress every five seconds.
If the reported simulation time stays fixed, check Gazebo's Play state. Keep one
`parcel_task` running per ROS domain; a second invocation is refused before it can move
the arm. Match `indexed:=true` in the launch with `--indexed` in the task, or omit both
for stationary cartons. `PARCEL_TASK_FAILED` is also saved in the ROS log.
Continuous mode uses `CONTINUOUS_WAIT` and `INTERCEPT` instead of `FEED`/`STOPPED`.
At startup it moves directly above the belt while opening the gripper in parallel.
The feed starts once both motions and fresh empty-tote/perception checks are complete.

To test the full GUI workflow automatically from a desktop terminal:

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin \
  DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash --noprofile --norc tools/run_parcel_cell_check.sh \
  --gui --quick-start --mode continuous --runs 1 --domain 88 \
  --logdir "logs/gui_check_$(date +%Y%m%d_%H%M%S)"
```

This opens a fresh cell, runs the task, checks actual placement and closes that test cell.
`--quick-start` begins preparation when controllers are active and runs independent
checks after the batch. Omit it to include the full pre-run geometry/perception inspection.
See [GUI_PARCEL_DIAGNOSIS.md](GUI_PARCEL_DIAGNOSIS.md) for the GUI investigation and evidence.
The continuous design and its verification are recorded in
[CONTINUOUS_PARCEL_PLAN.md](CONTINUOUS_PARCEL_PLAN.md).

Optional path-planning experiments, their validation gates and measured comparisons
are documented in [PATH_PLANNING_PLAN.md](PATH_PLANNING_PLAN.md). The normal planner
remains RRTConnect with TOTG. `--motion-metrics --trajectory-check sampled` on the
parcel harness records final planned paths and checks them before dispatch. Experimental
smoothing and arrival shadowing are separate opt-ins; the shadow does not control arrival.

## Layout

| Path | What |
|---|---|
| `src/twin_action_demo/worlds/industrial_sorting_line.sdf` | the belt-height cell (pick deck, tray table, 3 colour-floored trays) |
| `src/twin_action_demo/launch/industrial_sorting_line_bringup.launch.py` | robot + camera + boxes + (opt) detector |
| `src/twin_moveit_scripts/twin_moveit_scripts/move_to.py` | MoveIt grasp/place primitives (relative-height, grasp-width gate) |
| `…/object_detector_sorting_line.py` | HSV+depth detector; loads the calibrated extrinsic |
| `…/sorting_line_scene.py` | trays as MoveIt collision objects |
| `…/sort_task_sorting_line.py` / `…/sort_tree_sorting_line.py` | imperative / behaviour-tree sorter |
| `…/behaviour_tree.py` | dependency-free BT core (Sequence/Selector/Retry) |
| `…/sorting_validator.py` | runtime invariant monitor |
| `…/trial_db.py` + `tools/benchmark.py` | SQLite trial logging + metrics |
| `tools/*calibration*` | hand-eye extrinsic calibration (world, collector, Kabsch solver) |
| `tools/check_*.py`, `tools/run_*.sh` | ground-truth checkers + headless run harnesses |

## Design notes (hard-won)
- **Geometry is relative.** `pick`/`place` compute every height from the object/target z, so
  the ground cell and the belt-height line share one code path; belt height needed only a
  grasp-width retarget (Stage 0 proved reach first).
- **Perception uses true depth deprojection**, so raising objects moved only the z-gate — XY
  stayed accurate. The camera extrinsic is now *calibrated* (data-driven), not hardcoded.
- **No fill lighting, no blue in view**: the HSV gate is robust, but these invariants protect
  the detector (the arm's indicator dots are blue; extra light once dragged a centroid 29 mm).
- **Flat calibration targets**: tall boxes bias the blob centroid off-centre (parallax); flat
  tiles removed it (fit RMS 3.0 → 0.37 mm).

## Reproducibility
`Dockerfile` pins a ROS 2 Jazzy + Gazebo Harmonic environment and builds the workspace;
`.github/workflows/ci.yml` runs `tools/run_offline_tests.sh` on push. Building the image and
running CI require Docker / a GitHub runner respectively (not exercised in this sim-only repo).
