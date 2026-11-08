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
