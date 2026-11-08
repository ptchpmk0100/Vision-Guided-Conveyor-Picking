# V2 sorting cell — exhaustive session log

**Record date:** 2026-11-07

**Scope:** autonomous run through the written Phase-C capstone (`C3`), entirely in
`~/ros_test_ws_mirror`

**Status:** historical synthesis; every result remains a claim until checked against the
named source records and raw run outputs

**Primary sources:** `V2_DECISIONS.md` D0–D18, `V2_RESUME.md`, `V2_PROGRESS.md`, `README.md`,
the repository's git history, and `logs/v2/*.out`

This is the long-form “for later check” record. It intentionally includes failed approaches,
misleading intermediate results, and teardown anomalies. A number is included only when one
of the sources above states it. When the repository does not preserve an intermediate number,
this log points to the appropriate source instead of reconstructing one.

---

## 1. Overview and operating context

The project is a ROS 2 Jazzy / Gazebo Harmonic digital twin of an industrial pick-and-sort
cell. A Franka Panda observes payloads with a fixed overhead RGBD camera, detects them, picks
them with MoveIt 2, and routes them by colour. The verified V2 end state is a belt-height
sorting line: three identical `40×40×50 mm` red, green, and yellow boxes rest on a static
pick deck whose top is `z=0.295`; the boxes' centres are `z=0.320`; and the robot places them
in colour-labelled trays at the same working height. The production pipeline is measured by
Gazebo entity poses, not by its own status messages.

The run began from a Phase-A/S22 ground-cell system and was driven autonomously toward C3.
The non-negotiable boundary was that `~/ros_test_ws` remained read-only: all code, builds,
logs, commits, and the local `v2.0` tag live in `~/ros_test_ws_mirror`. Work was additive so
the older ground-cell path stayed runnable for A/B comparison and regression testing.

The governing engineering rules, preserved in `V2_RESUME.md`, were:

- Ground truth decides functional success. Logs are health evidence only. The project had
  already seen “HELD” on an empty hand and `place=True` when the object never moved.
- Every meaningful stage uses three legs where possible: Gazebo truth, log-health scanning,
  and at least two consecutive runs.
- Build from a clean shell because `~/.bashrc` sources the live overlay and can silently
  shadow mirror packages.
- Never add camera-visible fill lighting or blue geometry; never use broad `pkill -f` patterns;
  and run Gazebo headless to avoid GUI throttling and orphan processes.
- Local, reversible changes could proceed autonomously. Touching the live workspace,
  publishing, pushing, external accounts, or system-wide installs stayed user-gated.

### Autonomous-use model

D0 recorded a usage ladder because the session could not reliably observe an “80% of five
hours” threshold. When Claude was fresh it could execute directly; near the usage limit,
Codex became the main executor while Claude/manager retained simulation, review, decisions,
and ground-truth verdicts; if both were exhausted, the correct action was to wait rather than
degrade quality. D18 later switched the lane to completely Codex for execution, with the
manager still verifying and committing. See §7 for the actual division of labour that the
repository records.

---

## 2. Chronological narrative

## 2.1 Phase A and S22 — the verified ground-cell baseline

### S22: repeat without respawn

**Goal.** Establish a durable baseline before rebuilding the scene: detect, pick, place,
re-detect the dropped object, repeat without respawning, and park the arm at `ready`.

**Approach and reason.** Re-detection after the previous drop tests the whole closed loop,
not merely a scripted pose tour. It also exposes discrepancies between the task's belief and
the physics world. The baseline remained ground-level because its perception and grasp were
calibrated for object centre `z=0.015`.

**Built.** The baseline snapshot includes the camera-enabled Panda URDF and bridges, the
MoveIt primitives in `move_to.py`, the original single-object detector, the industrial cell
world and launch files, and the S22 repeat-loop records (`S21_DEBUGGING_LOG.md` and
`S22_REPEAT_LOOP_LOG.md`). The initial mirror commit, `33ea912`, captured the entire verified
A1–A5 tree before any sorting-line changes.

**Errors and fixes inherited into V2.** The S21/S22 history established several rules that
governed the redesign:

- A fill light contaminated the original blue mask. The detected x coordinate moved from
  `0.501` to `0.472`, a `29 mm` shift. The hand then closed on the cube edge at width
  `0.0200`, barely inside the `0.015 ± 0.006` plausibility gate, while the task reported a
  pick/place that Gazebo showed had not happened. The fix was sun-only lighting.
- Blue camera-visible surfaces were forbidden because the original detector treated them as
  the target; the arm itself has blue indicator dots.
- The headless launch argument was initially a silent no-op. It was fixed and verified by
  checking that `-s --headless-rendering` was present and no GUI process remained.
- `rc=245` after `Done.` was classified in `V2_RESUME.md` as a harmless MoveItCpp teardown
  segfault. It must not override a completed ground-truth verdict.

**Verification.** The S22 live baseline completed `4/4` repeat cycles, re-detected at the
previous drop within approximately `5 mm`, and parked at `ready` (`V2_RESUME.md` §1;
`V2_PROGRESS.md`).

### A1: worlds and industrial cell

**Goal.** Move from an empty pick/place demonstration toward an industrial cell while keeping
known grasp geometry.

**Approach and reason.** A1a introduced `sorting_cell.sdf`; A1b introduced
`industrial_cell.sdf`. Industrial dressing was kept visual-only where collision would break
the ground grasp. The ground centre-height invariant (`z=0.015`) was not disturbed.

**Built.** `worlds/sorting_cell.sdf`, `worlds/industrial_cell.sdf`, their bringup launches,
the RGBD sensor/bridge configuration, Panda controller configuration, and the original
`grasp_box.sdf` path.

**Verification.** A1a placed the cube inside bins `2/2`. A1b detected x=`0.501` exactly and
measured the grasp at `0.0151` exactly on `2/2` runs (`V2_RESUME.md` §1).

### A2: measured reach and multi-object perception

**Goal.** Support multiple coloured/shaped ground objects and determine a layout from actual
top-down grasp reach rather than a rectangular reach assumption.

**Approach and reason.** `tools/reach_probe.py` used the real seeded-IK top-down grasp posture,
open gripper, and repeated passes. The selected layout retained the three proven bins at
`(0.45,+0.18)`, `(0.45,-0.18)`, and `(0.37,0)` and put staging at x≈`0.56`,
`|y|≤0.10`. Multi-object perception used HSV saturation/colour gates, connected components,
median depth, and pinhole deprojection, published as `Detection3DArray` alongside the legacy
single-object path.

**Why not a back-row rectangle.** Reach narrowed with `|y|`. Positions `(0.37,±0.20)` solved
IK but stalled `28–30 mm` above grasp height on every pass and retry (`0/2`), while
`(0.45,±0.18)`, `(0.37,0)`, `(0.50,0)`, `(0.52,±0.15)`, and `(0.58,0)` were `2/2` reachable.
An earlier one-shot probe even called the verified `(0.50,0)` spawn point unreachable, which
is why repeat passes and known-good controls became mandatory.

**Built.** Five cube/cylinder colour models; `industrial_cell_a2_bringup.launch.py`;
`object_detector_multi.py`; `tools/reach_probe.py`; `tools/check_detections.py`;
`tools/run_reach_probe.sh`; and `tools/run_a2_detector_check.sh`.

**Detector errors and fixes.** Gates were set from measured pixels and depth rather than model
poses. Real objects measured centre z=`0.015`; a flat red floor marker computed to z=`-0.003`,
so `CENTER_Z_TOL` tightened from `0.04` to `0.010`. The marker's visible centroid was
y=`0.272`, not its model y=`0.30`, because only `119` pixels of roughly `400` were in frame;
the ROI became ±`0.26`. Shape fill separated as predicted: cube ≈`0.99`, cylinder ≈`0.79`.
Objects also needed roughly `0.07 m` spacing because a gripper opened to ±`0.04 m`; at
`0.05 m` spacing a neighbour's near edge at `0.035 m` lay in the finger path.

**Verification.** Three consecutive A2 detector runs produced `3/3` detections, `3/3`
class-correct, `0` phantoms, errors `1.0/1.2/2.9 mm`, and `0` logged errors.

### A3/A4: collision-aware bins and imperative sorting

**Goal.** Sort the three ground objects by colour while planning around actual bin walls.

**Approach and reason.** Bins were modelled in MoveIt as four thin wall boxes, not a solid
obstacle. The ground plane was omitted because the verified fingertips graze it. The workflow
was made one-way: staging row → bin. That matters because the old ring-tour test tried to
descend into a collision-modelled bin to pick an object back out and correctly failed with
`plan_failed` / OMPL “Unable to solve”; it was the wrong test for the sorting workflow.

**Built.** `cell_scene.py`, `sort_task.py`, `tools/check_sort.py`, and
`tools/run_a4_sort_check.sh`. Routing was red→`bin_a`, green→`bin_b`, yellow→`bin_c`.

**Error: swallowed first finger command.** The first open command after bringup repeatedly
reported success while the fingers remained at their `0.02` spawn position against a
`0.0400` target. Three retries `1.5 s` apart did not move them; the next later command worked.
This proved a first-command initialisation defect, not a timing window. A half-open hand then
stalled about `29 mm` above the object and could shove a cylinder `0.3 m`. The durable fix was
a throwaway gripper prime stroke in `setup()`; open verification/retry remained as a safety
net (`GRIPPER_OPEN_TOL=0.005`, up to `3` attempts with `1.5 s` wait).

**Error: open fingers collided with bin walls on retract.** Releasing on the bin floor put an
open `0.08 m` span (about `0.10 m` padded) inside a `0.132 m` interior. MoveIt reported
contact between `bin_a` and `panda_rightfinger`, invalid states starting at index `1/84`, and
`retract_failed` even after physics had delivered the object. Task belief said `1/4` or `2/4`
while ground truth said `3/3`. `place(clearance=)` and
`BIN_RELEASE_CLEARANCE=0.035` moved fingertips about `8 mm` above the rim so the object could
drop the last few centimetres. Widening bins was rejected because bins `0.18 m` apart would
touch and crowd the staging row.

**Error: cylinder roll and the misleading early success.** A first cylinder contact read
`0.0200` instead of `0.0151`; its hold settled at `0.0275`; the size gate correctly refused
the grasp. On retry it appeared `14 mm` away, from `(0.559,0.001)` to `(0.559,-0.013)`, proving
roll-off. The early A4 result was `3/3` in three runs, but the rounded-position attempt key
treated a nudged object as new, enabling accidental retries bounded only by `LOOP_GUARD=12`.
This result was therefore retained as history, not accepted as deterministic cylinder
performance.

**Verification.** The clean A4 record reported `0/3 → 3/3`, z=`0.015`, with `0` planning
failures, gripper retries, arrival misses, and tracebacks on each of three runs. A5 later
qualified the cylinder part of that result as luck.

### A5: evidence layer and Phase-A review

**Goal.** Separate task belief from world truth and measure repeatability, shape performance,
failure health, and throughput.

**Approach and reason.** Optional SQLite logging avoided changing the no-database execution
path. The harness recorded both task outcomes and Gazebo poses. Identity changed from rounded
coordinates to nearest-previous pose (`MATCH_RADIUS=0.08`), with a deliberate
`MAX_ATTEMPTS_PER_OBJECT=2`.

**Built.** `trial_db.py`, `tools/benchmark.py`, and `tools/run_benchmark.sh`, producing
`logs/v2/benchmark.db` and per-run logs.

**Errors found by evidence.** In S2 the cylinder could roll out of the ROI or be flung to
`(-12.9,-1.99)`, disappearing entirely. Its honest deterministic score was `0/2`, so retries
or a wider `GRASP_WIDTH_TOL` were rejected; a future shape-aware/antipodal B2 grasp was the
proper remedy. The old log-health filter also counted roughly eight OMPL chatter lines per
logical planner failure and a benign octomap message: S2-run1 showed `17` “errors.” The
refined scan reduced this to `0` unexpected system errors while retaining the `2` real
logical planning failures reported in the reviewed ledger. The raw early harness output is
preserved in `logs/v2/benchmark_harness.out` and therefore still shows the pre-refinement
counts (`17` and `4` chatter-level planning matches); the reviewed numbers in
`V2_RESUME.md` are the intended interpretation.

**Verification and metrics.** Over two runs per scenario:

- S1 (`--max 1`): `2/2` pick success, `2/2` first-attempt, mean cycle `71.4 s`, throughput
  `0.62 objects/min`, approximately `96 s/run`. Ground truth was `2/6` overall because only
  one of three presented objects was intentionally attempted per run.
- S2: correct bin `4/6` (`66.7%`); pick `4/8` over all attempts and `4/7` on first attempts;
  mean cycle `76.4 s`; throughput `0.48 objects/min`, approximately `248 s/run`; cubes
  `4/5` first-attempt and `4/4` final, cylinders `0/2`.
- Both scenarios had `0` logged-vs-ground-truth divergence. The refined review records
  `0` unexpected errors across four runs.

The review made B2 a hard blocker only while mixed shapes remained in scope. Khoi's later
redesign chose identical boxes, removing that immediate blocker.

## 2.2 Belt-height redesign — Stages 0/A/B/C/D/E

### Locked product decisions and safe baseline (D1–D4)

Khoi locked three product choices before implementation:

1. **Static staging**, not a moving conveyor. Moving-target picking was a separate, higher-risk
   problem; a fixed infeed buffer is also industrially plausible.
2. **Identical coloured boxes**, not mixed shapes. This removed the cylinder roll/fling and
   made colour the only routing variable.
3. **Coloured tray-floor panels** with gray walls. The label is visible while remaining below
   the object-height gate; coloured walls or text added perception/modeling risk.

The mirror was then git-initialised and committed at `33ea912` because the absence of a diff,
rollback, and verified baseline was the largest unattended-run operational risk.

### Stage 0: raised reach gate before scene construction

**Goal.** Answer the highest-risk question first: can the Panda execute the actual top-down
approach/work/retract postures at surface z=`0.295`, box centre z=`0.320`, and all proposed
box/tray positions?

**Approach and reason.** `reach_probe_sorting_line.py` reused the seeded-IK grasp helper and
open-gripper behaviour. It tested three staging points `(0.560,+0.110/0/-0.110)`, three tray
points `(0.450,+0.180)`, `(0.450,-0.180)`, `(0.370,0)`, and a known-good ground control,
with two passes and three motion legs. Verdicts came from `/world/empty/pose/info`, not
`MoveResult.ok`.

**Error 1: invalid ground-truth frame.** The first probe requested `panda_link8`, but Gazebo
does not publish that massless flange. Published Panda frames ended at `panda_link7` plus
left/right fingers. The fix used the midpoint of `panda_leftfinger` and
`panda_rightfinger`, with a geometric “hand points down” check against `panda_link7`.

**Error 2: wrong finger-midpoint model.** `FINGER_REACH_Z=0.100` describes link8 to the grasp
point/fingertip region, not link8 to the finger-link frame origins. Reusing it predicted the
wrong midpoint. The known-good ground-control work pose deliberately stopped evaluation and
exposed this model error. The fix calibrated a constant top-down link8-target→finger-midpoint
offset from the ground work leg, sanity-gated it, then applied that offset to every leg. The
final calibrated delta was `-58.4 mm` in z with `0.0 mm` xy and was marked sane. The named
repository records do not preserve a separate numeric value for the rejected intermediate
model; see D9 and `tools/reach_probe_sorting_line.py` for that correction path.

**Teardown anomaly.** The retained reach log finishes with final-ready `PASS`, then
`timeout: the monitored command dumped core`, `probe rc=139` during MoveItCpp deletion. The
decision did not infer reach from that return code: it counted the preceding Gazebo pose
checks. This is recorded rather than hidden.

**Verification.** D5 records `42/42` legs passing: seven poses × approach/work/lift × two
passes. The gripper was open, `down_xy=0`, vertical drop was about `165 mm`, midpoint error
was approximately `0 mm`, and the final `ready` check passed. Verdict: GO at surface
`0.295` / box centre `0.320`; no lower/nearer fallback was required.

### Stage A: belt-height geometry

**Goal.** Create a new scene with no ground cubes/bins, three real belt-height box targets,
and three belt-height colour-labelled trays while preserving the old world.

**Approach and reason.** The new world was additive. The deck had real collision so settled
box height could prove support. Tray walls were collision geometry; coloured floor panels
provided labels without blue. The same Panda/camera and sun-only lighting were retained.

**Built.** `industrial_sorting_line.sdf`; `industrial_sorting_line_bringup.launch.py`;
`obj_red_box.sdf`, `obj_green_box.sdf`, `obj_yellow_box.sdf`; `check_sorting_line.py`; and
`run_sorting_line_check.sh`.

**Error: checker parser, not scene geometry.** The first checker used a
`setdefault`/next-name parsing pattern and mis-associated pose data. It was replaced by the
proven inline-capture parser and made to read `dynamic_pose/info` for moved models. Static
trays and deck are omitted from the pose topics, so raw output still says “trays present:
NONE” and `pick_belt present: False`; this is a topic limitation, not absent geometry. Tray
function was later proven by placement.

**Verification.** `logs/v2/sline_stageA.out` records two consecutive runs with all three
boxes at x=`0.560`, y=`+0.110/0/-0.110`, z=`0.320`, and `legacy=NONE`: `3/3 ×2`.

### Stage B: raised detector

**Goal.** Detect the larger raised boxes without allowing tray labels or the robot's blue
dots to become phantoms.

**Approach and reason.** The proven HSV/connected-component/median-depth detector was reused.
Because pinhole deprojection uses measured depth, raising the object should require a z/size
recalibration, not an ad-hoc xy remap. Constants became `OBJ_HALF_HEIGHT=0.025`,
`EXPECTED_CENTER_Z=0.320`, area `200..1600`, and ROI covering staging plus trays. Blue was
removed and every target shape was fixed to `box`.

**Phantom analysis.** Coloured tray panels computed to approximately z=`0.272`, below the
centre gate (`0.310` lower edge), and occupied about `5850 px`, above `MAX_AREA=1600`.
They were rejected independently by height and area. This was the safer choice than relying
on only one filter.

**Built.** `object_detector_sorting_line.py`, its setup entry, and
`check_detections_sorting_line.py`.

**Verification.** On each of two consecutive runs the detector returned `3/3`, all classes
correct, `0` phantoms. Ground-truth xy errors were green `2.7 mm`, red `3.4 mm`, yellow
`4.5 mm`; z error was `0.0 mm`. Worst xy=`4.5 mm` (`logs/v2/sline_stageB.out`).

### Stages C and D: collision-aware full colour sort

**Goal.** Execute the intended one-object policy—identify box, map colour to tray, pick,
place, return ready—until staging is clear, with no dead first phase.

**Approach and reason.** The sorter reused `_DetectionCache`, `_ObjectTracker`, and the same
MoveIt `pick`/`place` primitives. Only geometry changed: colour→tray mapping at belt height,
expected half-width `0.020` for a `40 mm` box, and release clearance `0.035`. `pick()` gained
an optional expected-width parameter whose old default retained ground-cube behaviour.
`sorting_line_scene.py` published hollow tray walls at z=`0.315`; deck and table remained
omitted because fingertips intentionally graze their surfaces, matching the A3 ground-plane
reasoning.

**Why the “redundant failed phase” disappeared.** The earlier visible first failure was not a
separate sorting algorithm phase: it was consistent with the rolling cylinder and the
first-command gripper issue. `setup()` still performs its intentional prime stroke, and
`pick()` still homes to `ready`; those are preparation/safety actions. Uniform boxes removed
the cylinder's off-centre roll, so every redesign pick succeeded first attempt.

**Built.** `sort_task_sorting_line.py`, `sorting_line_scene.py`, `run_sort_line_check.sh`,
the checker's `--placed` mode, and the backward-compatible `move_to.pick()` parameter.

**Verification.** Two consecutive runs placed all boxes correctly:

- Run 1 final positions: green `(+0.453,-0.177,+0.320)`, red
  `(+0.453,+0.175,+0.320)`, yellow `(+0.373,-0.000,+0.320)`.
- Run 2: green `(+0.453,-0.176,+0.320)`, red `(+0.453,+0.175,+0.320)`, yellow
  `(+0.373,+0.003,+0.320)`.

Both were `3/3`; log health was `0` unexpected system errors, planning failures,
tracebacks, and fatals. The six box picks were `6/6` first attempt. D9 reports roughly
`58 s/box`, `206 s/run`, and `0` logged-vs-truth divergence.

### Stage E: backward-compatibility regression and review

**Goal.** Prove the optional grasp-width change did not break the verified ground path.

**Approach and reason.** Run the old `industrial_cell_a2` + `sort_task` flow unchanged and
score it by Gazebo truth.

**Verification.** `logs/v2/sline_stageE_regression.out` shows the red and yellow cubes placed
cleanly. The green cylinder first produced `pick_grasp_width_implausible`, then recovered and
was placed. Task summary was `3/4 placed`, while the physically meaningful result was `3/3`
objects in their correct bins at z=`0.015`. There were `0` planning failures, tracebacks,
gripper-open retries, and arrival misses. The harness counted one non-benign task error—the
honest failed first cylinder attempt. Backward compatibility passed.

**Plan review.** Since boxes were now `6/6` first-attempt, B2 shape-aware grasp ceased to be
a current blocker. B1 hand-eye calibration became the highest-impact Phase-B item because
the working detector still encoded the exact camera extrinsic by hand.

## 2.3 Phase B — B1, B3, B4, B5

### B1: eye-to-hand extrinsic calibration

#### Plan and method

**Goal.** Replace the fragile hardcoded mapping
`world_x=0.4-y_opt`, `world_y=-x_opt`, `world_z=1.2-z_opt` with a measured rigid transform,
while preserving that formula as an empty-parameter fallback.

**Approach and reason.** D10 approved a scale-free Kabsch fit. Intrinsics already came from
`CameraInfo`; only `T_world_optical` was estimated. Correspondences paired the optical top
surface with the Gazebo world top surface—pairing optical top with model centre would create
a hidden `25 mm` z bias. A `3×3` xy grid plus three raised targets constrained all rotation
axes. Per-colour Hungarian assignment used the nominal transform only to identify points,
never as fit input. Scale remained a diagnostic: accepting scale would disguise depth or
centroid bias as calibration.

Strict gates were RMS≤`2 mm`, max≤`4 mm`, diagnostic scale `[0.998,1.002]`, geometry ratio
≥`0.01`, proper orthonormal rotation, and convention sanity within `2°/20 mm`.

#### First Stage-1/2 attempt: correct failure on tall boxes

**Built initially.** A neutral calibration platform and launch, twelve production
`40×40×50 mm` coloured boxes, `collect_extrinsic_calibration.py`,
`fit_extrinsic_calibration.py`, and `run_b1_calibration.sh`.

**Failure.** Both collections obtained `12/12` clean targets and `30` frames each, but the
fit produced RMS `3.0 mm` (>2), max `4.2 mm` (>4), and scale `1.0226`, outside
`[0.998,1.002]`. Residuals grew monotonically from `0.2 mm` at `30 mm` radius to `4.2 mm`
at `219 mm`. Off-axis cameras saw the `50 mm` box side faces; connected-component centroids
shifted outward, looking like `2.3%` scale error.

**Decision and fix.** Loosening the gates was rejected because it would encode biased data.
The target changed to dynamic `40×40×4 mm` flat colour tiles, the normal calibration choice
for avoiding parallax. The collector gained `--target-half-height`; boxes default to `0.025`,
tiles use `0.002`. All solver gates stayed unchanged.

#### Flat-tile Stage-1/2 pass

**Built.** `obj_{red,green,yellow}_tile.sdf`,
`industrial_sorting_line_calibration.sdf`,
`sorting_line_extrinsic_calibration.launch.py`, the collector, gated solver, runner, two raw
JSON datasets, and a candidate YAML.

**Verification.** Each capture had `12/12` entities, `30` valid frames, `0` rejected frames,
and geometry ratio `0.190083` against the `0.010` minimum. Across `24` median
correspondences, pooled and per-run results were:

- RMS `0.370 mm`; max `0.530 mm`; diagnostic scale `0.998041610`.
- geometry ratio `0.190083`; determinant `1.000000000000`; orthonormality approximately
  `2.23e-15`.
- run-to-run dR=`0.00000°`, dt=`0.000 mm`, cross-RMS=`0.370 mm`.
- nominal recovery error dR=`0.06048°`, dt=`0.741 mm`.
- fitted translation `(+0.400252950,-0.000694094,+1.200062810)`; the full matrix is preserved
  in `overhead_camera_extrinsic.yaml` and `logs/v2/b1_calibration.out`.

#### Stage-3/4 integration and A/B

**Approach.** `object_detector_sorting_line.py` gained optional `extrinsic_file`. Empty means
the exact old function; a specified valid file applies `p_world=R p_opt+t`; specified but
missing, malformed, non-orthonormal, improper, or wrong-frame configuration fails at startup
rather than silently falling back. The candidate was promoted to installed config.

**Built.** `config/overhead_camera_extrinsic.yaml`; detector loader/validation; launch
argument plumbing; calibrated options in the run harnesses.

**Verification.** The calibrated detector loaded the YAML and, on two consecutive runs,
returned `3/3` class-correct, `0` phantoms. Errors were green `3.2 mm`, red `4.4 mm`, yellow
`4.3 mm`, z=`0.1 mm`; worst xy=`4.4 mm`, no worse than the hardcoded `4.5 mm` baseline.
Two calibrated full-sort runs were each `3/3` correct tray with `0` system errors, planning
failures, tracebacks, or fatals. A deliberately missing/malformed requested file failed loud
as designed. B1 was complete.

### B2: shape-aware grasp — deliberately skipped/deferred

B2 was not silently forgotten. A5 had made it mandatory for mixed shapes because cylinders
scored `0/2`, but D1/D2 removed mixed shapes from the product. Uniform boxes then achieved
`6/6` first-attempt picks. Implementing antipodal/shape-aware grasp for a payload class no
longer present had lower practical value than calibration, orchestration, and monitoring.
B2 returns if shape-variety/S3 is reinstated.

### B3: self-contained behaviour tree

**Goal.** Re-express the verified imperative policy as inspectable industrial orchestration
with explicit retry/recovery, without changing perception or motion primitives.

**Approach and reason.** `py_trees` and `py_trees_ros` were not installed. A system-wide
apt/pip install was outside the mirror-only autonomy boundary. D14 therefore chose a tiny
pure-Python BT interface—`Status`, `Behaviour`, `Sequence`, `Selector`, `Retry`, `Leaf`—that
could later be replaced by py_trees. The per-object policy was
`Sequence[GetFreshDetections, Retry(Pick,2), Place, ReturnReady]`; the outer loop advanced
objects and retained `_ObjectTracker` attempt bounds.

**Built.** `behaviour_tree.py`, `sort_tree_sorting_line.py`, console entry
`sort_tree_sorting_line`, and runner support. It reused `setup/shutdown/move_to_named/pick/place`,
the existing detector cache/tracker, tray routing and release constants, collision scene,
and optional trial database.

**Verification.** The offline core self-test passed Sequence short-circuit, Selector
fallback, and bounded Retry semantics. Tick logs showed the intended sequence and recovery.
`logs/v2/b3_tree_sort.out` records two consecutive `3/3` correct-tray runs, all objects at
z=`0.320`, with `0` system errors, planning failures, tracebacks, or fatals. Both process
returns were the already-known post-run `245` teardown code.

### B4: independent runtime validator

**Goal.** Add an observe-only consistency monitor separate from task logic.

**Approach and reason.** Pure functions check each detection against FR-2 reach/height,
FR-3 class and physical overlap, and FR-8 tray-colour consistency. The ROS wrapper only
subscribes to `/detected_objects/array` and publishes JSON on `/validation/report`; it has no
motion, action, service, or mutating Gazebo interface.

**Built.** `sorting_validator.py` and console entry `sorting_validator`. Concrete limits are
x=`[0.30,0.62]`, `|y|≤0.27`, z=`0.320±0.010`, known classes red/green/yellow box, overlap
distance `<0.040 m`, and tray radius `0.075 m` at the production centres.

**Verification.** The offline self-test reported valid→OK and correctly flagged
`OUT_OF_ENVELOPE`, `WRONG_TRAY`, `UNKNOWN_CLASS`, and `OVERLAP`. D16 records a live report of
`detections=3`, `violations=0`, verdict `OK`.

**Process error: unsafe process matching.** During validation, `pkill -f sorting_validator`
matched the calling shell and exited `144`. The run was repeated using explicit PID/process
group cleanup. No code defect was inferred; the operational rule is never to use a pattern
that can match the command executing it.

### B5: reproducibility layer and Phase-B review

**Goal.** Make the work understandable and cheaply regression-testable outside the long
Gazebo runs.

**Built.** `README.md`, `Dockerfile`, `.github/workflows/ci.yml`, and
`tools/run_offline_tests.sh`.

**Verification.** D17 records the offline suite ALL PASS: BT, validator, and Kabsch solver
self-tests plus `py_compile` of every module. The Docker image itself and GitHub-hosted CI
were not run because they require external runtime/account resources.

**Review.** All Phase-B components remained additive: calibration was fail-loud when
requested and fallback-safe when absent; the BT wrapped existing primitives; the validator
was observe-only; and the offline suite added a fast guard. Resolved errors were the tall-box
parallax and the process-kill landmine. No Phase-B error remained open.

## 2.4 Phase C and the autonomous stopping point

The Phase-B review challenged the original Phase-C ambitions against industrial value and
the mirror-only/no-network/no-system-install boundary. C1 learned perception had low marginal
value while classical HSV already achieved `3/3` and under `5 mm`; it also needed training
infrastructure. C2 VLM planning needed an API key and network. Docker execution, GitHub CI,
video capture, and remote publishing needed user resources/accounts.

The autonomously complete C3 deliverable was therefore the written capstone
`V2_PROGRESS.md` plus a local `v2.0` tag. Commit `7693ac6` created the final report and points
at that local tag. No push or release publication was performed. D18 then switched execution
fully to Codex for this exhaustive log; the manager retained accuracy verification and the
commit.

---

## 3. Full file inventory

The initial `33ea912` commit is a 141-file snapshot of the already-built ROS workspace; its
complete path-level stat is available with `git show --stat 33ea912`. The inventory below is
exhaustive for the functional Phase-A baseline artifacts named by the engineering ledger and
for every source/tool added or modified from that baseline through `v2.0`. Generated build
trees, historic `.bak` files, binary diagrams, and raw S21 acceptance data are evidence, not
active source, and remain enumerated by that commit's stat.

### Baseline / Phase-A functional source

- `src/twin_description/urdf/panda.urdf` — Panda model, ros2_control, fixed overhead RGBD
  camera, and optical frame.
- `src/twin_description/config/panda_controllers.yaml` — arm/gripper controller setup.
- `src/twin_action_demo/config/panda_bridge.yaml` — Gazebo↔ROS bridge topic mappings.
- `src/twin_action_demo/launch/bridge.launch.py` — launches the configured bridge.
- `src/twin_action_demo/launch/panda_bringup.launch.py` — original robot/camera bringup.
- `src/twin_action_demo/launch/sorting_cell_bringup.launch.py` — A1a sorting-cell bringup.
- `src/twin_action_demo/launch/industrial_cell_bringup.launch.py` — A1b industrial scene.
- `src/twin_action_demo/launch/industrial_cell_a2_bringup.launch.py` — A2 multi-object scene.
- `src/twin_action_demo/worlds/sensors_empty.sdf` — camera/sensor test world.
- `src/twin_action_demo/worlds/sorting_cell.sdf` — A1a ground sorting cell.
- `src/twin_action_demo/worlds/industrial_cell.sdf` — verified legacy industrial world.
- `src/twin_action_demo/models/grasp_box.sdf` — original graspable box model.
- `src/twin_action_demo/models/obj_red_cube.sdf` — red ground cube payload.
- `src/twin_action_demo/models/obj_green_cube.sdf` — green ground cube payload.
- `src/twin_action_demo/models/obj_yellow_cube.sdf` — yellow ground cube payload.
- `src/twin_action_demo/models/obj_red_cylinder.sdf` — red ground cylinder payload.
- `src/twin_action_demo/models/obj_green_cylinder.sdf` — green cylinder that exposed
  roll/fling behaviour in the verified A2 launch.
- `src/twin_action_demo/setup.py` — installs demo launches, worlds, models, and configuration.
- `src/twin_moveit_scripts/twin_moveit_scripts/move_to.py` — verified MoveIt/gripper
  primitives, arrival checks, grasp record, prime stroke, and relative pick/place geometry.
- `src/twin_moveit_scripts/twin_moveit_scripts/object_detector.py` — legacy single-object
  detector.
- `src/twin_moveit_scripts/twin_moveit_scripts/object_detector_multi.py` — A2 HSV/depth
  multi-object detector.
- `src/twin_moveit_scripts/twin_moveit_scripts/cell_scene.py` — hollow ground-bin collision
  objects.
- `src/twin_moveit_scripts/twin_moveit_scripts/sort_task.py` — imperative legacy colour sort,
  fresh detection cache, and identity tracker.
- `src/twin_moveit_scripts/twin_moveit_scripts/trial_db.py` — SQLite runs/trials/truth schema
  and writers.
- `src/twin_moveit_scripts/setup.py` — package console entries; later extended for V2 nodes.
- `src/twin_moveit_config/config/ompl_planning.yaml` — verified OMPL planner definitions used
  by the motion primitives.
- `src/twin_moveit_config/config/panda.srdf` — Panda semantic groups and named states.
- `src/twin_moveit_config/config/moveit_controllers.yaml` — MoveIt execution-controller map.
- `tools/reach_probe.py` — repeated ground reach measurement logic.
- `tools/run_reach_probe.sh` — ground reach bringup and runner.
- `tools/check_detections.py` — A2 perception-vs-truth checker.
- `tools/run_a2_detector_check.sh` — three-run A2 detector harness.
- `tools/check_sort.py` — legacy placement-vs-truth checker.
- `tools/run_a4_sort_check.sh` — legacy sort harness.
- `tools/benchmark.py` — A5 SQLite truth recorder and metrics report.
- `tools/run_benchmark.sh` — consecutive S1/S2 benchmark harness and health scan.
- `tools/run_groundtruth.sh` — durable headless truth harness.
- `tools/verify_empty_headless.launch.py` — empty-scene verification launch.
- `tools/verify_industrial_headless.launch.py` — bare industrial verification launch reused
  by Stage 0.

### Belt-height redesign additions and modifications

- `V2_DECISIONS.md` — D0–D18 option/reason/verdict ledger.
- `V2_RESUME.md` — engineering state, rules, historical faults, and resume instructions.
- `src/twin_action_demo/worlds/industrial_sorting_line.sdf` — pick deck, tray table, and
  three colour-floored trays.
- `src/twin_action_demo/launch/industrial_sorting_line_bringup.launch.py` — production raised
  bringup with optional detector/extrinsic.
- `src/twin_action_demo/models/obj_red_box.sdf` — red `40×40×50 mm` production payload.
- `src/twin_action_demo/models/obj_green_box.sdf` — green production payload.
- `src/twin_action_demo/models/obj_yellow_box.sdf` — yellow production payload.
- `src/twin_moveit_scripts/twin_moveit_scripts/object_detector_sorting_line.py` — raised HSV
  detector and optional calibrated extrinsic loader.
- `src/twin_moveit_scripts/twin_moveit_scripts/sorting_line_scene.py` — hollow raised-tray
  collision walls.
- `src/twin_moveit_scripts/twin_moveit_scripts/sort_task_sorting_line.py` — verified
  imperative raised sorter.
- `src/twin_moveit_scripts/twin_moveit_scripts/move_to.py` — modified only to parameterise
  the expected grasp half-width while retaining its legacy default.
- `src/twin_moveit_scripts/setup.py` — new detector, imperative sorter, BT, and validator
  console entries.
- `tools/reach_probe_sorting_line.py` — two-pass raised free-space probe with finger-midpoint
  Gazebo truth and ground calibration.
- `tools/run_reach_probe_sorting_line.sh` — headless Stage-0 runner/precheck.
- `tools/check_sorting_line.py` — raised geometry and `--placed` truth checker.
- `tools/check_detections_sorting_line.py` — raised detector-vs-Gazebo checker.
- `tools/run_sorting_line_check.sh` — consecutive Stage-A/B harness.
- `tools/run_sort_line_check.sh` — full raised-sort, truth, DB, and health harness.

### B1 calibration source and data

- `src/twin_action_demo/worlds/industrial_sorting_line_calibration.sdf` — neutral platform
  and three raised pedestals under production camera geometry.
- `src/twin_action_demo/launch/sorting_line_extrinsic_calibration.launch.py` — camera/robot
  plus twelve stable calibration target IDs.
- `src/twin_action_demo/models/obj_red_tile.sdf` — flat red `40×40×4 mm` target.
- `src/twin_action_demo/models/obj_green_tile.sdf` — flat green target.
- `src/twin_action_demo/models/obj_yellow_tile.sdf` — flat yellow target.
- `tools/collect_extrinsic_calibration.py` — 30-frame optical-top/world-top collector,
  stability/upright gates, dependency-free Hungarian association, and conditioning report.
- `tools/fit_extrinsic_calibration.py` — scale-free Kabsch, strict residual/scale/frame gates,
  repeatability checks, candidate writer, and offline self-test.
- `tools/run_b1_calibration.sh` — two-fresh-launch collection and fit runner.
- `logs/v2/calibration/dataset_run1.json` — first accepted 12-target/30-frame dataset.
- `logs/v2/calibration/dataset_run2.json` — second dataset.
- `logs/v2/calibration/overhead_camera_extrinsic.candidate.yaml` — gated candidate output.
- `src/twin_action_demo/config/overhead_camera_extrinsic.yaml` — promoted production
  world←`camera_optical_link` transform and fit metadata.

### B3/B4/B5 and capstone

- `src/twin_moveit_scripts/twin_moveit_scripts/behaviour_tree.py` — dependency-free BT core
  and offline semantics test.
- `src/twin_moveit_scripts/twin_moveit_scripts/sort_tree_sorting_line.py` — per-object tree,
  bounded retry/recovery, DB accounting, and final park.
- `src/twin_moveit_scripts/twin_moveit_scripts/sorting_validator.py` — FR-2/3/8 monitor and
  pure offline invariant tests.
- `tools/run_offline_tests.sh` — BT, validator, Kabsch, and full Python compile suite.
- `README.md` — public project narrative, architecture, results, and run commands.
- `Dockerfile` — ROS 2 Jazzy / Gazebo Harmonic build environment definition.
- `.github/workflows/ci.yml` — GitHub job invoking the offline suite.
- `V2_PROGRESS.md` — C3 executive summary and user-gated remainder.
- `V2_FULL_LOG.md` — this exhaustive synthesis, written after D18.

### Retained top-level run evidence

- `logs/v2/reachline_runner.out` — final raised probe execution and teardown anomaly.
- `logs/v2/sline_stageA.out` — two geometry passes.
- `logs/v2/sline_stageB.out` — two hardcoded-detector truth passes.
- `logs/v2/sline_stageCD.out` — two imperative full sorts.
- `logs/v2/sline_stageE_regression.out` — legacy regression.
- `logs/v2/benchmark_harness.out` — raw A5 scenarios and metrics.
- `logs/v2/b1_calibration.out` — two tile captures plus fit matrix/gates.
- `logs/v2/b1_ab_detector.out` — calibrated detector truth A/B evidence.
- `logs/v2/b1_ab_sort.out` — calibrated full-sort evidence.
- `logs/v2/b3_tree_sort.out` — two BT-driven ground-truth sorts.

---

## 4. Git history and release state

The repository records these commits, oldest first:

| Commit | Delivery |
|---|---|
| `33ea912` | Baseline of the verified A1–A5 mirror before the belt-height redesign; establishes diff/rollback safety. |
| `eaae128` | Stage-0 raised reach probe and runner; `42/42` Gazebo-truth legs, calibrated finger-mid delta `-58.4 mm`; introduces decision/autonomy ledger. |
| `cdd7823` | Stages A+B: raised world, three box models, launch, checkers, and detector; geometry and perception verified ×2. |
| `7e53793` | Stages C+D: raised imperative sorter, tray collision scene, width-gate parameter, and two `3/3` sorts. |
| `601cf48` | Stage-E legacy regression and redesign review; reprioritises Phase B to B1 and deprioritises B2 for uniform boxes. |
| `842d569` | B1 Stages 1+2: flat-tile world/models, collector, Kabsch solver, runner, datasets, and verified candidate. |
| `6359b4c` | B1 complete: promoted extrinsic, strict optional detector loader, launch/harness A/B; calibrated detector and sort pass. |
| `c1120fa` | B1 closure plus D14 decision to build a self-contained BT instead of installing py_trees. |
| `21eb206` | B3 BT core/tree and runner integration; `3/3 ×2`. |
| `feaf2ae` | B4 validator, self-test, and live OK result. |
| `f4db6ca` | B5 README, Dockerfile, CI, offline suite, and Phase-B review. |
| `7693ac6` | Phase-C `V2_PROGRESS.md` capstone; current `master` and local `v2.0` target. |

The annotated local tag is:

> `v2.0 — factory sorting cell: belt-height colour sort, hand-eye calibration, behaviour-tree orchestration, runtime validator; all ground-truth verified (local tag; push is user-gated)`

No remote push or public release is claimed. Those are explicitly reserved for Khoi.

---

## 5. Verified-results consolidation

| Milestone | Ground-truth / measured result | Evidence |
|---|---|---|
| S22 repeat loop | `4/4`; re-detect within ~`5 mm`; parks ready | `V2_RESUME.md`, `V2_PROGRESS.md` |
| A1a/A1b | A1a cube-in-bin `2/2`; A1b x=`0.501`, grasp=`0.0151`, `2/2` | `V2_RESUME.md` §1 |
| Ground reach layout | proven poses `2/2`; `(0.37,±0.20)` `0/2`, stalls `28–30 mm` high | `V2_RESUME.md` §2 |
| A2 detector | `3/3`, class `3/3`, `0` phantoms ×3; errors `1.0/1.2/2.9 mm` | `V2_RESUME.md` §1 |
| A4 historical sort | `3/3 ×3`, later qualified as lucky for the cylinder | `V2_RESUME.md` A4/A5 review |
| A5 S1 | pick `2/2`, first attempt `2/2`; `71.4 s` cycle; `0.62/min` | `benchmark_harness.out` |
| A5 S2 | truth `4/6`; cubes final `4/4`, first `4/5`; cylinders `0/2`; `0` divergence | `benchmark_harness.out`, reviewed in `V2_RESUME.md` |
| Raised Stage 0 | `42/42`; delta z=`-58.4 mm`; down_xy=`0`; drop≈`165 mm` | D5, `reachline_runner.out` |
| Raised Stage A | three boxes z=`0.320`, `3/3 ×2`, no legacy entities | `sline_stageA.out` |
| Raised Stage B baseline | `3/3`, `0` phantoms ×2; xy `2.7/3.4/4.5 mm`; z `0.0 mm` | `sline_stageB.out` |
| Imperative C/D sort | `3/3` correct tray ×2; picks `6/6` first attempt; health all zero | `sline_stageCD.out`, D8/D9 |
| Stage E legacy | truth `3/3`; cylinder fails once then succeeds; no planning/arrival failures | `sline_stageE_regression.out` |
| Tall-box calibration | correctly FAIL: RMS `3.0 mm`, max `4.2 mm`, scale `1.0226`; radial `0.2→4.2 mm` | D11 |
| Flat-tile calibration | RMS `0.370 mm`, max `0.530 mm`, scale `0.998041610`, geometry `0.190083` | `b1_calibration.out` |
| Extrinsic recovery | dR=`0.06048°`, dt=`0.741 mm`; repeat dR=`0`, dt=`0`, cross-RMS=`0.370 mm` | `b1_calibration.out` |
| Calibrated detector | `3/3`, `0` phantoms ×2; xy `3.2/4.4/4.3 mm`, z=`0.1 mm` | `b1_ab_detector.out` |
| Calibrated full sort | `3/3` correct tray ×2; health all zero | `b1_ab_sort.out` |
| BT sort | `3/3` correct tray ×2; health all zero; tick sequence/recovery visible | `b3_tree_sort.out`, D15 |
| Validator | all synthetic cases flag correctly; live `3` detections, `0` violations, OK | D16 |
| Offline suite | ALL PASS: BT + validator + solver self-tests + py_compile | D17 |

---

## 6. Khoi's original redesign asks and disposition

| Ask | How it was met | Verification |
|---|---|---|
| Remove little red/green/yellow cubes and ground sorting trays from the new scene | `industrial_sorting_line.sdf` is additive and contains the raised deck/trays; old assets remain only in the legacy world | Stage A: `legacy=NONE`, `3/3 ×2` |
| Use boxes on the belt as real targets | Three dynamic `40×40×50 mm` colour boxes spawn at x=`0.560`, y=`+0.110/0/-0.110`, centre z=`0.320` | Stage A truth and C/D picks |
| Identify each box by colour and route to its matching tray | Detector publishes `red_box`, `green_box`, `yellow_box`; sorter maps to matching tray centres | Detector `3/3`; sort `3/3 ×2` |
| Put trays at the same height as the belt | Tray floor top and pick deck top are z=`0.295`; nominal box/place centre z=`0.320` | All final boxes rest z=`0.320` |
| Make each tray visibly declare its colour | Saturated floor panel per tray with gray walls; no blue | Visual world design; panels produce `0` phantoms |
| Eliminate the dead first move/failure | Uniform boxes removed cylinder roll; intentional prime/home retained as safety preparation | Picks `6/6` first attempt; no redundant failed sort phase |
| Preserve the old verified pipeline | New world/launch/detector/sorter live alongside the old path; `pick()` default stayed compatible | Stage E legacy truth `3/3` |

All four owner-facing scene/flow asks listed in `V2_PROGRESS.md` are therefore met; the final
row above records the engineering safety constraint that made the change additive.

---

## 7. Claude-versus-Codex split

The repository records a manager/executor collaboration, not a reliable token ledger.
Exact per-call token counts exist only in the manager's session and are **not in the repo**;
none are invented here.

### Work explicitly delegated to or recorded as Codex

- Stage-0 planning and the raised reach-probe artifact/correction iterations; commit
  `eaae128` explicitly calls the probe Codex-written.
- The read-only B1 calibration design (D10), followed by the calibration world/launch,
  collector, dependency-free Hungarian assignment, Kabsch solver, flat-tile correction, and
  strict detector integration in the delegated build lane.
- The self-contained B3 behaviour-tree core and sorter, preserving existing primitives and
  adding bounded reactive recovery/tick logs.
- The B4 validator node and pure offline invariant checks.
- D18's first completely-Codex task: this exhaustive `V2_FULL_LOG.md` synthesis.

Qualitatively, Codex handled closed-spec planning and mechanical implementation well, but the
Stage-0 and B1 correction rounds show why manager verification mattered: an apparently
reasonable EE frame or target geometry could still invalidate the experiment.

### Work done directly or reserved to the manager/Claude

- D6 explicitly records Stage-A geometry as Claude-complete/direct.
- The manager owned all heavy Gazebo launches, clean-environment builds, raw evidence capture,
  ground-truth verdicts, log-health interpretation, and repeated-run acceptance.
- The manager diagnosed and supplied correction evidence for absent `panda_link8`, the
  finger-origin offset model, and tall-box radial parallax; chose not to loosen gates; and
  decided the flat-target correction.
- The manager maintained D0–D18, stage reviews, priority changes, B2 deferral, C1/C2 gating,
  local commits, and the `v2.0` tag.
- B5 and the Phase-C capstone were completed/reviewed in the manager lane; the git record
  proves their artifacts but does not justify a finer per-call authorship claim.
- Pushes, release publication, live-workspace edits, system installs, and external-account
  actions remained reserved to Khoi.

Commit `Co-Authored-By` trailers document collaboration but do not, by themselves, measure
who typed each line or how many tokens were spent.

---

## 8. Decision index (D0–D18)

Each entry points to the authoritative reasoning in `V2_DECISIONS.md`.

- **D0 — Operating mode and usage ladder:** choose a state-dependent Claude/Codex ladder so
  the autonomous run continues without pretending an unobservable usage threshold is known.
- **D1 — Belt behaviour:** static staging; moving-target picking is a different risk class.
- **D2 — Box scheme:** identical coloured boxes; colour-only routing removes cylinder roll.
- **D3 — Tray labels:** coloured floor panels with gray walls; visible but doubly rejectable
  by the object detector.
- **D4 — Git baseline:** initialise/commit the mirror for diff, rollback, and unattended safety.
- **D5 — Raised reach:** GO at surface `0.295`, box centre `0.320`, after `42/42` GT legs.
- **D6 — Stage A:** new geometry passes `3/3 ×2`; checker parser error fixed.
- **D7 — Stage B:** raised detector passes `3/3`, `0` phantoms, worst xy `4.5 mm ×2`.
- **D8 — Stages C/D:** full belt-height sort perfect `3/3 ×2`; original asks met.
- **D9 — Stage E/review:** old path still `3/3`; B1 promoted above now-unneeded B2.
- **D10 — B1 plan:** approve top/top scale-free Kabsch, non-degenerate targets, strict A/B.
- **D11 — Tall-box failure:** reject looser gates; use flat calibration tiles.
- **D12 — Flat-tile pass:** RMS `0.370 mm`, recovery `0.060°/0.74 mm`, repeatable.
- **D13 — B1 integration:** calibrated detector and full sort pass ×2; B1 complete.
- **D14 — BT dependency choice:** no unapproved system install; build a replaceable pure-Python
  core.
- **D15 — B3 pass:** self-test and tree-driven `3/3 ×2`, no errors.
- **D16 — B4 pass:** validator self/live checks pass; records `pkill -f` exit-144 lesson.
- **D17 — B5/review:** docs, Docker/CI definitions, offline suite all pass; external Phase-C
  work becomes user-gated.
- **D18 — Complete Codex mode:** Codex writes/builds; manager verifies and commits, beginning
  with this log.

---

## 9. User-gated remainder

These items were deliberately not represented as completed:

- **C1 learned perception:** needs synthetic/training infrastructure and a deliberate value
  decision. Classical HSV already delivers `3/3`, `0` phantoms, and <`5 mm` xy, so the
  marginal gain must justify training/maintenance cost.
- **C2 VLM task planning:** needs an LLM/VLM API key, network access, and an ablation against
  the deterministic rule-based planner. Those resources and external calls were outside the
  autonomous boundary.
- **Docker image build:** the `Dockerfile` exists, but an actual `docker build` requires a
  Docker runtime and potentially external package access on Khoi's machine.
- **CI execution:** the GitHub workflow exists, but a run requires a remote repository and
  GitHub runner/account.
- **Demo video:** requires an approved capture/presentation choice and user environment; the
  headless evidence runs were prioritised over an unverified visual demo.
- **Git push and `v2.0` publication:** the tag is local. Selecting/configuring the remote,
  pushing commits/tags, and publishing a release affect external state and are Khoi-gated.

Conditional remainder: **B2 shape-aware grasp** is not needed for the locked uniform-box
product. It becomes necessary if mixed cylinders/shapes or S3 are brought back.

---

## 10. Lessons and landmines

1. **Ground truth is the verdict.** Task summaries under-reported delivered objects after
   retract failure and over-reported an unmoved cube under lighting contamination. Read
   `/world/empty/dynamic_pose/info` / `pose/info` and keep belief-vs-truth metrics.
2. **Logs still matter for health.** Count logical failures, not every downstream OMPL line;
   keep errors, tracebacks, controller failures, and fatals separate from success.
3. **Repeat runs and known-good controls.** A one-shot probe falsely rejected `(0.50,0)`.
   Require N≥2 and distinguish reach limits from stochastic planning.
4. **Reach is not rectangular.** `(0.37,±0.20)` failed while nearby centre/side poses passed.
   Probe the real posture at the real height before committing world geometry.
5. **Use relative heights.** `pick`/`place` derive approach/work/retract from object or target
   z. Belt height then needed a width-gate change, not a forked motion stack.
6. **Depth deprojection preserves xy across height.** Correct intrinsics plus median depth
   kept xy under `5 mm`; the main raised-scene detector changes were object half-height,
   expected z, ROI, area, and class set.
7. **Calibrate frame meanings, not names.** Gazebo omitted massless `panda_link8`; finger-link
   origins were not fingertips. A verified ground control made the measurement model
   falsifiable and produced the sane `-58.4 mm` offset.
8. **Pair like surfaces.** Optical top must pair with world top. Model centre would inject a
   hidden `25 mm` error for the production boxes.
9. **Use flat calibration targets.** Tall boxes exposed side faces and created radial
   centroid bias/false scale. Flat tiles reduced RMS from `3.0` to `0.370 mm`; never loosen a
   gate just to pass biased data.
10. **Do not add camera-visible fill light or blue.** Fill light once shifted a centroid
    `29 mm`; blue belongs to the arm's indicators/original target classifier.
11. **A first gripper command can be swallowed.** Verify actual finger width; a prime stroke
    is an intentional initialisation action, not an object-sorting phase.
12. **Do not widen a grasp plausibility gate to hide roll-off.** The cylinder's `0/2`
    benchmark was an upstream grasp-geometry problem, not a threshold problem.
13. **Model hollow containers as walls.** Solid bins make every placement impossible. Release
    above the rim (`0.035` clearance here) rather than planning a retract with open fingers
    already in wall collision.
14. **Static models may be absent from pose topics.** A checker should not infer absent
    geometry from that alone; test its physical function or inspect the world.
15. **Run headless and clean up exact PIDs/PGIDs.** GUI processes throttle real-time factor
    and orphan. Never `pkill -f` a pattern matching the caller; the recorded consequence was
    exit `144`.
16. **A post-completion teardown code is not a functional verdict.** `245` is documented as
    benign after completion; the Stage-0 retained log also records rc=`139` only after final
    ground-truth PASS. Preserve and diagnose these, but do not replace physics evidence with
    them.
17. **Additive delivery makes long autonomous work survivable.** Old launch/world/detector/
    sorter paths remained available until the new path passed and for Stage-E regression.
18. **Keep external authority explicit.** A local tag, Dockerfile, or CI definition is not a
    pushed release, built image, or completed hosted run.

---

## 11. Evidence map and final state

For quick independent checking:

- Decisions and reasoning: `V2_DECISIONS.md` D0–D18.
- Engineering history/landmines: `V2_RESUME.md`.
- Executive result and remainder: `V2_PROGRESS.md`.
- Operator-facing commands: `README.md`.
- Raw stage outputs: `logs/v2/*.out` listed in §3.
- Exact file/commit provenance: `git log --stat`, `git log --oneline --decorate --all`.
- Local release marker: `git tag -n` (`v2.0`); remote publication is not done.

At the recorded stopping point, the mirror contains a ground-truth-verified belt-height
colour sorter, data-driven camera extrinsic with safe fallback, imperative and behaviour-tree
orchestrators, an independent runtime validator, regression evidence for the legacy ground
path, offline tests, documentation, and reproducibility definitions. The live workspace was
not touched. The remaining work in §9 is waiting on Khoi by design, not hidden as unfinished
autonomous execution.
