# Parcel-cell path-planning development

Work in `ros_test_ws_mirror` only. Preserve the working RRTConnect + TOTG pipeline,
the camera calibration, payload attachments and legacy callers. No commit, primary
promotion or new heavy dependency is implicit. Feature defaults preserve current
behavior. Research input: `~/Downloads/Path_Planning_Development_Plan/`, read as
reference data; downloaded Python code is not executed.

## Adjustments after inspecting this checkout

- Historical indexed cycle improvement 182.4 -> 92–93 simulation seconds is recorded
  in `MOTION_SPEED_PLAN.md`; it is not a fresh result for this working tree.
- BIT* and Informed-RRT* are not configured here. The installed MoveIt OMPL library
  advertises RRTstar, PRMstar and AnytimePathShortening; it does not advertise BITstar
  or InformedRRTstar. Do not silently map an unavailable planner to RRTConnect.
- Installed CHOMP/STOMP are planner plugins. Their presence alone does not prove
  that a release-compatible postprocessing adapter or sequence endpoint is active.
- TOTG can move the path within its tolerance. Recheck the final retimed motion,
  including attached payload geometry, before testing a new optimizer.
- Current available RAM is approximately 5.4 GiB, with 2.9 GiB swap used. Run one
  simulator/optimizer at a time. TrajOpt, GPMP2 and GCS/Drake remain deferred.

## Stages and gates

| Stage | Deliverable | Gate before adoption |
|---|---|---|
| 0 | Optional JSONL recorder, final-trajectory sampled validator, source/profile/scene hashes and fresh baseline | Offline malformed-input tests; indexed ×2; independent placements and health |
| 1 | `TWIN_PLANNER_ID`, matched planning budgets; RRTConnect vs RRTstar/PRMstar/APS | Two consistent trials for a candidate, shorter paths without worse cycle/placement/health; retain incumbent if none wins |
| 2 | Optional bounded OMPL -> STOMP/CHOMP smoothing, then TOTG | Verify installed chaining support; endpoint/geometry checks; compare stage-1 incumbent; no claimed improvement without evidence |
| 3 | Pilz sequence capability and planning-only adapter first; unloaded PTP baseline then blends | Synchronized scene, nonoverlapping spheres, final radius zero, zero blend at contact; full payload collision recheck; one execution owner |
| 4 | TOPPRA offline retiming of saved paths, optional runtime only after it wins | Same effective limits and pinned endpoints; path-fit deviation and residual checks; no runtime dependency if comparison fails |
| 5 | Shadow per-joint ArrivalGate and richer execution events | Freshness, distinct samples, dwell, clock reset and controller-ID fault tests; shadow never replaces arrival by default |

Structural validation is brought forward into stage 0. Stages 2–4 require a passing
measurement/validation baseline; failure is a reason to diagnose or retain the
incumbent, not weaken the gate. Stage 5 can be developed offline in parallel with
benchmarking but activation remains separate. Full dynamics/contact/jerk validation
is explicitly NOT_CHECKED; sampled collision checks are not continuous certificates.

## Experiment interface and rollback

- `TWIN_MOTION_METRICS=/absolute/run/motion.jsonl`: opt-in per-process motion records.
  Harness `--motion-metrics` assigns a separate file per run.
- `TWIN_TRAJECTORY_CHECK=sampled`: exact arm joint order, finite positions/derivatives,
  strict times and the intersection of URDF hard/soft/configured position and velocity
  limits (configured acceleration limits), with v/a scaling, followed by dense collision sampling
  of the final q/v/a spline. `off` is the unchanged default.
- `TWIN_PLANNER_ID=RRTConnectkConfigDefault` (default), `RRTstarkConfigDefault`,
  `PRMstarkConfigDefault`, or `APSConfigDefault`; reject unknown IDs.
- `TWIN_PLANNING_TIME=5.0`, `TWIN_PLANNING_ATTEMPTS=5`: existing defaults, held
  constant for comparisons. A budget is not an instruction to spend it all.
- `TWIN_SMOOTHER=stomp`: bounded OMPL -> STOMP -> TOTG experiment, separate
  `ompl_smooth` pipeline; requires sampled preflight. Default `off`. A returned
  planning failure tries the incumbent pipeline; rejected final candidates still
  fail preflight. This option failed the current motion acceptance gate (below).
- `TWIN_ARRIVAL_SHADOW=/absolute/profile.json`: requires metrics output and an
  explicit seven-joint `ArrivalProfile` JSON. Uses a separate ROS node/executor,
  measured positions/velocities and ROS simulation stamps; reports its decision
  beside the existing settle result without extending/shortening the wait.
- Existing `TWIN_VEL_SCALE` / `TWIN_ACC_SCALE` stay at 0.5. Do not speed up by
  reducing perception, grasp verification or placement checks.
- Disable the experiment flags to restore the incumbent. Before-session copies of
  edited tracked code are in `logs/path_planning/prechange/`; these include earlier
  uncommitted work and must not be replaced with `git checkout`/reset.

## Validation commands

Use clean shell, fresh log directory and free ROS domain; the harness creates an
isolated Gazebo partition and cleans only marked processes.

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin TWIN_TRAJECTORY_CHECK=sampled \
  bash --noprofile --norc tools/run_parcel_cell_check.sh --mode indexed \
  --motion-metrics --runs 2 --domain 92 --logdir logs/path_planning/baseline
python3 tools/motion_metrics.py logs/path_planning/baseline/run1/motion.jsonl \
  --output logs/path_planning/baseline/run1/motion_metrics.json
bash tools/run_offline_tests.sh
```

For a planner comparison, add `TWIN_PLANNER_ID=<config>` with the same budget,
scaling, scene and checks. After shared-code changes, run colour ×2 with all
experiment flags absent. Also run continuous confirmed ×2 before recommending
experiments for the moving-belt cell: planner latency can miss an intercept even
when indexed placement succeeds. Planned path metrics do not measure tracking or
grasp quality. Use the existing ground-truth checkers and `task.json` events.
`benchmark.py` remains the existing SQLite throughput reporter; no second DB is added.

## Evidence and status

All evidence below is new for this task, under `logs/path_planning/` (ignored by
Git). Source hashes are in each run's manifest. Historical motion/grasp records
remain in their own ledgers. No candidate is adopted merely because installed.

| Comparison | Indexed READY->FULL (sim s) | Total planning (wall s) | Joint path (rad) | EE path (m) | Verdict |
|---|---:|---:|---:|---:|---|
| RRTConnect baseline, run 1 | 91.467 | 1.690 | 17.6802 | 5.05818 | 3/3 independent placements, exit 0, no health errors |
| RRTConnect baseline, run 2 | 92.232 | 1.648 | 17.6773 | 5.05657 | Same full pass |
| RRTstar, run 1 | 146.996 | 115.708 | 17.6749 | 5.05525 | 3/3, all checks pass; rejected for cycle regression |
| RRTstar, run 2 | 140.488 | 115.660 | 17.6689 | 5.05246 | Same outcome; no adoption |

These are matched 5-second/5-attempt budgets and .5 v/a scaling, using the same
incumbent TOTG chain blending. The baseline already produces very direct paths:
RRTstar's travel improvement is negligible while planning consumes the full budget.
Further runtime optimal-planner sweeps (PRMstar/APS) are gated on a representative
detour scenario that the incumbent actually needs to improve; no unsupported
BIT*/Informed-RRT* configuration is added.

STOMP: `stomp/` failed before the task started because the independent perception
snapshot was stale. `stomp_v2/` exercised the real OMPL -> STOMP chain; it failed
the task, including empty grasp/recovery and sampled acceleration bounds. The
validator blocked values such as 10.01157 rad/s² against the configured operating
limit 10.0 (and 7.50977 against 7.5). Those limits were NOT loosened. The original
planner remains default; smoothing is an explicitly unaccepted experiment.

TOPPRA: wheel 0.6.11 downloaded and installed ONLY in
`/tmp/parcel-path-planning-python`, not system/site packages or runtime requirements.
`tools/toppra_retime.py` reconstructs the exact saved q/v/a quintic polynomial and
uses `ParametrizeConstAccel` to avoid fitting a different path. It reports solver
status and densely sampled residuals, including both sides of timing switches.
It does not export a controller command or authorize execution.

- Baseline run 1, grid .01: original motion sum 13.38873 s; TOPPRA 13.91377 s;
  all 16 nonstationary cases failed sampled residuals (max acceleration excess
  0.10423 rad/s²). One stationary hold is explicitly skipped.
- Baseline run 2, finer grid .002: original 13.39018 s; TOPPRA 13.43434 s;
  all 16 still failed residuals (max acceleration excess 0.005711 rad/s²).
- Neither faster nor sufficiently feasible: runtime timing integration gate FAILED.
  No changed runtime timer or extra runtime dependency is introduced.

Arrival shadow: pure predicate/replay plus optional live hook implemented. Negative
tests cover stale/future/duplicate state, missing measured velocity, wrong result ID,
clock reset, gaps, timeout and a single-joint outlier. The first two continuous runs
(`continuous_shadow/`) passed all independent motion/grasp/transport/health checks,
but exposed stale shadow state/clock from the shared gripper node. The corrected
hook uses its own depth-one state subscription and executor (`continuous_shadow_v2/`).
The shadow NEVER replaces `_settle_pose`.

Corrected shadow results: `continuous_shadow_v2/` passed TWO fresh headless confirmed
continuous batches (6/6 placements, exit 0, no unexpected health errors), including
independent transport, perception and grasp-retention verdicts. Shadow outcomes were
run 1: 12 DWELLING / 1 ARRIVED; run 2: 13 DWELLING, with no stale/error verdicts.
The trial profile uses 0.01 rad position tolerance, 0.05 rad/s velocity tolerance,
0.15 sim-s maximum age and 0.12 sim-s dwell. These are simulation shadow proposals,
not manufacturer tolerances. Existing EE settle uses wall-time dwell; the difference
does NOT by itself prove the existing arrival unsafe. Gate activation remains unapproved.

Final review intersected URDF soft limits and configured overrides with hard bounds.
`final_profile_replay.json` rechecks all 26 recorded free-space trajectories from the
two corrected runs under that stricter profile: both PASS. This is a static limit
replay, not a repeat of live collision checking. Original live collision verdicts
remain in JSONL. Final boolean/profile-shape guards were covered by offline tests.

Pilz: `pilz_sequence.py` provides a planning-only service adapter with service-type
discovery, explicit scene synchronization/revision matching, first-item-only start
state, group/radius/contact-stop checks and nonoverlapping blend spheres. The probe
`tools/probe_pilz_sequence.py` starts a separate namespaced server with execution
actions disabled and verifies `allow_trajectory_execution=False` by querying the
running server. It never calls a controller. Its start is a declared ready-pose
fixture, not a measured execution start; there is no loaded-motion acceptance yet.

Two initial fresh servers produced valid unloaded zero-blend and 5 mm blend plans.
For the small fixture, planned interior zero-speed samples drop 1 -> 0, while
duration grows .565685 -> .582843 s. Both server teardowns crashed. The initial
prototype JSON (`pilz_probe.json`) judged planning only and must NOT be treated as
a complete pass. The strict rerun (`pilz_strict/probe.json`) verifies execution is
disabled and explicitly fails `SERVER_HEALTH_FAILED` on the shutdown SIGSEGV
(exit -11, in `rclcpp::Executor` destruction). No exception for old legacy teardown
245 is applied to this new crash. The earlier integer ROS message conversion abort
was corrected and is recorded in `pilz_initial_abort.json`.

Result: planning capability demonstrated; runtime Pilz integration remains BLOCKED
by server health and then requires measured-start/loaded execution validation.
`TWIN_BLEND=pilz` is not exposed by the working task. No second arm execution
owner or persistent planning server is added to normal bringup.

Final default-path regression: `colour_regression/` passed two fresh colour sorts,
6/6 independent placements, zero task/planning/traceback/fatal health counts. Both
sorters retain the known post-completion MoveIt teardown exit 245, accepted only
after completed work and passing ground truth. This task does not fix that legacy
destructor issue or the continuous process's existing semaphore warning.

Final offline suite: ALL 11 checks PASS. Build of `twin_action_demo` and
`twin_moveit_scripts` passed; overlay prefixes were explicitly checked in this
checkout. `process_audit.json` checked 14 run sessions and three recorded Pilz
owners, plus any remaining Pilz owner marker: no task-owned processes remained.
No new GUI runs, hardware tests, continuous-collision certificates, torque/contact
model validation or primary-repo promotion are claimed.

Recorder scope: shared free-space MoveIt motions, including the final merged TOTG
output. The custom moving-belt following/intercept trajectory retains its existing
Hermite/IK/collision and sensor checks. Planned path lengths and interior zero-speed
samples are NOT measurements of robot tracking or actual full stops. Binary sampled
collision checks do not measure a minimum-clearance certificate. The old initial
`baseline_host/` failure remains recorded: a legitimate already-at-ready single-point
hold was rejected until measured stationary-hold handling was implemented.

## Sources and reproducible offline tools

TOTG can deviate from its input path, so final collision checks matter:
[MoveIt timing documentation](https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html).
Installed-release planner chaining was checked against the
[MoveIt 2.12.4 implementation](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp).
The offline timing adapter follows the [TOPPRA API](https://hungpham2511.github.io/toppra/python_api.html)
and the downloaded wheel's parametrizer implementation. Local dependency/model inventory:
`python3 tools/motion_inventory.py --output <fresh-output.json>`.

```bash
python3 tools/motion_metrics.py <run>/motion.jsonl --output <run>/motion_metrics.json
python3 tools/trajectory_validator.py <run>/motion.jsonl --output <run>/sampled_replay.json
# Optional TOPPRA install is isolated; this command uses the task's temporary wheel:
PYTHONPATH=/tmp/parcel-path-planning-python MPLCONFIGDIR=/tmp/parcel-path-planning-mpl \
  python3 tools/toppra_retime.py <run>/motion.jsonl --grid-step .002 --output <report.json>
```

Old baseline JSONL files predate recording the measured stationary start; their
single-point hold cannot retrospectively pass stationary-state replay without that
evidence. The runtime validation verdict remains recorded as originally observed.
Later recordings include `measured_start`. No old evidence is relabeled.
