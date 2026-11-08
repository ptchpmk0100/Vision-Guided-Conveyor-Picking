# V2 SORTING CELL — RESUME LEDGER

**Purpose:** the single authoritative "where are we / what's next" file. If a session ends
(usage reset, crash, new chat), **read this first**, then `~/.claude` project memory.
Trust this file and the repo over any recollection. Updated after every verified sub-step.

**Last updated:** 2026-11-07 — **BELT-HEIGHT SORTING-LINE REDESIGN COMPLETE + verified**
(Khoi's scene rework: colour boxes on a belt-height deck → colour-labelled trays at belt
height). Stages 0/A/B/C/D/E all pass by Gazebo ground truth. **Next = Phase B, re-ordered to
B1 hand-eye calibration first** (B2 shape-grasp deprioritised — uniform boxes removed the
roll-off). Full decision trail in `V2_DECISIONS.md`. Autonomous run to C3 (rule 10).

**SORTING-LINE REDESIGN (2026-11-07, mirror, all ground-truth verified, additive):**
- New: `worlds/industrial_sorting_line.sdf`, `launch/industrial_sorting_line_bringup.launch.py`
  (`detector:=`/`headless:=`), models `obj_{red,green,yellow}_box.sdf` (40×40×50 mm),
  `object_detector_sorting_line.py`, `sorting_line_scene.py`, `sort_task_sorting_line.py`,
  one backward-compat param on `move_to.pick()` (grasp gate). Tools: `check_sorting_line.py`
  (+`--placed`), `check_detections_sorting_line.py`, `run_sorting_line_check.sh`,
  `run_sort_line_check.sh`, `reach_probe_sorting_line.py`, `run_reach_probe_sorting_line.sh`.
- Verified: reach GO at belt height (42/42 legs); boxes settle z=0.320 (3/3 ×2); detector
  3/3 class-correct 0 phantoms xy≤4.5 mm (×2); **full sort 3/3 correct tray ×2, pick 6/6
  first-attempt, 0 errors**; old ground A4 pipeline still 3/3 (no regression).
- Run it: `ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py detector:=true`
  then `ros2 run twin_moveit_scripts sort_task_sorting_line`; verdict
  `python3 tools/check_sorting_line.py --placed`. Old `industrial_cell_a2`+`sort_task` path kept.
- Geometry: deck top z=0.295, box centre 0.320; trays (tray_red 0.45,+0.18 · tray_green
  0.45,−0.18 · tray_yellow 0.37,0) floor-top 0.295, walls to 0.335, coloured FLOOR panel label.
  pick/place heights are all RELATIVE to object z, so belt height needed only the grasp-gate
  retarget (half-width 0.020) once Stage 0 proved reach. Detector: deprojection is depth-based
  so only the z-gate moved (EXPECTED_CENTER_Z 0.320, OBJ_HALF_HEIGHT 0.025, ROI to tray span,
  blue removed). Collision: tray WALLS added; deck+table omitted (A3 ground-plane reasoning).

---

## 0. Non-negotiable rules (carry into every session)

1. **`~/ros_test_ws` is READ-ONLY.** All work happens in `~/ros_test_ws_mirror`. Only modify
   the live repo if Khoi *specifically* asks — and then restate the plan and wait for an
   explicit confirmation first. Keep `.pre_<step>_bak` backups of every live file changed.
2. **Judge by Gazebo ground truth, never by log lines.** Every status message in this project
   has lied at least once (`HELD` on an empty hand, `place=True` on an unmoved cube).
3. **Ignore `rc=245`** — a harmless MoveItCpp teardown segfault that fires *after* `Done.`.
4. **Never add lighting to a world the detector sees** (see landmine below).
5. **Additive, not destructive:** new components ship alongside the working ones and are
   switched in only after ground-truth verification, so an interruption never leaves a
   broken pipeline.
6. Build in a clean env: `env -i … bash --norc --noprofile`, sourcing only
   `/opt/ros/jazzy/setup.bash` then the **mirror** install. `~/.bashrc` sources the *live*
   overlay, which would silently shadow mirror packages.
7. **Three-legged validation of every part** (Khoi, 2026-11-07). All three, every sub-step:
   - **(a) Ground truth** — did the world actually change as intended? This is the arbiter
     of *success*. (Rule 2.)
   - **(b) Log validation** — scan the run logs for errors/exceptions/tracebacks/controller
     failures. Logs must never decide success, but they ARE the right source for detecting
     *system health* problems. Complementary to rule 2, not in conflict with it.
   - **(c) Consecutive-run validation** — run the same thing back-to-back N times (N>=2,
     prefer 3) and require consistent behaviour with no major system errors. A single pass
     cannot distinguish "works" from "got lucky": the first reach probe called the verified
     spawn point unreachable on a one-shot run.
8. **Stage reviews.** After each big stage (A5, B5, C3): review the code, review the plan,
   and revise — hunt error-handling gaps, dead assumptions, and anything the plan got wrong.
   Record the revision in this ledger.
9. **Usage escalation ladder** (Khoi, 2026-11-07): prefer **Claude**; if Claude usage runs
   out, hand the mechanical remainder to **Codex** (`codex_preflight.sh --dir
   ~/ros_test_ws_mirror --skip-smoke` first, then drive from the closed specs in §4);
   if **both** are exhausted, **STOP and wait for reset** rather than degrade quality.
   Reserved actions (touching `~/ros_test_ws`, pushing, releasing) always wait for Khoi.
10. **AUTONOMOUS RUN (Khoi, 2026-11-07): run unattended to Phase C3 — DO NOT STOP, DO NOT
    ASK** (changes stay in `~/ros_test_ws_mirror` only; never jeopardise the system; the
    `~/ros_test_ws` read-only rule stands — that is the ONLY thing needing confirmation).
    - Decisions are made autonomously: list options in `V2_DECISIONS.md`, reason + challenge
      them through the lens **"most PRACTICAL, INDUSTRIAL, REAL-WORLD impact (feasible +
      verifiable)"**, drive until ONE stands out, record the verdict, proceed. No AskUser.
    - Operating mode (D0): **Codex = main executor/planner/coder; Claude = manager** (frames,
      challenges, runs the Gazebo sims, verifies by ground truth, decides+logs, reserved
      actions). Ladder: Claude out → Codex solo; both out → wait; reset → resume.
    - Every big stage (A5 done; B5; C3): review code, review errors, revise the plan for a
      more effective/industrial approach; record it here + in `V2_DECISIONS.md`.
    - On full completion + verification, write the final `V2_PROGRESS.md` summary.
    - Continuity: each background task completion re-invokes the manager, which reviews and
      dispatches the next — the chain self-sustains; always queue the next action.

---

## 1. Verified state (DONE)

| Step | Artifact | Ground-truth result |
|------|----------|---------------------|
| Phase 0 | S22 repeat-loop on the **live** install | **4/4** — re-detect at prev drop, land within ~5 mm, park at `ready` |
| A1a | `worlds/sorting_cell.sdf` + `launch/sorting_cell_bringup.launch.py` | **2/2** — cube placed inside bins |
| A1b | `worlds/industrial_cell.sdf` + `launch/industrial_cell_bringup.launch.py` | **2/2** — detect 0.501 exact, grasp 0.0151 exact |
| fix | `headless:=true` launch arg (was silently a no-op) | verified: `-s --headless-rendering` present, no GUI proc |
| A2a | layout locked by `tools/reach_probe.py` | bins (0.45,±0.18)+(0.37,0) and staging row x=0.56, y∈[−0.10,+0.10] all **2/2**; back row (0.37,±0.20) **0/2 BLOCKED** |
| A2b | 5 object models + `launch/industrial_cell_a2_bringup.launch.py` (3 objects @ 0.10 m spacing, no blue cube) | spawns and settles at z=0.015 |
| A2c | `object_detector_multi.py` (HSV + connected components → `vision_msgs/Detection3DArray`, + legacy PoseStamped) | installed **alongside** the old node |
| A2d | `tools/check_detections.py` + `tools/run_a2_detector_check.sh` | **3/3 detected, 3/3 class-correct, 0 phantoms, ×3 consecutive**; err 1.0/1.2/2.9 mm; logs 0 errors |
| A3 | `cell_scene.py` bins as HOLLOW MoveIt collision objects (opt-in `TWIN_CELL_COLLISION=1`) | validated via A4 (collision-aware place plans + executes) |
| A4 | `sort_task.py` + `tools/check_sort.py` + `tools/run_a4_sort_check.sh` | **3/3 correct bin** (but partly luck — see A5 review) |
| A5 | `trial_db.py` + `tools/benchmark.py` + `tools/run_benchmark.sh` → `logs/v2/benchmark.db` | **evidence layer verified**: S1 2/2 attempted (pick 100%), S2 cubes 4/4 100% / cylinders 0/2; 0 logged-vs-truth divergence; 0 unexpected errors ×4 runs |

**A2 numbers worth keeping:** shape discriminator separates exactly as predicted from
geometry — cube `fill≈0.99`, cylinder `fill≈0.79` (square 1.0 vs circle π/4≈0.785).
Object spacing must be ≥ ~0.07 m: the hand opens to ±0.04 m from the grasp centre, so at
0.05 m spacing a neighbour's near edge (0.035 m) sits *inside* the finger path.
Elongated blocks are deferred — a block's grasped dimension depends on which world axis
the fixed top-down gripper closes along; cubes/cylinders are 3 cm in every horizontal
direction and so are grasp-axis-agnostic.

Functional core (unchanged, verified): cube picked from the ground at centre **z = 0.015**,
auto-chosen drop slot via `choose_place_target` ring-tour, arm parks at `ready` each cycle,
repeat cycles need **no respawn**.

---

## 2. LANDMINES (hard-won — do not rediscover)

- **Lighting.** The detector's blue mask has only **5 units of headroom**: S21 measured the
  Panda hand's blue indicator dots at `B-G <= 31` vs the threshold `B-G >= 36`. Adding one
  overhead fill light pushed arm pixels into the mask → detected pose drifted
  **0.501 → 0.472 (29 mm)** → the hand closed on the cube's **edge** at width 0.0200
  (squeaking past the `0.015 ± 0.006` gate) → reported `pick=True, place=True` on a cube that
  **never moved**. Sun-only, exact verified params. A dark floor is fine; added lights are not.
- **No blue in camera view.** Any blue-dominant surface is read as the cube. The reference
  render's light-blue bin became teal. (A2's HSV detector removes this constraint.)
- **Geometry invariant.** Everything is calibrated for objects resting at centre **z = 0.015**
  on the ground plane. Raised trays/conveyors/pedestals require re-calibrating the detector
  z-gate (`EXPECTED_CENTER_Z`/`CENTER_Z_TOL`) and `FINGER_REACH_Z`, plus a reach re-check.
  All industrial dressing is `<visual>`-only (no collision) for exactly this reason.
- **Orphan processes.** `setsid` + process-group kill misses `gz sim gui`/`server` children.
  Run headless. If orphans remain, kill by explicit PID; **never** `pkill -f` a pattern that
  also matches your own command line (it kills the calling shell, exit 144).
- **Reach is NOT a rectangle — it narrows as |y| grows.** Measured by `tools/reach_probe.py`
  (2 passes each, gripper open, real seeded-IK top-down grasp posture):
  | pose | grasp | verdict |
  |---|---|---|
  | (0.45, ±0.18) | 2/2 | REACHABLE (existing bins a/b) |
  | (0.37, 0.00) | 2/2 | REACHABLE (existing bin c) |
  | (0.50, 0.00), (0.52, ±0.15), (0.58, 0.00) | 2/2 | REACHABLE |
  | **(0.37, ±0.20)** | **0/2** | **GRASP-Z BLOCKED** — IK solves, but the arm stalls
    **28–30 mm above grasp height** on every pass and every retry |
  So a back-row bin layout at y = ±0.20 is **not viable**; the three existing bins are the
  verified set. The staging row goes out FRONT (x ≈ 0.56, |y| ≤ 0.10) — clear of the bins'
  x-extent (≤0.52) and of the side bins' y (≥0.11).
- **One-shot probes lie.** Round 1 run once per pose called the *verified spawn point*
  (0.50, 0) unreachable. Always repeat (PASSES ≥ 2) and keep known-good controls in the
  candidate list, or flakiness reads as a reachability limit.

---

## 3. How to verify anything (durable harness)

```bash
# ground-truth run: <launch-file> [cycles] [tag]
~/ros_test_ws_mirror/tools/run_groundtruth.sh \
  ~/ros_test_ws_mirror/tools/verify_industrial_headless.launch.py 2 ind
# logs -> ~/ros_test_ws_mirror/logs/v2/
```
Run it from a clean env:
```bash
env -i HOME="$HOME" TERM=xterm PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --norc --noprofile ~/ros_test_ws_mirror/tools/run_groundtruth.sh <launch> <cycles> <tag>
```
Pass criteria: `detected` ≈ `pre` (camera found it where it really is), `post` ≈ the chosen
slot/bin within a few cm, `parked=1`. Rebuild after any edit:
```bash
colcon build --symlink-install --packages-select twin_action_demo twin_moveit_scripts
```

To look at the cell yourself (GUI):
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros_test_ws_mirror/install/setup.bash && export GZ_VERSION=harmonic
ros2 launch twin_action_demo industrial_cell_bringup.launch.py        # headless:=true for no GUI
```

---

## 4. PHASE A — COMPLETE (A1–A5). Next: Phase B (B1 calibration, B2 shape-aware grasp…).

### A5 — evidence layer — **DONE + VALIDATED 3 WAYS**
`trial_db.py` (SQLite: `runs`/`trials`/`truth`), `tools/benchmark.py` (ground-truth
recorder + metrics report), `tools/run_benchmark.sh` (scenarios S1 single-sort + S2
colour-sort, each ×2 consecutive). DB at `logs/v2/benchmark.db`. The sort writes trials
only when `TWIN_DB` is set, so A4's verified path is byte-for-byte unchanged without it.
Measured (2 runs/scenario):
- **S1** (sort 1 object): **2/2 attempted correct, pick 100%**, ~96 s/run.
- **S2** (sort all 3): **cubes 4/4 = 100%, cylinders 0/2 = 0%** → 4/6 overall; pick
  first-attempt cube 4/5, cylinder 0/2; ~248 s/run; **0 logged-vs-truth divergence**
  (belief matched Gazebo — the grasp-width gate honestly refused the bad cylinder grasp).
- **Log health (refined scan):** 0 unexpected system errors / 0 tracebacks / 0 fatals on
  all 4 runs; only S2-run1 had 2 planning failures, both downstream of the cylinder (below).
Re-print metrics anytime: `python3 tools/benchmark.py report` (reads the DB, no sim).

### A-STAGE REVIEW (Khoi's standing rule 8 — code + plan, revise)
1. **REVISED the cylinder finding (was under-stated in the A4 note).** It is not merely
   "needs ~2 attempts". A failed off-centre grasp can **fling the cylinder clean off the
   world** — ground truth caught it at **(−12.9, −1.99)** in S2-run2 — or roll it out of
   the detectable ROI, so it vanishes from `/detected_objects/array` and the deliberate
   2nd attempt has no target. **A4's 3/3 was luck:** the old leaky key gave *unbounded*
   retries (bounded only by LOOP_GUARD=12) and the cylinder happened to settle somewhere
   graspable. The honest, deterministic rate is **cylinders 0/2**. ⇒ **B2 is now a hard
   blocker, not a nice-to-have:** it must stop the roll/fling at the source (antipodal /
   shape-aware grasp), NOT add retries and NOT widen `GRASP_WIDTH_TOL`.
2. **Wart FIXED — leaky attempt key.** `sort_task` now tracks objects by nearest-previous
   pose (`_ObjectTracker`, `MATCH_RADIUS=0.08`), not `(cls, round x, round y)`. Retries are
   now a deliberate, counted policy (`MAX_ATTEMPTS_PER_OBJECT=2`). The benchmark proves the
   fix two ways: a cube that missed its 1st grasp recovered on a *counted* 2nd attempt
   (cube first-attempt 4/5 → final 4/4), and the cylinder's retries are now bounded instead
   of spinning on a shifting key. **Caveat for S3:** colour-based identity matching assumes
   one object per colour; multi-same-colour scenes need detector-assigned IDs.
3. **Wart FIXED — log-health over-count.** The scan excluded the task's own honest
   `pick/place failed` lines but still counted MoveIt/OMPL planner chatter (~8 lines per
   logical plan failure) and the benign octomap startup notice → S2-run1 read "17 errors".
   Refined `log_health` in `run_benchmark.sh` excludes planner-internal lines (counted once
   under "planning failures") and the octomap notice → **17 → 0 unexpected**, 2 real
   planning failures surfaced separately.
4. **Validation upgrade (new, keep):** the benchmark now records BOTH the task's belief and
   Gazebo ground truth per run, so "a status line lied" is a measured number
   (`logged-vs-truth divergence`). A4 under-reported (−); A5 reads 0 — the clearance fix +
   honest cylinder refusal closed the gap.

### Historical decision (now settled): bins **by colour** (red→a, green→b, yellow→c);
shape is detected + logged but does not route. Shape routing is the S3 benchmark (needs B2).

### A2 — multi-object perception — **DONE** (see §1). Thresholds set from MEASURED data:
`CENTER_Z_TOL` 0.04→**0.010** (real objects measure z=0.015 dead on; the flat red floor
marker computes to z=−0.003) and `ROI_Y` ±**0.26** (the marker's *visible* centroid is
y=0.272, not its model pose 0.30 — only 119 px of ~400 is in frame). Lesson: set gates
from what the camera actually reports, not from the SDF pose.

### A2 (original plan, kept for reference)
Constraint: **all objects share the same 3 cm grasp dimension**, varying only colour and
shape, so the hardcoded grasp gate (`0.015 ± 0.006`) and `CUBE_HALF_HEIGHT` stay valid.
Shapes: cube, cylinder, rectangular block (distinguishable top-down). **No spheres** (roll),
**no blue** (arm indicator dots).

- **A2a** Re-layout: bins to a back row, staging tray in a front strip. **Verify reach** to
  every new bin position + tray extent by ground truth *before* anything else.
- **A2b** Object set: 5–6 objects, same grasp dimension, mixed colour/shape, on a
  ground-level tray (flush — raised breaks the z invariant).
- **A2c** New node `object_detector_multi.py` **alongside** the existing one (do not delete
  it): HSV + connected components → publish a **list** of objects (id, colour, shape, pose).
  Back it with a new launch; keep the old path runnable.
- **A2d** Measure per-object/per-class classification accuracy and pose error vs ground truth.

### A4 — sort task — **DONE: 3/3 by ground truth on 3 consecutive runs**
Final state: `0/3 before -> 3/3 after`, **0 planning failures, 0 gripper retries, 0
arrival misses, 0 tracebacks** on every run. Objects rest inside their bins at z=0.015.

**DEFECT FOUND + FIXED — open fingers collide with the bin walls on retract.** MoveIt:
`Found a contact between 'bin_a' and 'panda_rightfinger'` / `Computed path is not valid.
Invalid states at index locations: [1] out of 84`. Reaching to the BIN FLOOR to release
puts the opened fingers (0.08 m span, ~0.10 padded) inside a 0.132 m interior, so the
retract is unplannable from its first states — and `place()` returned `retract_failed`
for objects it had actually delivered (the task UNDER-reported: 1/4 and 2/4 while ground
truth said 3/3). Cure: `place(..., clearance=)` raises the release height;
`sort_task.BIN_RELEASE_CLEARANCE = 0.035` puts the fingertips ~8 mm above the rim and the
object drops the last few cm in — standard bin-placing practice. Rejected the obvious
alternative (widening the bins) because at 0.18 m they touch each other and crowd the
x=0.56 staging row, invalidating the reach verification. No bounce-out observed.

**KNOWN LIMITATION — cylinders need ~2 attempts (deterministic, 3/3 runs).**
First grasp: detect stroke stalls at **0.0200** (not 0.0151), hold settles at **0.0275**,
and the size-plausibility gate correctly refuses it; the cylinder is found 14 mm away on
the retry (`(0.559, 0.001)` -> `(0.559, -0.013)`), i.e. **it rolls on first contact** —
one finger catches it off-centre while the other keeps closing, so the mean width reads
too wide. The gate is doing its job; the defect is upstream. **Do NOT widen
`GRASP_WIDTH_TOL`** — that gate caught a fake grasp in S21. Proper fix belongs to **B2
(shape-aware grasp / antipodal sampling + recovery)**. The sort self-recovers today
because of the leaky attempt key (below), which is luck, not design.
`twin_moveit_scripts/sort_task.py` (entry point `sort_task`) + `tools/check_sort.py` +
`tools/run_a4_sort_check.sh`. Reads `/detected_objects/array`, routes **red→bin_a,
green→bin_b, yellow→bin_c**, picks each at its detected pose, places it in its bin, parks
at `ready`. Objects already inside a bin footprint are skipped, so the flow is strictly
one-way. First clean run: all three objects ended in the correct bin by Gazebo pose, with
0 planning failures / 0 tracebacks / 0 arrival misses. **This also validates A3** — placing
into a collision-modelled bin plans and executes fine.

**DEFECT FOUND + FIXED — the first finger command in a process is swallowed.**
Measured sequence: `target=0.0400 first=0.0199 final=0.02` three times in a row (retries
1.5 s apart all failed), then the NEXT command later in the process worked first time.
So it is first-command initialisation, **not** a timing window a retry can win — two
rounds of escalating retries were the wrong fix. Cure: `setup()` now sends one throwaway
prime stroke, so the first REAL open is the second command (`setup: gripper prime stroke`
in the log). Cost ~1.5 s. The verify+retry in `move_gripper` stays as a safety net but is
no longer load-bearing. Impact when it bit: a half-open hand (0.04 total) fouls the
descend onto a 3 cm object — arm stalls ~29 mm high, object shoved (a cylinder was knocked
0.3 m), pick fails at `descend_failed`/`grasp_width_implausible`.

**Known wart (latent, not yet fixed):** `sort_task` keys attempted objects on
`(class, round(x,2), round(y,2))`. If a failed grasp NUDGES an object, its key changes and
it becomes "new", so it gets retried. That accidentally rescued a cylinder in one run, but
it means `MAX_ATTEMPTS_PER_OBJECT = 1` is leaky; only `LOOP_GUARD` (12) bounds it. Fix by
keying on identity/nearest-previous rather than rounded position.

**Cylinders are marginal to grasp:** one attempt failed `grasp_width_implausible` before
succeeding. Do NOT "fix" this by widening `GRASP_WIDTH_TOL` — that gate is what caught a
fake grasp in S21; loosening a safety net to improve a score is the wrong trade.

### A3 — bins as MoveIt collision objects — **IMPLEMENTED + VALIDATED via A4**
`twin_moveit_scripts/cell_scene.py` adds the 3 bins as HOLLOW collision objects (four
thin wall boxes each — a solid box would make every `place()` unplannable). The ground
plane is deliberately NOT added: the grasp pose puts fingertips ~3 mm off the floor, so
a floor object would make every pick unplannable. Opt-in via `TWIN_CELL_COLLISION=1`.

**KEY FINDING — A3 cannot be validated with the A1 ring-tour test.** With bins as
collision objects, **descending INTO a bin to pick an object out of it is unplannable**
(`plan_failed: planner returned no trajectory`, OMPL "Unable to solve"). That is not a
bug: the A4 sort workflow moves objects **tray → bin, one way** and never picks from
inside a bin. The ring-tour test does, so it is the wrong scenario. **Validate A3
together with A4**, using pick-from-staging-row → place-in-bin.

**DEFECT FOUND + FIXED (pre-existing, not an A3 regression — it reproduces with
collision OFF):** the first gripper command after bringup is silently dropped. The
fingers stay at their 0.02 spawn position while `move_gripper` reports
`ok [reached] width=0.0200` against a 0.0400 command — the third "lying status" in this
codebase. A half-open hand (0.04 total) then fouls the descend onto a 3 cm object: the
arm stalls ~29 mm high, the object is nudged, pick fails at `descend_failed`. S21 added
retry for ARM execution but gripper commands had no arrival verification at all. Fixed
in `move_gripper`: an OPENING stroke that settles > `GRIPPER_OPEN_TOL` (0.005) short is
re-sent, up to `GRIPPER_OPEN_ATTEMPTS` (3), waiting `GRIPPER_OPEN_RETRY_WAIT` (1.5 s)
and pumping the executor between tries; it then fails honestly instead of handing a
half-open hand to the descend. CLOSING strokes are exempt — stopping short IS the grasp
signal.

### A3 (original plan, kept for reference)
Pull *before* the sort task: the arm will repeatedly reach into bins that already hold
objects. Today the 3 cm walls are cleared only because planning is collision-unaware.

### A4 — sort task
class → bin mapping, sort-until-clear state machine, failure recovery. Ground truth: every
object ends in its correct bin. This is where the colour-vs-shape decision lands.

---

## 5. If the session dies mid-run

1. Read this file, then project memory (`~/.claude/projects/-home-pmkhoi-ros-test-ws/memory/`).
2. `ls ~/ros_test_ws_mirror/logs/v2/` — the last ground-truth run tells you what actually worked.
3. Because changes are **additive**, the last verified launch still runs. Re-verify it with
   the harness in §3 before building further.
4. Resume at the first unchecked sub-step in §4.
5. If Claude usage is the constraint, §4's sub-steps are closed specs Codex can execute
   (run `codex_preflight.sh --dir ~/ros_test_ws_mirror --skip-smoke` first). Reserved actions
   — anything touching `~/ros_test_ws`, pushing, or releasing — **wait for Khoi**.
