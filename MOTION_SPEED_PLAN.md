# Panda parcel-cell motion speed & smoothness plan

Goal: make the arm move at near real-life speed with natural, smooth motion and
the minimum number of deliberate stops — move to object, brief stop, grasp; move
directly to tray, brief stop, place; move straight to the next target; return to
home only after all parcels are placed. The planner should route directly from
the current pose to the next target (object↔tray) rather than detouring home.

Scope: the parcel pick-and-place path only
(`twin_moveit_scripts/parcel_task.py` + shared `move_to.py` + MoveIt config).
The original colour/ground sorting pipeline and shared grasp/place *semantics*
must keep passing. Validate every phase against actual Gazebo poses, not status
flags (AGENTS.md).

## Diagnosis (confirmed in code)

| # | Cause | Location | Effect |
| --- | --- | --- | --- |
| 1 | Velocity/accel scaling hard-set to 0.1 (10%) in both the MoveIt default and every request | `config/joint_limits.yaml` (`default_*_scaling_factor: 0.1`); `move_to.py:220,234` | Arm runs at 1/10 of real joint limits — primary "slow" factor |
| 2 | Fixed `time.sleep(3.0)` after every arm segment | `move_to.py:708` (`_plan_execute`) | ~30 s/parcel of pure dead time; `robot.execute()` already blocks until done |
| 3 | Each waypoint is an independent plan→execute starting/ending at zero velocity; no blending | `_plan_execute` called per leg | "Too many stopping points" |
| 4 | Redundant homing to `ready`: `pick()` re-homes after the task already did; `place()` homes **while holding the object** (object→home→tray detour); task homes again post-place to clear the camera | `move_to.py:1087`, `move_to.py:1284`, `parcel_task.py:181,230` | Extra stops **and** non-optimal path (opposite of direct object→tray) |
| 5 | Short vertical legs planned as random free-space RRTConnect joint paths | `move_to_grasp_pose`/`move_to_pose` → OMPL | Unnatural wiggle; no straight descend/lift |

Per-parcel today: ~10 arm segments + 3 gripper moves. Fixed sleeps alone =
10 × 3 s = ~30 s/parcel before any motion (at 10%) or perception dwell
(`stationary_target`, `confirm_slot`, up to ~8 s each). Gripper: ~2.5 s traj +
1.2–6 s settle each.

Caveat: Gazebo real-time factor ≈ 0.45 (handoff) bounds wall-clock speed
regardless; the causes above dominate the stoppy/unnatural feel and are what we
control.

## Target cycle (desired)

Per parcel: `move→(stop)grasp→move→(stop)place→move to next`. Two deliberate
stops per parcel. One `ready` home at the very end. Transit legs blended/smooth.

## Phase 1 — Speed + remove dead time (lowest risk, biggest immediate win)

- Raise `max_velocity_scaling_factor` / `max_acceleration_scaling_factor` from
  0.1 toward ~0.5–0.7 (accel ≤ velocity for smoothness). Set in the params
  object (`move_to.py`) so all callers inherit; leave `joint_limits.yaml`
  defaults as a conservative floor or raise in step. Make it tunable via one
  constant/env so re-tuning doesn't touch call sites.
- Replace `time.sleep(3.0)` in `_plan_execute` with a short settle: poll the EE
  pose until it stops moving (Δ < tol) or a small cap (~0.3–0.5 s). Execution
  already blocks, so arrival verification (`miss <= ARRIVAL_TOL`) is unaffected.
- Keep `EXEC_ATTEMPTS`, `ARRIVAL_TOL` semantics unchanged.

Acceptance: indexed ×2 + colour regression still 3/3 correct placement with
perception error within prior bounds; no new collisions/overshoot; per-parcel
sim-time (READY→SETTLED) materially reduced; no grasp/placement regression.

## Phase 2 — Direct paths, remove home detours (fixes optimal routing + stops)

- `place()`: drop the `move_to_named('ready')` at `:1284`; plan directly from the
  lift pose to the place-approach pose (object→tray). Keep a fallback: if the
  direct plan fails, home-then-go (old behaviour) so reliability is preserved.
- `pick()`: gate the `ready` home (`:1087`) behind a `skip_home`/`assume_ready`
  flag; the task already homes before the first pick and the end-of-cycle retract
  leaves a well-conditioned pose.
- Post-place verification: replace the full `move_to_named('ready')` at
  `parcel_task.py:230` with a minimal retract that clears the overhead camera of
  the tote, and confirm the slot from there. Full `ready` only once at cycle end
  (`parcel_task.py:234`).
- Preserve the S17 rationale (well-conditioned basin) via the seeded-IK grasp
  targets + fallback, not by always homing.

Acceptance: as Phase 1, plus verified that the arm no longer transits through the
home pose between pick and place (inspect planned/played paths and event
timing), and occupied-slot collision protection still holds.

## Phase 3 — Smooth, blended, natural motion

- Use Cartesian straight-line motion for the pure vertical legs (descend, lift,
  retract) instead of OMPL free-space, for predictable straight travel.
- Blend transit segments so the arm does not fully stop between approach and
  transport. The Pilz planner is already configured
  (`pilz_cartesian_limits.yaml`); use a `MotionSequenceRequest` with
  `blend_radius`, or MoveItCpp trajectory concatenation + TOTG over the merged
  path. Grasp poses are already seeded joint targets → Pilz PTP is a natural,
  deterministic, smooth fit. Keep deliberate stops only at grasp and release.
- If Pilz sequencing via `moveit_py` proves limited, fall back to concatenating
  per-leg trajectories and re-time-parameterizing with TOTG to remove the
  intermediate zero-velocity dwell.

Acceptance: as Phase 2, plus visibly smooth/continuous transit (no full stop
between approach and transport legs) in captured frames/video, with placement
accuracy and grasp success unchanged.

## Phase 4 (optional) — trim gripper/perception dwell

Only if still slow: modestly reduce `GRIPPER_SETTLE_MIN`/move time and the
perception stability windows, carefully — these guard grasp/placement
correctness. Not required to hit the goal; do last with its own validation.

## Validation (every phase)

Clean env, isolated domain/partition, judge by poses:

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin bash --noprofile --norc \
  tools/run_parcel_cell_check.sh --mode indexed --runs 2 \
  --logdir /tmp/motion-phaseN-indexed
env -i PATH=/usr/bin:/bin:/usr/local/bin bash --noprofile --norc \
  tools/run_parcel_cell_check.sh --mode colour --runs 2 \
  --logdir /tmp/motion-phaseN-colour
bash tools/run_offline_tests.sh
```

Cycle time is read from the `PARCEL_STATE` events' `sim_time` (READY→SETTLED per
slot, READY→FULL overall) in each run's `task.log`/result JSON — no new
instrumentation needed. Record before/after per phase.

Rollback: each phase is an isolated commit; revert if a gate regresses.

## Results log (measured, indexed mode, 3 parcels)

Cycle time = `PARCEL_STATE` event `sim_time`, READY→FULL. Placement from
`placed.json` (upright/stable/footprint vs target).

| Stage | Full cycle (sim-s) | Speedup vs base | Per-slot PICK→PLACE / PLACE→SETTLED | Placement |
| --- | --- | --- | --- | --- |
| Baseline (0.1× scaling, 3 s sleeps) | 182.4 | 1.00× | ~20.3 / ~27.1 | PASS (~1 mm) |
| Phase 1 (0.5× scaling + condition settle) | 97.6 | 1.87× (−85 s) | ~11.9 / ~9.0 | PASS (~1 mm) |
| Phase 2 (direct routing, no home detours) | 93.6 | 1.95× (−89 s) | ~11.6 / ~8.0 | PASS (~1 mm) |
| Phase 3 (blended approach+descend, fewer homes) | 92–93 | ~1.98× (−89 s) | ~11.5 / ~7.8 | PASS (~1–5 mm) |

Final gate (two consistent runs each): parcel **indexed ×2 PASS** (92.5 s, 93.0 s;
6/6 approach+descend motions blended, 0 fallbacks, all parcels upright/stable/
footprint-ok) and the **legacy colour pipeline ×2 PASS** (3/3 placed each) — the
global speed/settle changes did not regress the original sorter.

Phase 3 mechanism: moveit_py exposes no Cartesian-path or Pilz-sequence binding,
so blending is done by planning each leg (still collision-checked), concatenating
the paths, and re-timing the whole with TOTG (`apply_totg_time_parameterization`)
on a group-aware trajectory container. TOTG zeroes velocity only at the two ends,
so the arm flows through the pre-grasp / pre-place corner instead of stopping.
Any failure returns None and falls back to the original per-segment moves.
Also removed the redundant per-slot re-home (task-level 'ready' homes 8 → 5).

Stops per parcel now: grasp, lift, release, retract (≈4, down from ~10), with the
approach-to-grasp and approach-to-place legs continuous. Deliberate perception
homes (one post-place, clearing the camera + staging the next cycle) remain.

Notes:
- FEED→STOPPED (belt transport) ≈ 10 s is simulated physics, unchanged by motion
  tuning.
- Phase 1 validated: `/tmp/motion-p1-indexed` indexed ×1 PASS, exit 245, 0 log
  errors; parcels at (0.261,−0.362), (0.381,−0.362), (0.320,−0.272), all upright.
  Speed/accel tunable live via `TWIN_VEL_SCALE` / `TWIN_ACC_SCALE`; settle cap via
  `TWIN_SETTLE_CAP`.
